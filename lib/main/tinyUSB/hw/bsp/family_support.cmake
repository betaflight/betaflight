include_guard(GLOBAL)

include(CMakePrintHelpers)

# TOP is path to root directory
set(TOP "${CMAKE_CURRENT_LIST_DIR}/../..")
get_filename_component(TOP ${TOP} ABSOLUTE)

set(UF2CONV_PY ${TOP}/tools/uf2/utils/uf2conv.py)

#-------------------------------------------------------------
# Toolchain
# Can be changed via -DTOOLCHAIN=gcc|iar or -DCMAKE_C_COMPILER=
#-------------------------------------------------------------
# Detect toolchain based on CMAKE_C_COMPILER
if (DEFINED CMAKE_C_COMPILER)
  string(FIND ${CMAKE_C_COMPILER} "iccarm" IS_IAR)
  string(FIND ${CMAKE_C_COMPILER} "clang" IS_CLANG)
  string(FIND ${CMAKE_C_COMPILER} "gcc" IS_GCC)

  if (NOT IS_IAR EQUAL -1)
    set(TOOLCHAIN iar)
  elseif (NOT IS_CLANG EQUAL -1)
    set(TOOLCHAIN clang)
  elseif (NOT IS_GCC EQUAL -1)
    set(TOOLCHAIN gcc)
  endif ()
endif ()

# default to gcc
if (NOT DEFINED TOOLCHAIN)
  set(TOOLCHAIN gcc)
endif ()

#-------------------------------------------------------------
# FAMILY and BOARD
#-------------------------------------------------------------
if (NOT DEFINED FAMILY)
  if (NOT DEFINED BOARD)
    message(FATAL_ERROR "You must set a FAMILY variable for the build (e.g. rp2040, espressif).
    You can do this via -DFAMILY=xxx on the cmake command line")
  endif ()

  # Find path contains BOARD
  file(GLOB BOARD_PATH LIST_DIRECTORIES true
    RELATIVE ${TOP}/hw/bsp
    ${TOP}/hw/bsp/*/boards/${BOARD}
    )
  if (NOT BOARD_PATH)
    message(FATAL_ERROR "Could not detect FAMILY from BOARD=${BOARD}")
  endif ()

  # replace / with ; so that we can get the first element as FAMILY
  string(REPLACE "/" ";" BOARD_PATH ${BOARD_PATH})
  list(GET BOARD_PATH 0 FAMILY)
endif ()

if (NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/${FAMILY}/family.cmake)
  message(FATAL_ERROR "Family '${FAMILY}' is not known/supported")
endif()

if (NOT FAMILY STREQUAL rp2040)
  # enable LTO if supported skip rp2040
  include(CheckIPOSupported)
  check_ipo_supported(RESULT IPO_SUPPORTED)
  cmake_print_variables(IPO_SUPPORTED)
  if (IPO_SUPPORTED)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
  endif()
endif()

if (NOT NO_WARN_RWX_SEGMENTS_SUPPORTED)
  set(NO_WARN_RWX_SEGMENTS_SUPPORTED 1)
endif()

set(WARNING_FLAGS_GNU
  -Wall
  -Wextra
  -Werror
  -Wfatal-errors
  -Wdouble-promotion
  -Wstrict-prototypes
  -Wstrict-overflow
  -Werror-implicit-function-declaration
  -Wfloat-equal
  -Wundef
  -Wshadow
  -Wwrite-strings
  -Wsign-compare
  -Wmissing-format-attribute
  -Wunreachable-code
  -Wcast-align
  -Wcast-function-type
  -Wcast-qual
  -Wnull-dereference
  -Wuninitialized
  -Wunused
  -Wreturn-type
  -Wredundant-decls
  )

set(WARNING_FLAGS_IAR "")

#-------------------------------------------------------------
# Functions
#-------------------------------------------------------------

# Filter example based on only.txt and skip.txt
function(family_filter RESULT DIR)
  get_filename_component(DIR ${DIR} ABSOLUTE BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

  if (EXISTS "${DIR}/skip.txt")
    file(STRINGS "${DIR}/skip.txt" SKIPS_LINES)
    foreach(MCU IN LISTS FAMILY_MCUS)
      # For each line in only.txt
      foreach(_line ${SKIPS_LINES})
        # If mcu:xxx exists for this mcu then skip
        if (${_line} STREQUAL "mcu:${MCU}" OR ${_line} STREQUAL "board:${BOARD}" OR ${_line} STREQUAL "family:${FAMILY}")
          set(${RESULT} 0 PARENT_SCOPE)
          return()
        endif()
      endforeach()
    endforeach()
  endif ()

  if (EXISTS "${DIR}/only.txt")
    file(STRINGS "${DIR}/only.txt" ONLYS_LINES)
    foreach(MCU IN LISTS FAMILY_MCUS)
      # For each line in only.txt
      foreach(_line ${ONLYS_LINES})
        # If mcu:xxx exists for this mcu or board:xxx then include
        if (${_line} STREQUAL "mcu:${MCU}" OR ${_line} STREQUAL "board:${BOARD}" OR ${_line} STREQUAL "family:${FAMILY}")
          set(${RESULT} 1 PARENT_SCOPE)
          return()
        endif()
      endforeach()
    endforeach()

    # Didn't find it in only file so don't build
    set(${RESULT} 0 PARENT_SCOPE)
  else()
    # only.txt not exist so build
    set(${RESULT} 1 PARENT_SCOPE)
  endif()
endfunction()

function(family_add_subdirectory DIR)
  family_filter(SHOULD_ADD "${DIR}")
  if (SHOULD_ADD)
    add_subdirectory(${DIR})
  endif()
endfunction()

function(family_get_project_name OUTPUT_NAME DIR)
  get_filename_component(SHORT_NAME ${DIR} NAME)
  set(${OUTPUT_NAME} ${TINYUSB_FAMILY_PROJECT_NAME_PREFIX}${SHORT_NAME} PARENT_SCOPE)
endfunction()

function(family_initialize_project PROJECT DIR)
  # set output suffix to .elf (skip espressif and rp2040)
  if(NOT FAMILY STREQUAL "espressif" AND NOT FAMILY STREQUAL "rp2040")
    set(CMAKE_EXECUTABLE_SUFFIX .elf PARENT_SCOPE)
  endif()

  family_filter(ALLOWED "${DIR}")
  if (NOT ALLOWED)
    get_filename_component(SHORT_NAME ${DIR} NAME)
    message(FATAL_ERROR "${SHORT_NAME} is not supported on FAMILY=${FAMILY}")
  endif()
endfunction()

#-------------------------------------------------------------
# Common Target Configure
# Most families use these settings except rp2040 and espressif
#-------------------------------------------------------------

# Add RTOS to example
function(family_add_rtos TARGET RTOS)
  if (RTOS STREQUAL "freertos")
    # freertos config
    if (NOT TARGET freertos_config)
      add_library(freertos_config INTERFACE)
      target_include_directories(freertos_config INTERFACE ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/${FAMILY}/FreeRTOSConfig)
      # add board definition to freertos_config mostly for SystemCoreClock
      target_link_libraries(freertos_config INTERFACE board_${BOARD})
    endif()

    # freertos kernel
    if (NOT TARGET freertos_kernel)
      add_subdirectory(${TOP}/lib/FreeRTOS-Kernel ${CMAKE_BINARY_DIR}/lib/freertos_kernel)
    endif ()

    target_link_libraries(${TARGET} PUBLIC freertos_kernel)
  endif ()
endfunction()

# Add common configuration to example
function(family_configure_common TARGET RTOS)
  family_add_rtos(${TARGET} ${RTOS})

  string(TOUPPER ${BOARD} BOARD_UPPER)
  string(REPLACE "-" "_" BOARD_UPPER ${BOARD_UPPER})
  target_compile_definitions(${TARGET} PUBLIC
    BOARD_${BOARD_UPPER}
  )

  # compile define from command line
  if(DEFINED CFLAGS_CLI)
    separate_arguments(CFLAGS_CLI)
    target_compile_options(${TARGET} PUBLIC ${CFLAGS_CLI})
  endif()

  target_compile_options(${TARGET} PUBLIC ${WARNING_FLAGS_${CMAKE_C_COMPILER_ID}})

  # Generate linker map file
  if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    target_link_options(${TARGET} PUBLIC "LINKER:-Map=$<TARGET_FILE:${TARGET}>.map")
    if (CMAKE_C_COMPILER_VERSION VERSION_GREATER_EQUAL 12.0 AND NO_WARN_RWX_SEGMENTS_SUPPORTED)
      target_link_options(${TARGET} PUBLIC "LINKER:--no-warn-rwx-segments")
    endif ()
 elseif (CMAKE_C_COMPILER_ID STREQUAL "Clang")
    target_link_options(${TARGET} PUBLIC "LINKER:-Map=$<TARGET_FILE:${TARGET}>.map")
  elseif (CMAKE_C_COMPILER_ID STREQUAL "IAR")
    target_link_options(${TARGET} PUBLIC "LINKER:--map=$<TARGET_FILE:${TARGET}>.map")
  endif()

  # ETM Trace option
  if (TRACE_ETM STREQUAL "1")
    target_compile_definitions(${TARGET} PUBLIC TRACE_ETM)
  endif ()

  # LOGGER option
  if (DEFINED LOGGER)
    target_compile_definitions(${TARGET} PUBLIC LOGGER_${LOGGER})
    # Add segger rtt to example
    if(LOGGER STREQUAL "RTT" OR LOGGER STREQUAL "rtt")
      if (NOT TARGET segger_rtt)
        add_library(segger_rtt STATIC ${TOP}/lib/SEGGER_RTT/RTT/SEGGER_RTT.c)
        target_include_directories(segger_rtt PUBLIC ${TOP}/lib/SEGGER_RTT/RTT)
#        target_compile_definitions(segger_rtt PUBLIC SEGGER_RTT_MODE_DEFAULT=SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL)
      endif()
      target_link_libraries(${TARGET} PUBLIC segger_rtt)
    endif ()
  endif ()

  # run size after build
  find_program(SIZE_EXE ${CMAKE_SIZE})
  if(NOT ${SIZE_EXE} STREQUAL SIZE_EXE-NOTFOUND)
    add_custom_command(TARGET ${TARGET} POST_BUILD
      COMMAND ${SIZE_EXE} $<TARGET_FILE:${TARGET}>
      )
  endif ()
endfunction()

# Add tinyusb to example
function(family_add_tinyusb TARGET OPT_MCU RTOS)
  # tinyusb target is built for each example since it depends on example's tusb_config.h
  set(TINYUSB_TARGET_PREFIX ${TARGET}-)
  add_library(${TARGET}-tinyusb_config INTERFACE)

  # path to tusb_config.h
  target_include_directories(${TARGET}-tinyusb_config INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_compile_definitions(${TARGET}-tinyusb_config INTERFACE CFG_TUSB_MCU=${OPT_MCU})

  if (DEFINED LOG)
    target_compile_definitions(${TARGET}-tinyusb_config INTERFACE CFG_TUSB_DEBUG=${LOG})
    if (LOG STREQUAL "4")
      # no inline for debug level 4
      target_compile_definitions(${TARGET}-tinyusb_config INTERFACE TU_ATTR_ALWAYS_INLINE=)
    endif ()
  endif()

  if (RTOS STREQUAL "freertos")
    target_compile_definitions(${TARGET}-tinyusb_config INTERFACE CFG_TUSB_OS=OPT_OS_FREERTOS)
  endif ()

  # tinyusb's CMakeList.txt
  add_subdirectory(${TOP}/src ${CMAKE_CURRENT_BINARY_DIR}/tinyusb)

  if (RTOS STREQUAL "freertos")
    # link tinyusb with freeRTOS kernel
    target_link_libraries(${TARGET}-tinyusb PUBLIC freertos_kernel)
  endif ()

  # use max3421 as host controller
  if (MAX3421_HOST STREQUAL "1")
    target_compile_definitions(${TARGET}-tinyusb_config INTERFACE CFG_TUH_MAX3421=1)
    target_sources(${TARGET}-tinyusb PUBLIC
      ${TOP}/src/portable/analog/max3421/hcd_max3421.c
      )
  endif ()

  # compile define from command line
  if(DEFINED CFLAGS_CLI)
    separate_arguments(CFLAGS_CLI)
    target_compile_options(${TARGET}-tinyusb PUBLIC ${CFLAGS_CLI})
  endif()

endfunction()

# Add bin/hex output
function(family_add_bin_hex TARGET)
  if (CMAKE_C_COMPILER_ID STREQUAL "IAR")
    add_custom_command(TARGET ${TARGET} POST_BUILD
      COMMAND ${CMAKE_OBJCOPY} --bin $<TARGET_FILE:${TARGET}> $<TARGET_FILE_DIR:${TARGET}>/${TARGET}.bin
      COMMAND ${CMAKE_OBJCOPY} --ihex $<TARGET_FILE:${TARGET}> $<TARGET_FILE_DIR:${TARGET}>/${TARGET}.hex
      VERBATIM)
  else()
    add_custom_command(TARGET ${TARGET} POST_BUILD
      COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${TARGET}> $<TARGET_FILE_DIR:${TARGET}>/${TARGET}.bin
      COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${TARGET}> $<TARGET_FILE_DIR:${TARGET}>/${TARGET}.hex
      VERBATIM)
  endif()
endfunction()

# Add uf2 output
function(family_add_uf2 TARGET FAMILY_ID)
  set(BIN_FILE $<TARGET_FILE_DIR:${TARGET}>/${TARGET}.hex)
  add_custom_command(TARGET ${TARGET} POST_BUILD
    COMMAND python ${UF2CONV_PY} -f ${FAMILY_ID} -c -o $<TARGET_FILE_DIR:${TARGET}>/${TARGET}.uf2 ${BIN_FILE}
    VERBATIM)
endfunction()

#----------------------------------
# Example Target Configure (Default rule)
# These function can be redefined in FAMILY/family.cmake
#----------------------------------

function(family_configure_example TARGET RTOS)
  # empty function, should be redefined in FAMILY/family.cmake
endfunction()

# Configure device example with RTOS
function(family_configure_device_example TARGET RTOS)
  family_configure_example(${TARGET} ${RTOS})
endfunction()

# Configure host example with RTOS
function(family_configure_host_example TARGET RTOS)
  family_configure_example(${TARGET} ${RTOS})
endfunction()

# Configure host + device example with RTOS
function(family_configure_dual_usb_example TARGET RTOS)
  family_configure_example(${TARGET} ${RTOS})
endfunction()

function(family_example_missing_dependency TARGET DEPENDENCY)
  message(WARNING "${DEPENDENCY} submodule needed by ${TARGET} not found, please run 'python tools/get_deps.py ${DEPENDENCY}' to fetch it")
endfunction()

#----------------------------------
# RPI specific: refactor later
#----------------------------------
function(family_add_default_example_warnings TARGET)
  target_compile_options(${TARGET} PUBLIC
    -Wall
    -Wextra
    -Werror
    -Wfatal-errors
    -Wdouble-promotion
    -Wfloat-equal
    # FIXME commented out because of https://github.com/raspberrypi/pico-sdk/issues/1468
    #-Wshadow
    -Wwrite-strings
    -Wsign-compare
    -Wmissing-format-attribute
    -Wunreachable-code
    -Wcast-align
    -Wcast-qual
    -Wnull-dereference
    -Wuninitialized
    -Wunused
    -Wredundant-decls
    #-Wstrict-prototypes
    #-Werror-implicit-function-declaration
    #-Wundef
    )

  if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    if (CMAKE_C_COMPILER_VERSION VERSION_GREATER_EQUAL 12.0 AND NO_WARN_RWX_SEGMENTS_SUPPORTED)
      target_link_options(${TARGET} PUBLIC "LINKER:--no-warn-rwx-segments")
    endif()

    # GCC 10
    if (CMAKE_C_COMPILER_VERSION VERSION_GREATER_EQUAL 10.0)
      target_compile_options(${TARGET} PUBLIC -Wconversion)
    endif()

    # GCC 8
    if (CMAKE_C_COMPILER_VERSION VERSION_GREATER_EQUAL 8.0)
      target_compile_options(${TARGET} PUBLIC -Wcast-function-type -Wstrict-overflow)
    endif()

    # GCC 6
    if (CMAKE_C_COMPILER_VERSION VERSION_GREATER_EQUAL 6.0)
      target_compile_options(${TARGET} PUBLIC -Wno-strict-aliasing)
    endif()
  endif()
endfunction()

#----------------------------------
# Flashing target
#----------------------------------

# Add flash jlink target
function(family_flash_jlink TARGET)
  if (NOT DEFINED JLINKEXE)
    set(JLINKEXE JLinkExe)
  endif ()

  if (NOT DEFINED JLINK_IF)
    set(JLINK_IF swd)
  endif ()

  if (NOT DEFINED JLINK_OPTION)
    set(JLINK_OPTION "")
  endif ()
  separate_arguments(OPTION_LIST UNIX_COMMAND ${JLINK_OPTION})

  file(GENERATE
    OUTPUT $<TARGET_FILE_DIR:${TARGET}>/${TARGET}.jlink
    CONTENT "halt
loadfile $<TARGET_FILE:${TARGET}>
r
go
exit"
    )

  add_custom_target(${TARGET}-jlink
    DEPENDS ${TARGET}
    COMMAND ${JLINKEXE} -device ${JLINK_DEVICE} ${OPTION_LIST} -if ${JLINK_IF} -JTAGConf -1,-1 -speed auto -CommandFile $<TARGET_FILE_DIR:${TARGET}>/${TARGET}.jlink
    VERBATIM
    )

  # optional flash post build
#  add_custom_command(TARGET ${TARGET} POST_BUILD
#    COMMAND ${JLINKEXE} -device ${JLINK_DEVICE} ${OPTION_LIST} -if ${JLINK_IF} -JTAGConf -1,-1 -speed auto -CommandFile $<TARGET_FILE_DIR:${TARGET}>/${TARGET}.jlink
#    VERBATIM
#    )
endfunction()


# Add flash stlink target
function(family_flash_stlink TARGET)
  if (NOT DEFINED STM32_PROGRAMMER_CLI)
    set(STM32_PROGRAMMER_CLI STM32_Programmer_CLI)
  endif ()

  add_custom_target(${TARGET}-stlink
    DEPENDS ${TARGET}
    COMMAND ${STM32_PROGRAMMER_CLI} --connect port=swd --write $<TARGET_FILE:${TARGET}> --go
    )
endfunction()


# Add flash st-flash target
function(family_flash_stflash TARGET)
  if (NOT DEFINED ST_FLASH)
    set(ST_FLASH st-flash)
  endif ()

  add_custom_target(${TARGET}-stflash
    DEPENDS ${TARGET}
    COMMAND ${ST_FLASH} write $<TARGET_FILE_DIR:${TARGET}>/${TARGET}.bin 0x8000000
    )
endfunction()


# Add flash openocd target
function(family_flash_openocd TARGET)
  if (NOT DEFINED OPENOCD)
    set(OPENOCD openocd)
  endif ()

  if (NOT DEFINED OPENOCD_OPTION2)
    set(OPENOCD_OPTION2 "")
  endif ()

  separate_arguments(OPTION_LIST UNIX_COMMAND ${OPENOCD_OPTION})
  separate_arguments(OPTION_LIST2 UNIX_COMMAND ${OPENOCD_OPTION2})

  # note skip verify since it has issue with rp2040
  add_custom_target(${TARGET}-openocd
    DEPENDS ${TARGET}
    COMMAND ${OPENOCD} -c "tcl_port disabled" -c "gdb_port disabled" ${OPTION_LIST} -c init -c halt -c "program $<TARGET_FILE:${TARGET}>" -c reset ${OPTION_LIST2} -c exit
    VERBATIM
    )
endfunction()


# Add flash openocd-wch target
# compiled from https://github.com/hathach/riscv-openocd-wch or https://github.com/dragonlock2/miscboards/blob/main/wch/SDK/riscv-openocd.tar.xz
function(family_flash_openocd_wch TARGET)
  if (NOT DEFINED OPENOCD)
    set(OPENOCD $ENV{HOME}/app/riscv-openocd-wch/src/openocd)
  endif ()

  family_flash_openocd(${TARGET})
endfunction()


# Add flash openocd adi (Analog Devices) target
# included with msdk or compiled from release branch of https://github.com/analogdevicesinc/openocd
function(family_flash_openocd_adi TARGET)
  if (DEFINED $ENV{MAXIM_PATH})
    # use openocd from msdk
    set(OPENOCD ENV{MAXIM_PATH}/Tools/OpenOCD/openocd)
    set(OPENOCD_OPTION2 "-s ENV{MAXIM_PATH}/Tools/OpenOCD/scripts")
  else()
    # compiled from source
    if (NOT DEFINED OPENOCD_ADI_PATH)
      set(OPENOCD_ADI_PATH $ENV{HOME}/app/openocd_adi)
    endif ()
    set(OPENOCD ${OPENOCD_ADI_PATH}/src/openocd)
    set(OPENOCD_OPTION2 "-s ${OPENOCD_ADI_PATH}/tcl")
  endif ()

  family_flash_openocd(${TARGET})
endfunction()

# Add flash with https://github.com/ch32-rs/wlink
function(family_flash_wlink_rs TARGET)
  if (NOT DEFINED WLINK_RS)
    set(WLINK_RS wlink)
  endif ()

  add_custom_target(${TARGET}-wlink-rs
    DEPENDS ${TARGET}
    COMMAND ${WLINK_RS} flash $<TARGET_FILE:${TARGET}>
    )
endfunction()


# Add flash pycod target
function(family_flash_pyocd TARGET)
  if (NOT DEFINED PYOC)
    set(PYOCD pyocd)
  endif ()

  add_custom_target(${TARGET}-pyocd
    DEPENDS ${TARGET}
    COMMAND ${PYOCD} flash -t ${PYOCD_TARGET} $<TARGET_FILE:${TARGET}>
    )
endfunction()


# Flash with UF2
function(family_flash_uf2 TARGET FAMILY_ID)
  add_custom_target(${TARGET}-uf2
    DEPENDS ${TARGET}
    COMMAND python ${UF2CONV_PY} -f ${FAMILY_ID} --deploy $<TARGET_FILE_DIR:${TARGET}>/${TARGET}.uf2
    )
endfunction()


# Add flash teensy_cli target
function(family_flash_teensy TARGET)
  if (NOT DEFINED TEENSY_CLI)
    set(TEENSY_CLI teensy_loader_cli)
  endif ()

  add_custom_target(${TARGET}-teensy
    DEPENDS ${TARGET}
    COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${TARGET}> $<TARGET_FILE_DIR:${TARGET}>/${TARGET}.hex
    COMMAND ${TEENSY_CLI} --mcu=${TEENSY_MCU} -w -s $<TARGET_FILE_DIR:${TARGET}>/${TARGET}.hex
    )
endfunction()


# Add flash using NXP's LinkServer (redserver)
# https://www.nxp.com/design/software/development-software/mcuxpresso-software-and-tools-/linkserver-for-microcontrollers:LINKERSERVER
function(family_flash_nxplink TARGET)
  if (NOT DEFINED LINKSERVER)
    set(LINKSERVER LinkServer)
  endif ()

  # LinkServer has a bug that can only execute with full path otherwise it throws:
  # realpath error: No such file or directory
  execute_process(COMMAND which ${LINKSERVER} OUTPUT_VARIABLE LINKSERVER_PATH OUTPUT_STRIP_TRAILING_WHITESPACE)

  add_custom_target(${TARGET}-nxplink
    DEPENDS ${TARGET}
    COMMAND ${LINKSERVER_PATH} flash ${NXPLINK_DEVICE} load $<TARGET_FILE:${TARGET}>
    )
endfunction()


function(family_flash_dfu_util TARGET OPTION)
  if (NOT DEFINED DFU_UTIL)
    set(DFU_UTIL dfu-util)
  endif ()

  add_custom_target(${TARGET}-dfu-util
    DEPENDS ${TARGET}
    COMMAND ${DFU_UTIL} -R -d ${DFU_UTIL_VID_PID} -a 0 -D $<TARGET_FILE_DIR:${TARGET}>/${TARGET}.bin
    VERBATIM
    )
endfunction()

function(family_flash_msp430flasher TARGET)
  if (NOT DEFINED MSP430Flasher)
    set(MSP430FLASHER MSP430Flasher)
  endif ()

  # set LD_LIBRARY_PATH to find libmsp430.so (directory containing MSP430Flasher)
  find_program(MSP430FLASHER_PATH MSP430Flasher)
  get_filename_component(MSP430FLASHER_PARENT_DIR "${MSP430FLASHER_PATH}" DIRECTORY)
  add_custom_target(${TARGET}-msp430flasher
    DEPENDS ${TARGET}
    COMMAND ${CMAKE_COMMAND} -E env LD_LIBRARY_PATH=${MSP430FLASHER_PARENT_DIR}
            ${MSP430FLASHER} -w $<TARGET_FILE_DIR:${TARGET}>/${TARGET}.hex -z [VCC]
    )
endfunction()


function(family_flash_uniflash TARGET)
  if (NOT DEFINED DSLITE)
    set(DSLITE dslite.sh)
  endif ()

  separate_arguments(OPTION_LIST UNIX_COMMAND ${UNIFLASH_OPTION})

  add_custom_target(${TARGET}-uniflash
    DEPENDS ${TARGET}
    COMMAND ${DSLITE} ${UNIFLASH_OPTION} -f $<TARGET_FILE_DIR:${TARGET}>/${TARGET}.hex
    VERBATIM
    )
endfunction()

#----------------------------------
# Family specific
#----------------------------------

# family specific: can override above functions
include(${CMAKE_CURRENT_LIST_DIR}/${FAMILY}/family.cmake)

if (NOT FAMILY_MCUS)
  set(FAMILY_MCUS ${FAMILY})
endif()

# if use max3421 as host controller, expand FAMILY_MCUS to include max3421
if (MAX3421_HOST STREQUAL "1")
  set(FAMILY_MCUS ${FAMILY_MCUS} MAX3421)
endif ()

# save it in case of re-inclusion
set(FAMILY_MCUS ${FAMILY_MCUS} CACHE INTERNAL "")
