include_guard()

set(UF2_FAMILY_ID 0x699b62ec)
set(CH32_FAMILY ch32v20x)
set(SDK_DIR ${TOP}/hw/mcu/wch/${CH32_FAMILY})
set(SDK_SRC_DIR ${SDK_DIR}/EVT/EXAM/SRC)

# include board specific
include(${CMAKE_CURRENT_LIST_DIR}/boards/${BOARD}/board.cmake)

# toolchain set up
set(CMAKE_SYSTEM_PROCESSOR rv32imac-ilp32 CACHE INTERNAL "System Processor")
set(CMAKE_TOOLCHAIN_FILE ${TOP}/examples/build_system/cmake/toolchain/riscv_${TOOLCHAIN}.cmake)

set(FAMILY_MCUS CH32V20X CACHE INTERNAL "")
set(OPENOCD_OPTION "-f ${CMAKE_CURRENT_LIST_DIR}/wch-riscv.cfg")

# Port0 use FSDev, Port1 use USBFS
if (NOT DEFINED PORT)
  set(PORT 0)
endif()

#------------------------------------
# BOARD_TARGET
#------------------------------------
# only need to be built ONCE for all examples
function(add_board_target BOARD_TARGET)
  if (TARGET ${BOARD_TARGET})
    return()
  endif()

  if (NOT DEFINED LD_FILE_GNU)
    set(LD_FILE_GNU ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/linker/${CH32_FAMILY}.ld)
  endif ()
  set(LD_FILE_Clang ${LD_FILE_GNU})

  if (NOT DEFINED STARTUP_FILE_GNU)
    set(STARTUP_FILE_GNU ${SDK_SRC_DIR}/Startup/startup_${CH32_FAMILY}_${MCU_VARIANT}.S)
  endif ()
  set(STARTUP_FILE_Clang ${STARTUP_FILE_GNU})

  add_library(${BOARD_TARGET} STATIC
    ${SDK_SRC_DIR}/Core/core_riscv.c
    ${SDK_SRC_DIR}/Peripheral/src/${CH32_FAMILY}_flash.c
    ${SDK_SRC_DIR}/Peripheral/src/${CH32_FAMILY}_gpio.c
    ${SDK_SRC_DIR}/Peripheral/src/${CH32_FAMILY}_misc.c
    ${SDK_SRC_DIR}/Peripheral/src/${CH32_FAMILY}_rcc.c
    ${SDK_SRC_DIR}/Peripheral/src/${CH32_FAMILY}_usart.c
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/system_${CH32_FAMILY}.c
    ${STARTUP_FILE_${CMAKE_C_COMPILER_ID}}
    )
  target_include_directories(${BOARD_TARGET} PUBLIC
    ${SDK_SRC_DIR}/Core
    ${SDK_SRC_DIR}/Peripheral/inc
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}
    )
  target_compile_definitions(${BOARD_TARGET} PUBLIC
    CH32V20x_${MCU_VARIANT}
    )

  if (PORT EQUAL 0)
    target_compile_definitions(${BOARD_TARGET} PUBLIC
      CFG_TUD_WCH_USBIP_FSDEV=1
      )
  elseif (PORT EQUAL 1)
    target_compile_definitions(${BOARD_TARGET} PUBLIC
      CFG_TUD_WCH_USBIP_USBFS=1
      )
  else()
    message(FATAL_ERROR "Invalid PORT ${PORT}")
  endif()

  update_board(${BOARD_TARGET})

  if (LD_FLASH_SIZE STREQUAL 224K)
    target_compile_definitions(${BOARD_TARGET} PUBLIC
      CH32_FLASH_ENHANCE_READ_MODE=1
      )
  endif()

  if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    target_compile_options(${BOARD_TARGET} PUBLIC
      -mcmodel=medany
      )
    target_link_options(${BOARD_TARGET} PUBLIC
      -nostartfiles
      --specs=nosys.specs --specs=nano.specs
      -Wl,--defsym=__FLASH_SIZE=${LD_FLASH_SIZE}
      -Wl,--defsym=__RAM_SIZE=${LD_RAM_SIZE}
      "LINKER:--script=${LD_FILE_GNU}"
      )
  elseif (CMAKE_C_COMPILER_ID STREQUAL "Clang")
    message(FATAL_ERROR "Clang is not supported for CH32v")
  elseif (CMAKE_C_COMPILER_ID STREQUAL "IAR")
    target_link_options(${BOARD_TARGET} PUBLIC
      "LINKER:--config=${LD_FILE_IAR}"
      )
  endif ()
endfunction()


#------------------------------------
# Functions
#------------------------------------
function(family_configure_example TARGET RTOS)
  family_configure_common(${TARGET} ${RTOS})

  # Board target
  add_board_target(board_${BOARD})

  #---------- Port Specific ----------
  # These files are built for each example since it depends on example's tusb_config.h
  target_sources(${TARGET} PUBLIC
    # BSP
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/family.c
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../board.c
    )
  target_include_directories(${TARGET} PUBLIC
    # family, hw, board
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../../
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/boards/${BOARD}
    )

  # Add TinyUSB target and port source
  family_add_tinyusb(${TARGET} OPT_MCU_CH32V20X ${RTOS})

  target_sources(${TARGET}-tinyusb PUBLIC
    ${TOP}/src/portable/wch/dcd_ch32_usbfs.c
    ${TOP}/src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c
    )
  target_link_libraries(${TARGET}-tinyusb PUBLIC board_${BOARD})

  # Link dependencies
  target_link_libraries(${TARGET} PUBLIC board_${BOARD} ${TARGET}-tinyusb)

  # Flashing
  family_add_bin_hex(${TARGET})
  family_flash_openocd_wch(${TARGET})
  family_flash_wlink_rs(${TARGET})

  #family_add_uf2(${TARGET} ${UF2_FAMILY_ID})
  #family_flash_uf2(${TARGET} ${UF2_FAMILY_ID})
endfunction()
