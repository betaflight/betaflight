include_guard()

set(SDK_DIR ${TOP}/hw/mcu/nxp/mcux-sdk)
set(CMSIS_DIR ${TOP}/lib/CMSIS_5)

# include board specific
include(${CMAKE_CURRENT_LIST_DIR}/boards/${BOARD}/board.cmake)

# toolchain set up
if (MCU_VARIANT STREQUAL "MCXA153")
  set(CMAKE_SYSTEM_PROCESSOR cortex-m33-nodsp-nofp CACHE INTERNAL "System Processor")
  set(FAMILY_MCUS MCXA15 CACHE INTERNAL "")
elseif (MCU_VARIANT STREQUAL "MCXN947")
  set(CMAKE_SYSTEM_PROCESSOR cortex-m33 CACHE INTERNAL "System Processor")
  set(FAMILY_MCUS MCXN9 CACHE INTERNAL "")
else()
  message(FATAL_ERROR "MCU_VARIANT not supported")
endif()

set(CMAKE_TOOLCHAIN_FILE ${TOP}/examples/build_system/cmake/toolchain/arm_${TOOLCHAIN}.cmake)

#------------------------------------
# BOARD_TARGET
#------------------------------------
# only need to be built ONCE for all examples
function(add_board_target BOARD_TARGET)
  if (TARGET ${BOARD_TARGET})
    return()
  endif()

  if (NOT DEFINED LD_FILE_GNU)
    set(LD_FILE_GNU ${SDK_DIR}/devices/${MCU_VARIANT}/gcc/${MCU_CORE}_flash.ld)
  endif ()
  set(LD_FILE_Clang ${LD_FILE_GNU})

  if (NOT DEFINED STARTUP_FILE_GNU)
    set(STARTUP_FILE_GNU ${SDK_DIR}/devices/${MCU_VARIANT}/gcc/startup_${MCU_CORE}.S)
  endif()
  set(STARTUP_FILE_Clang ${STARTUP_FILE_GNU})

  add_library(${BOARD_TARGET} STATIC
    ${STARTUP_FILE_${CMAKE_C_COMPILER_ID}}
    # driver
    ${SDK_DIR}/devices/${MCU_VARIANT}/drivers/fsl_gpio.c
    ${SDK_DIR}/devices/${MCU_VARIANT}/drivers/fsl_common_arm.c
    ${SDK_DIR}/devices/${MCU_VARIANT}/drivers/fsl_lpuart.c
    # mcu
    ${SDK_DIR}/devices/${MCU_VARIANT}/drivers/fsl_clock.c
    ${SDK_DIR}/devices/${MCU_VARIANT}/drivers/fsl_reset.c
    ${SDK_DIR}/devices/${MCU_VARIANT}/system_${MCU_CORE}.c
    )
  target_include_directories(${BOARD_TARGET} PUBLIC
    ${CMSIS_DIR}/CMSIS/Core/Include
    ${SDK_DIR}/devices/${MCU_VARIANT}
    ${SDK_DIR}/devices/${MCU_VARIANT}/drivers
    )

  if (${FAMILY_MCUS} STREQUAL "MCXN9")
    target_sources(${BOARD_TARGET} PRIVATE
      ${SDK_DIR}/devices/${MCU_VARIANT}/drivers/fsl_lpflexcomm.c
    )
  elseif(${FAMILY_MCUS} STREQUAL "MCXA15")
    target_sources(${BOARD_TARGET} PRIVATE
    ${SDK_DIR}/devices/${MCU_VARIANT}/drivers/fsl_spc.c
  )
  endif()

  update_board(${BOARD_TARGET})

  if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    target_link_options(${BOARD_TARGET} PUBLIC
      "LINKER:--script=${LD_FILE_GNU}"
      --specs=nosys.specs --specs=nano.specs
      #-nostartfiles
      )
  elseif (CMAKE_C_COMPILER_ID STREQUAL "Clang")
    target_link_options(${BOARD_TARGET} PUBLIC
      "LINKER:--script=${LD_FILE_Clang}"
      )
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
  if (${FAMILY_MCUS} STREQUAL "MCXN9")
    family_add_tinyusb(${TARGET} OPT_MCU_MCXN9 ${RTOS})
  elseif(${FAMILY_MCUS} STREQUAL "MCXA15")
    family_add_tinyusb(${TARGET} OPT_MCU_MCXA15 ${RTOS})
  endif()

  target_sources(${TARGET}-tinyusb PUBLIC
    # TinyUSB: Port0 is chipidea FS, Port1 is chipidea HS
    ${TOP}/src/portable/chipidea/$<IF:${PORT},ci_hs/dcd_ci_hs.c,ci_fs/dcd_ci_fs.c>
    )
  target_link_libraries(${TARGET}-tinyusb PUBLIC board_${BOARD})

  # Link dependencies
  target_link_libraries(${TARGET} PUBLIC board_${BOARD} ${TARGET}-tinyusb)

  # Flashing
  family_add_bin_hex(${TARGET})
  family_flash_jlink(${TARGET})
  #family_flash_nxplink(${TARGET})
  #family_flash_pyocd(${TARGET})
endfunction()
