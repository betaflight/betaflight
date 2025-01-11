include_guard()

if (NOT BOARD)
  message(FATAL_ERROR "BOARD not specified")
endif ()

set(SDK_DIR ${TOP}/hw/mcu/nxp/mcux-sdk)
set(CMSIS_DIR ${TOP}/lib/CMSIS_5)

# include board specific
include(${CMAKE_CURRENT_LIST_DIR}/boards/${BOARD}/board.cmake)

# toolchain set up
set(CMAKE_SYSTEM_PROCESSOR cortex-m0plus CACHE INTERNAL "System Processor")
set(CMAKE_TOOLCHAIN_FILE ${TOP}/examples/build_system/cmake/toolchain/arm_${TOOLCHAIN}.cmake)

set(FAMILY_MCUS KINETIS_KL CACHE INTERNAL "")


#------------------------------------
# BOARD_TARGET
#------------------------------------
# only need to be built ONCE for all examples
function(add_board_target BOARD_TARGET)
  if (TARGET ${BOARD_TARGET})
    return()
  endif ()

  # LD_FILE and STARTUP_FILE can be defined in board.cmake
  set(LD_FILE_Clang ${LD_FILE_GNU})
  set(STARTUP_FILE_Clang ${STARTUP_FILE_GNU})

  add_library(${BOARD_TARGET} STATIC
    ${STARTUP_FILE_${CMAKE_C_COMPILER_ID}}
    ${SDK_DIR}/drivers/gpio/fsl_gpio.c
    ${SDK_DIR}/drivers/lpsci/fsl_lpsci.c
    ${SDK_DIR}/drivers/uart/fsl_uart.c
    ${SDK_DIR}/devices/${MCU_VARIANT}/drivers/fsl_clock.c
    ${SDK_DIR}/devices/${MCU_VARIANT}/system_${MCU_VARIANT}.c
    )
  target_compile_definitions(${BOARD_TARGET} PUBLIC
    __STARTUP_CLEAR_BSS
    )
  target_include_directories(${BOARD_TARGET} PUBLIC
    ${CMSIS_DIR}/CMSIS/Core/Include
    ${SDK_DIR}/devices/${MCU_VARIANT}
    ${SDK_DIR}/devices/${MCU_VARIANT}/drivers
    ${SDK_DIR}/drivers/common
    ${SDK_DIR}/drivers/gpio
    ${SDK_DIR}/drivers/lpsci
    ${SDK_DIR}/drivers/port
    ${SDK_DIR}/drivers/smc
    ${SDK_DIR}/drivers/uart
    )
  update_board(${BOARD_TARGET})

  if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    target_link_options(${BOARD_TARGET} PUBLIC
      "LINKER:--script=${LD_FILE_GNU}"
      --specs=nosys.specs --specs=nano.specs
      -nostartfiles
      )
  elseif (CMAKE_C_COMPILER_ID STREQUAL "Clang")
    target_link_options(${BOARD_TARGET} PUBLIC
      "LINKER:--script=${LD_FILE_GNU}"
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
  family_add_tinyusb(${TARGET} OPT_MCU_KINETIS_KL ${RTOS})
  target_sources(${TARGET}-tinyusb PUBLIC
    ${TOP}/src/portable/chipidea/ci_fs/dcd_ci_fs.c
    ${TOP}/src/portable/nxp/khci/hcd_khci.c
    )
  target_link_libraries(${TARGET}-tinyusb PUBLIC board_${BOARD})

  # Link dependencies
  target_link_libraries(${TARGET} PUBLIC board_${BOARD} ${TARGET}-tinyusb)

  # Flashing
  family_add_bin_hex(${TARGET})
  family_flash_jlink(${TARGET})
endfunction()
