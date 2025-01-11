include_guard()

set(ST_FAMILY f1)
set(ST_PREFIX stm32${ST_FAMILY}xx)

set(ST_HAL_DRIVER ${TOP}/hw/mcu/st/stm32${ST_FAMILY}xx_hal_driver)
set(ST_CMSIS ${TOP}/hw/mcu/st/cmsis_device_${ST_FAMILY})
set(CMSIS_5 ${TOP}/lib/CMSIS_5)

# include board specific
include(${CMAKE_CURRENT_LIST_DIR}/boards/${BOARD}/board.cmake)

# toolchain set up
set(CMAKE_SYSTEM_PROCESSOR cortex-m3 CACHE INTERNAL "System Processor")
set(CMAKE_TOOLCHAIN_FILE ${TOP}/examples/build_system/cmake/toolchain/arm_${TOOLCHAIN}.cmake)

set(FAMILY_MCUS STM32F1 CACHE INTERNAL "")


#------------------------------------
# BOARD_TARGET
#------------------------------------
# only need to be built ONCE for all examples
function(add_board_target BOARD_TARGET)
  if (TARGET ${BOARD_TARGET})
    return()
  endif()

# Startup & Linker script
  set(STARTUP_FILE_GNU ${ST_CMSIS}/Source/Templates/gcc/startup_${MCU_VARIANT}.s)
  set(STARTUP_FILE_Clang ${STARTUP_FILE_GNU})
  set(STARTUP_FILE_IAR ${ST_CMSIS}/Source/Templates/iar/startup_${MCU_VARIANT}.s)

  set(LD_FILE_Clang ${LD_FILE_GNU})
  if (NOT DEFINED LD_FILE_IAR)
    set(LD_FILE_IAR ${ST_CMSIS}/Source/Templates/iar/linker/${MCU_VARIANT}_flash.icf)
  endif ()

  add_library(${BOARD_TARGET} STATIC
    ${ST_CMSIS}/Source/Templates/system_${ST_PREFIX}.c
    ${ST_HAL_DRIVER}/Src/${ST_PREFIX}_hal.c
    ${ST_HAL_DRIVER}/Src/${ST_PREFIX}_hal_cortex.c
    ${ST_HAL_DRIVER}/Src/${ST_PREFIX}_hal_rcc.c
    ${ST_HAL_DRIVER}/Src/${ST_PREFIX}_hal_rcc_ex.c
    ${ST_HAL_DRIVER}/Src/${ST_PREFIX}_hal_gpio.c
    ${ST_HAL_DRIVER}/Src/${ST_PREFIX}_hal_uart.c
    ${STARTUP_FILE_${CMAKE_C_COMPILER_ID}}
    )
  target_include_directories(${BOARD_TARGET} PUBLIC
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}
    ${CMSIS_5}/CMSIS/Core/Include
    ${ST_CMSIS}/Include
    ${ST_HAL_DRIVER}/Inc
    )
  #target_compile_options(${BOARD_TARGET} PUBLIC)
  #target_compile_definitions(${BOARD_TARGET} PUBLIC)

  update_board(${BOARD_TARGET})

  if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    target_link_options(${BOARD_TARGET} PUBLIC
      "LINKER:--script=${LD_FILE_GNU}"
      -nostartfiles
      --specs=nosys.specs --specs=nano.specs
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
  family_add_tinyusb(${TARGET} OPT_MCU_STM32F1 ${RTOS})
  target_sources(${TARGET}-tinyusb PUBLIC
    ${TOP}/src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c
    )
  target_link_libraries(${TARGET}-tinyusb PUBLIC board_${BOARD})

  # Link dependencies
  target_link_libraries(${TARGET} PUBLIC board_${BOARD} ${TARGET}-tinyusb)

  # Flashing
  family_add_bin_hex(${TARGET})
  family_flash_stlink(${TARGET})
  family_flash_jlink(${TARGET})
endfunction()
