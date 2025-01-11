include_guard()

include(${CMAKE_CURRENT_LIST_DIR}/boards/${BOARD}/board.cmake)

string(REPLACE "mm32f" "MM32F" MCU_VARIANT_UPPER ${MCU_VARIANT})
set(SDK_DIR ${TOP}/hw/mcu/mindmotion/mm32sdk/${MCU_VARIANT_UPPER})
set(CMSIS_5 ${TOP}/lib/CMSIS_5)

# toolchain set up
set(CMAKE_SYSTEM_PROCESSOR cortex-m3 CACHE INTERNAL "System Processor")
set(CMAKE_TOOLCHAIN_FILE ${TOP}/examples/build_system/cmake/toolchain/arm_${TOOLCHAIN}.cmake)

set(FAMILY_MCUS MM32F327X CACHE INTERNAL "")


#------------------------------------
# BOARD_TARGET
#------------------------------------
# only need to be built ONCE for all examples
function(add_board_target BOARD_TARGET)
  if (TARGET ${BOARD_TARGET})
    return()
  endif()

  # Startup & Linker script
  set(STARTUP_FILE_GNU ${SDK_DIR}/Source/GCC_StartAsm/startup_${MCU_VARIANT}_gcc.s)
  set(STARTUP_FILE_Clang ${STARTUP_FILE_GNU})
  set(STARTUP_FILE_IAR ${SDK_DIR}/Source/IAR_StartAsm/startup_${MCU_VARIANT}_iar.s)

  set(LD_FILE_Clang ${LD_FILE_GNU})
  # set(LD_FILE_IAR )

  add_library(${BOARD_TARGET} STATIC
    ${SDK_DIR}/Source/system_${MCU_VARIANT}.c
    ${SDK_DIR}/HAL_Lib/Src/hal_gpio.c
    ${SDK_DIR}/HAL_Lib/Src/hal_rcc.c
    ${SDK_DIR}/HAL_Lib/Src/hal_uart.c
    ${STARTUP_FILE_${CMAKE_C_COMPILER_ID}}
    )
  target_include_directories(${BOARD_TARGET} PUBLIC
    ${CMSIS_5}/CMSIS/Core/Include
    ${SDK_DIR}/Include
    ${SDK_DIR}/HAL_Lib/Inc
    )

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
  family_add_tinyusb(${TARGET} OPT_MCU_MM32F327X ${RTOS})
  target_sources(${TARGET}-tinyusb PUBLIC
    ${TOP}/src/portable/mindmotion/mm32/dcd_mm32f327x_otg.c
    )
  target_link_libraries(${TARGET}-tinyusb PUBLIC board_${BOARD})

  # Link dependencies
  target_link_libraries(${TARGET} PUBLIC board_${BOARD} ${TARGET}-tinyusb)

  # Flashing
  family_add_bin_hex(${TARGET})
  family_flash_jlink(${TARGET})
endfunction()
