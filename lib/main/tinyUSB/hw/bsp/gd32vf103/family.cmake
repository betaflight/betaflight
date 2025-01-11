include_guard()

set(SDK_DIR ${TOP}/hw/mcu/gd/nuclei-sdk)
set(SOC_DIR ${SDK_DIR}/SoC/gd32vf103)

# include board specific
include(${CMAKE_CURRENT_LIST_DIR}/boards/${BOARD}/board.cmake)

# toolchain set up
set(CMAKE_SYSTEM_PROCESSOR rv32imac-ilp32 CACHE INTERNAL "System Processor")
set(CMAKE_TOOLCHAIN_FILE ${TOP}/examples/build_system/cmake/toolchain/riscv_${TOOLCHAIN}.cmake)

set(FAMILY_MCUS GD32VF103 CACHE INTERNAL "")

set(JLINK_IF jtag)

#------------------------------------
# BOARD_TARGET
#------------------------------------
# only need to be built ONCE for all examples
function(add_board_target BOARD_TARGET)
  if (TARGET ${BOARD_TARGET})
    return()
  endif()

  if (NOT DEFINED LD_FILE_GNU)
    message(FATAL_ERROR "LD_FILE_GNU is not defined")
  endif ()
  set(LD_FILE_Clang ${LD_FILE_GNU})

  if (NOT DEFINED STARTUP_FILE_GNU)
    set(STARTUP_FILE_GNU
      ${SOC_DIR}/Common/Source/GCC/startup_gd32vf103.S
      ${SOC_DIR}/Common/Source/GCC/intexc_gd32vf103.S
      )
  endif ()
  set(STARTUP_FILE_Clang ${STARTUP_FILE_GNU})

  add_library(${BOARD_TARGET} STATIC
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/system_gd32vf103.c
    ${SOC_DIR}/Common/Source/Drivers/gd32vf103_rcu.c
    ${SOC_DIR}/Common/Source/Drivers/gd32vf103_gpio.c
    ${SOC_DIR}/Common/Source/Drivers/Usb/gd32vf103_usb_hw.c
    ${SOC_DIR}/Common/Source/Drivers/gd32vf103_usart.c
    ${STARTUP_FILE_${CMAKE_C_COMPILER_ID}}
    )
  target_include_directories(${BOARD_TARGET} PUBLIC
    ${SDK_DIR}/NMSIS/Core/Include
    ${SOC_DIR}/Common/Include
    ${SOC_DIR}/Common/Include/Usb
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/boards/${BOARD}
    )
  target_compile_definitions(${BOARD_TARGET} PUBLIC
    DOWNLOAD_MODE=DOWNLOAD_MODE_FLASHXIP
    )

  update_board(${BOARD_TARGET})

  if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    target_compile_options(${BOARD_TARGET} PUBLIC
      -mcmodel=medlow
      -mstrict-align
      )
    target_link_options(${BOARD_TARGET} PUBLIC
      "LINKER:--script=${LD_FILE_GNU}"
      -nostartfiles
      )
  elseif (CMAKE_C_COMPILER_ID STREQUAL "Clang")
    message(FATAL_ERROR "Clang is not supported for MSP432E4")
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
    ${SOC_DIR}/Common/Source/Stubs/sbrk.c
    ${SOC_DIR}/Common/Source/Stubs/close.c
    ${SOC_DIR}/Common/Source/Stubs/isatty.c
    ${SOC_DIR}/Common/Source/Stubs/fstat.c
    ${SOC_DIR}/Common/Source/Stubs/lseek.c
    ${SOC_DIR}/Common/Source/Stubs/read.c
    )
  target_include_directories(${TARGET} PUBLIC
    # family, hw, board
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../../
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/boards/${BOARD}
    )

  # Add TinyUSB target and port source
  family_add_tinyusb(${TARGET} OPT_MCU_GD32VF103 ${RTOS})
  target_sources(${TARGET}-tinyusb PUBLIC
    ${TOP}/src/portable/synopsys/dwc2/dcd_dwc2.c
    ${TOP}/src/portable/synopsys/dwc2/hcd_dwc2.c
    ${TOP}/src/portable/synopsys/dwc2/dwc2_common.c
    )
  target_link_libraries(${TARGET}-tinyusb PUBLIC board_${BOARD})

  # Link dependencies
  target_link_libraries(${TARGET} PUBLIC board_${BOARD} ${TARGET}-tinyusb)

  # Flashing
  family_add_bin_hex(${TARGET})
  family_flash_jlink(${TARGET})
endfunction()
