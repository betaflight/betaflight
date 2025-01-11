include_guard()

set(SDK_DIR ${TOP}/hw/mcu/nxp/lpcopen/lpc40xx/lpc_chip_40xx)
set(CMSIS_DIR ${TOP}/lib/CMSIS_5)

# include board specific
include(${CMAKE_CURRENT_LIST_DIR}/boards/${BOARD}/board.cmake)

# toolchain set up
set(CMAKE_SYSTEM_PROCESSOR cortex-m4 CACHE INTERNAL "System Processor")
set(CMAKE_TOOLCHAIN_FILE ${TOP}/examples/build_system/cmake/toolchain/arm_${TOOLCHAIN}.cmake)

set(FAMILY_MCUS LPC40XX CACHE INTERNAL "")


#------------------------------------
# BOARD_TARGET
#------------------------------------
# only need to be built ONCE for all examples
function(add_board_target BOARD_TARGET)
  if (TARGET ${BOARD_TARGET})
    return()
  endif ()

  add_library(${BOARD_TARGET} STATIC
    ${SDK_DIR}/../gcc/cr_startup_lpc40xx.c
    ${SDK_DIR}/src/chip_17xx_40xx.c
    ${SDK_DIR}/src/clock_17xx_40xx.c
    ${SDK_DIR}/src/fpu_init.c
    ${SDK_DIR}/src/gpio_17xx_40xx.c
    ${SDK_DIR}/src/iocon_17xx_40xx.c
    ${SDK_DIR}/src/sysctl_17xx_40xx.c
    ${SDK_DIR}/src/sysinit_17xx_40xx.c
    ${SDK_DIR}/src/uart_17xx_40xx.c
    )
  target_compile_definitions(${BOARD_TARGET} PUBLIC
    __USE_LPCOPEN
    CORE_M4
    CFG_TUSB_MEM_SECTION=__attribute__\(\(section\(\".data.$RAM2\"\)\)\)
    )
  target_include_directories(${BOARD_TARGET} PUBLIC
    ${SDK_DIR}/inc
    ${CMSIS_DIR}/CMSIS/Core/Include
    )

  update_board(${BOARD_TARGET})

  if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    target_compile_options(${BOARD_TARGET} PUBLIC -nostdlib)
    target_link_options(${BOARD_TARGET} PUBLIC
      "LINKER:--script=${LD_FILE_GNU}"
      --specs=nosys.specs --specs=nano.specs
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
  family_add_tinyusb(${TARGET} OPT_MCU_LPC40XX ${RTOS})
  target_sources(${TARGET}-tinyusb PUBLIC
    ${TOP}/src/portable/nxp/lpc17_40/dcd_lpc17_40.c
    ${TOP}/src/portable/nxp/lpc17_40/hcd_lpc17_40.c
    ${TOP}/src/portable/ohci/ohci.c
    )
  target_link_libraries(${TARGET}-tinyusb PUBLIC board_${BOARD})

  # Link dependencies
  target_link_libraries(${TARGET} PUBLIC board_${BOARD} ${TARGET}-tinyusb)

  # Flashing
  family_add_bin_hex(${TARGET})
  family_flash_jlink(${TARGET})
  #family_flash_nxplink(${TARGET})
endfunction()
