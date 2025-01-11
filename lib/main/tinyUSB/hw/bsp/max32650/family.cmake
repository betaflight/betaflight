include_guard()

set(MAX32_PERIPH ${TOP}/hw/mcu/analog/max32/Libraries/PeriphDrivers)
set(MAX32_CMSIS ${TOP}/hw/mcu/analog/max32/Libraries/CMSIS)
set(CMSIS_5 ${TOP}/lib/CMSIS_5)

# include board specific information and functions
include(${CMAKE_CURRENT_LIST_DIR}/boards/${BOARD}/board.cmake)

# Get the linker file
set(LD_FILE_Clang ${LD_FILE_GNU})

# toolchain set up
set(CMAKE_SYSTEM_PROCESSOR cortex-m4 CACHE INTERNAL "System Processor")
set(CMAKE_TOOLCHAIN_FILE ${TOP}/examples/build_system/cmake/toolchain/arm_${TOOLCHAIN}.cmake)
set(JLINK_DEVICE max32650)
set(OPENOCD_OPTION "-f interface/cmsis-dap.cfg -f target/max32650.cfg")

set(FAMILY_MCUS MAX32650 CACHE INTERNAL "")

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    TARGET=MAX32650
    TARGET_REV=0x4131
    MXC_ASSERT_ENABLE
    MAX32650
    IAR_PRAGMAS=0
    CFG_TUSB_MCU=OPT_MCU_MAX32650
    BOARD_TUD_MAX_SPEED=OPT_MODE_HIGH_SPEED
    )

    # Run any board specific updates
    update_board_extras(${TARGET})
endfunction()

#------------------------------------
# BOARD_TARGET
#------------------------------------
# only need to be built ONCE for all examples
function(add_board_target BOARD_TARGET)
  if (TARGET ${BOARD_TARGET})
    return()
  endif ()

  # Startup & Linker script
  set(STARTUP_FILE_GNU ${MAX32_CMSIS}/Device/Maxim/MAX32650/Source/GCC/startup_max32650.S)
  set(STARTUP_FILE_Clang ${STARTUP_FILE_GNU})

  set(PERIPH_SRC ${MAX32_PERIPH}/Source)
  add_library(${BOARD_TARGET} STATIC
    ${MAX32_CMSIS}/Device/Maxim/MAX32650/Source/heap.c
    ${MAX32_CMSIS}/Device/Maxim/MAX32650/Source/header_MAX32650.c
    ${MAX32_CMSIS}/Device/Maxim/MAX32650/Source/system_max32650.c
    ${PERIPH_SRC}/SYS/mxc_assert.c
    ${PERIPH_SRC}/SYS/mxc_delay.c
    ${PERIPH_SRC}/SYS/mxc_lock.c
    ${PERIPH_SRC}/SYS/nvic_table.c
    ${PERIPH_SRC}/SYS/pins_me10.c
    ${PERIPH_SRC}/SYS/sys_me10.c
    ${PERIPH_SRC}/TPU/tpu_me10.c
    ${PERIPH_SRC}/TPU/tpu_reva.c
    ${PERIPH_SRC}/FLC/flc_common.c
    ${PERIPH_SRC}/FLC/flc_me10.c
    ${PERIPH_SRC}/FLC/flc_reva.c
    ${PERIPH_SRC}/GPIO/gpio_common.c
    ${PERIPH_SRC}/GPIO/gpio_me10.c
    ${PERIPH_SRC}/GPIO/gpio_reva.c
    ${PERIPH_SRC}/ICC/icc_me10.c
    ${PERIPH_SRC}/ICC/icc_reva.c
    ${PERIPH_SRC}/ICC/icc_common.c
    ${PERIPH_SRC}/UART/uart_common.c
    ${PERIPH_SRC}/UART/uart_me10.c
    ${PERIPH_SRC}/UART/uart_reva.c
    ${STARTUP_FILE_${CMAKE_C_COMPILER_ID}}
    )
  target_include_directories(${BOARD_TARGET} PUBLIC
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}
    ${MAX32_CMSIS}/Include
    ${MAX32_CMSIS}/Device/Maxim/MAX32650/Include
    ${MAX32_PERIPH}/Include/MAX32650
    ${PERIPH_SRC}/SYS
    ${PERIPH_SRC}/GPIO
    ${PERIPH_SRC}/TPU
    ${PERIPH_SRC}/ICC
    ${PERIPH_SRC}/FLC
    ${PERIPH_SRC}/UART
    )

  target_compile_options(${BOARD_TARGET} PRIVATE
    -Wno-error=strict-prototypes
  )
  update_board(${BOARD_TARGET})

  if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    target_link_options(${BOARD_TARGET} PUBLIC
      "LINKER:--script=${LD_FILE_GNU}"
      -nostartfiles
      --specs=nosys.specs --specs=nano.specs
      -u sb_header #Needed when linking libraries to not lose the Signing header
      )
  elseif (CMAKE_C_COMPILER_ID STREQUAL "Clang")
    target_link_options(${BOARD_TARGET} PUBLIC
      "LINKER:--script=${LD_FILE_Clang}"
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
  family_add_tinyusb(${TARGET} OPT_MCU_MAX32650 ${RTOS})
  target_sources(${TARGET}-tinyusb PUBLIC
    ${TOP}/src/portable/mentor/musb/dcd_musb.c
    )
  target_compile_options(${TARGET} PRIVATE
    -Wno-error=strict-prototypes
    )

    target_link_libraries(${TARGET}-tinyusb PUBLIC board_${BOARD})
  target_compile_options(${TARGET}-tinyusb PRIVATE
    -Wno-error=strict-prototypes
    )

  # Link dependencies
  target_link_libraries(${TARGET} PUBLIC board_${BOARD} ${TARGET}-tinyusb)

  # Flashing
  family_add_bin_hex(${TARGET})
  family_flash_jlink(${TARGET})
  family_flash_openocd_adi(${TARGET})

  # Add the optional MSDK OpenOCD flashing
  family_flash_msdk(${TARGET})
endfunction()

function(family_flash_msdk TARGET)
  # Prepare the image (signed) if the board requires it
  prepare_image(${TARGET})

  set(MAXIM_PATH "$ENV{MAXIM_PATH}")
  add_custom_target(${TARGET}-msdk
    DEPENDS ${TARGET}
    COMMAND ${MAXIM_PATH}/Tools/OpenOCD/openocd -s ${MAXIM_PATH}/Tools/OpenOCD/scripts
            -f interface/cmsis-dap.cfg -f target/max32650.cfg
            -c "program $<TARGET_FILE:${TARGET}> verify; init; reset; exit"
    VERBATIM
  )
endfunction()
