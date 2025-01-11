include_guard()

set(SDK_DIR ${TOP}/hw/mcu/ti/msp430/msp430-gcc-support-files/include)

# include board specific
include(${CMAKE_CURRENT_LIST_DIR}/boards/${BOARD}/board.cmake)

# toolchain set up
set(CMAKE_SYSTEM_PROCESSOR msp430 CACHE INTERNAL "System Processor")
set(CMAKE_TOOLCHAIN_FILE ${TOP}/examples/build_system/cmake/toolchain/msp430_${TOOLCHAIN}.cmake)

set(FAMILY_MCUS MSP430x5xx CACHE INTERNAL "")


#------------------------------------
# BOARD_TARGET
#------------------------------------
# only need to be built ONCE for all examples
function(add_board_target BOARD_TARGET)
  if (NOT TARGET ${BOARD_TARGET})
    add_library(${BOARD_TARGET} INTERFACE)
    target_compile_definitions(${BOARD_TARGET} INTERFACE
      CFG_TUD_ENDPOINT0_SIZE=8
      CFG_EXAMPLE_VIDEO_READONLY
      CFG_EXAMPLE_MSC_READONLY
      )
    target_include_directories(${BOARD_TARGET} INTERFACE
      ${CMAKE_CURRENT_FUNCTION_LIST_DIR}
      ${SDK_DIR}
      )

    update_board(${BOARD_TARGET})

    if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
      target_link_options(${BOARD_TARGET} INTERFACE
        "LINKER:--script=${LD_FILE_GNU}"
        -L${SDK_DIR}
        )
    elseif (CMAKE_C_COMPILER_ID STREQUAL "IAR")
      target_link_options(${BOARD_TARGET} INTERFACE
        "LINKER:--config=${LD_FILE_IAR}"
        )
    endif ()
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
  family_add_tinyusb(${TARGET} OPT_MCU_MSP430x5xx ${RTOS})
  target_sources(${TARGET}-tinyusb PUBLIC
    ${TOP}/src/portable/ti/msp430x5xx/dcd_msp430x5xx.c
    )
  target_link_libraries(${TARGET}-tinyusb PUBLIC board_${BOARD})

  # Link dependencies
  target_link_libraries(${TARGET} PUBLIC board_${BOARD} ${TARGET}-tinyusb)

  # Flashing
  family_add_bin_hex(${TARGET})
  family_flash_msp430flasher(${TARGET})
endfunction()
