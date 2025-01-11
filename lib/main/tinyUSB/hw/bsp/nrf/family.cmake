include_guard()

set(NRFX_DIR ${TOP}/hw/mcu/nordic/nrfx)
set(CMSIS_DIR ${TOP}/lib/CMSIS_5)

# include board specific
include(${CMAKE_CURRENT_LIST_DIR}/boards/${BOARD}/board.cmake)

# toolchain set up
if (MCU_VARIANT STREQUAL "nrf5340_application")
  set(CMAKE_SYSTEM_PROCESSOR cortex-m33 CACHE INTERNAL "System Processor")
  set(JLINK_DEVICE nrf5340_xxaa_app)
else ()
  set(CMAKE_SYSTEM_PROCESSOR cortex-m4 CACHE INTERNAL "System Processor")
  set(JLINK_DEVICE ${MCU_VARIANT}_xxaa)
endif ()

set(CMAKE_TOOLCHAIN_FILE ${TOP}/examples/build_system/cmake/toolchain/arm_${TOOLCHAIN}.cmake)

set(FAMILY_MCUS NRF5X CACHE INTERNAL "")


#------------------------------------
# BOARD_TARGET
#------------------------------------
# only need to be built ONCE for all examples
function(add_board_target BOARD_TARGET)
  if (TARGET ${BOARD_TARGET})
    return()
  endif ()

  if (MCU_VARIANT STREQUAL "nrf5340_application")
    set(MCU_VARIANT_XXAA "nrf5340_xxaa_application")
  else ()
    set(MCU_VARIANT_XXAA "${MCU_VARIANT}_xxaa")
  endif ()

  if (NOT DEFINED LD_FILE_GNU)
    set(LD_FILE_GNU ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/linker/${MCU_VARIANT_XXAA}.ld)
  endif ()

  if (NOT DEFINED STARTUP_FILE_${CMAKE_C_COMPILER_ID})
    set(STARTUP_FILE_GNU ${NRFX_DIR}/mdk/gcc_startup_${MCU_VARIANT}.S)
    set(STARTUP_FILE_Clang ${STARTUP_FILE_GNU})
  endif ()

  add_library(${BOARD_TARGET} STATIC
    ${NRFX_DIR}/helpers/nrfx_flag32_allocator.c
    ${NRFX_DIR}/drivers/src/nrfx_gpiote.c
    ${NRFX_DIR}/drivers/src/nrfx_power.c
    ${NRFX_DIR}/drivers/src/nrfx_spim.c
    ${NRFX_DIR}/drivers/src/nrfx_uarte.c
    ${NRFX_DIR}/mdk/system_${MCU_VARIANT}.c
    ${NRFX_DIR}/soc/nrfx_atomic.c
    ${STARTUP_FILE_${CMAKE_C_COMPILER_ID}}
    )
  string(TOUPPER "${MCU_VARIANT_XXAA}" MCU_VARIANT_XXAA_UPPER)
  target_compile_definitions(${BOARD_TARGET} PUBLIC
    __STARTUP_CLEAR_BSS
    CONFIG_GPIO_AS_PINRESET
    ${MCU_VARIANT_XXAA_UPPER}
    )

  if (TRACE_ETM STREQUAL "1")
    # ENABLE_TRACE will cause system_nrf5x.c to set up ETM trace
    target_compile_definitions(${BOARD_TARGET} PUBLIC ENABLE_TRACE)
  endif ()

  target_include_directories(${BOARD_TARGET} PUBLIC
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}
    ${NRFX_DIR}
    ${NRFX_DIR}/mdk
    ${NRFX_DIR}/hal
    ${NRFX_DIR}/drivers/include
    ${NRFX_DIR}/drivers/src
    ${CMSIS_DIR}/CMSIS/Core/Include
    )

  update_board(${BOARD_TARGET})

  if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    target_link_options(${BOARD_TARGET} PUBLIC
      "LINKER:--script=${LD_FILE_GNU}"
      -L${NRFX_DIR}/mdk
      --specs=nosys.specs --specs=nano.specs
      -nostartfiles
      )
  elseif (CMAKE_C_COMPILER_ID STREQUAL "Clang")
    target_link_options(${BOARD_TARGET} PUBLIC
      "LINKER:--script=${LD_FILE_GNU}"
      -L${NRFX_DIR}/mdk
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

#function(family_flash_adafruit_nrfutil TARGET)
#  add_custom_target(${TARGET}-adafruit-nrfutil
#    DEPENDS ${TARGET}
#    COMMAND adafruit-nrfutil --verbose dfu serial --package $^ -p /dev/ttyACM0 -b 115200 --singlebank --touch 1200
#    )
#endfunction()


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
  family_add_tinyusb(${TARGET} OPT_MCU_NRF5X ${RTOS})
  target_sources(${TARGET}-tinyusb PUBLIC
    ${TOP}/src/portable/nordic/nrf5x/dcd_nrf5x.c
    )
  target_link_libraries(${TARGET}-tinyusb PUBLIC board_${BOARD})

  # Link dependencies
  target_link_libraries(${TARGET} PUBLIC board_${BOARD} ${TARGET}-tinyusb)

  # Flashing
  family_add_bin_hex(${TARGET})
  family_flash_jlink(${TARGET})
#  family_flash_adafruit_nrfutil(${TARGET})
endfunction()
