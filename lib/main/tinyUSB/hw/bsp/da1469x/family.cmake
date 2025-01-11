include_guard()

set(MCU_DIR ${TOP}/hw/mcu/dialog/da1469x)

# include board specific
include(${CMAKE_CURRENT_LIST_DIR}/boards/${BOARD}/board.cmake)

set(CMAKE_SYSTEM_PROCESSOR cortex-m33-nodsp CACHE INTERNAL "System Processor")
set(CMAKE_TOOLCHAIN_FILE ${TOP}/examples/build_system/cmake/toolchain/arm_${TOOLCHAIN}.cmake)
set(FAMILY_MCUS DA1469X CACHE INTERNAL "")

#------------------------------------
# BOARD_TARGET
#------------------------------------
# only need to be built ONCE for all examples
function(add_board_target BOARD_TARGET)
  if (TARGET ${BOARD_TARGET})
    return()
  endif ()

  if (NOT DEFINED LD_FILE_GNU)
    set(LD_FILE_GNU ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/linker/da1469x.ld)
  endif ()

  if (NOT DEFINED STARTUP_FILE_${CMAKE_C_COMPILER_ID})
    set(STARTUP_FILE_GNU ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/gcc_startup_da1469x.S)
    set(STARTUP_FILE_Clang ${STARTUP_FILE_GNU})
  endif ()

  add_library(${BOARD_TARGET} STATIC
    ${MCU_DIR}/src/system_da1469x.c
    ${MCU_DIR}/src/da1469x_clock.c
    ${MCU_DIR}/src/hal_gpio.c
    ${STARTUP_FILE_${CMAKE_C_COMPILER_ID}}
    )
  target_compile_options(${BOARD_TARGET} PUBLIC -mthumb-interwork)
  target_compile_definitions(${BOARD_TARGET} PUBLIC
    CORE_M33
    CFG_TUD_ENDPOINT0_SIZE=8
    )
  target_include_directories(${BOARD_TARGET} PUBLIC
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/boards/${BOARD}
    ${MCU_DIR}/include
    ${MCU_DIR}/SDK_10.0.8.105/sdk/bsp/include
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

function(family_flash_jlink_dialog TARGET)
  set(JLINKEXE JLinkExe)
  set(JLINK_IF swd)

  # mkimage from sdk
  set(MKIMAGE $ENV{HOME}/code/tinyusb-mcu-driver/dialog/SDK_10.0.8.105/binaries/mkimage)

  file(GENERATE OUTPUT $<TARGET_FILE_DIR:${TARGET}>/version.h
    CONTENT "#define SW_VERSION \"v_1.0.0.1\"
#define SW_VERSION_DATE \"2024-07-17 17:55\""
    )

  file(GENERATE OUTPUT $<TARGET_FILE_DIR:${TARGET}>/${TARGET}.jlink
    CONTENT "r
halt
loadfile $<TARGET_FILE_DIR:${TARGET}>/${TARGET}-image.bin 0x16000000
r
go
exit"
  )

  add_custom_target(${TARGET}-image
    DEPENDS ${TARGET}
    COMMAND ${MKIMAGE} da1469x $<TARGET_FILE_DIR:${TARGET}>/${TARGET}.bin $<TARGET_FILE_DIR:${TARGET}>/version.h $<TARGET_FILE_DIR:${TARGET}>/${TARGET}.bin.img
    COMMAND cp ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/product_header.dump $<TARGET_FILE_DIR:${TARGET}>/${TARGET}-image.bin
    COMMAND cat $<TARGET_FILE_DIR:${TARGET}>/${TARGET}.bin.img >> $<TARGET_FILE_DIR:${TARGET}>/${TARGET}-image.bin
    )
  add_custom_target(${TARGET}-jlink
    DEPENDS ${TARGET}-image
    COMMAND ${JLINKEXE} -device ${JLINK_DEVICE} -if ${JLINK_IF} -JTAGConf -1,-1 -speed auto -CommandFile $<TARGET_FILE_DIR:${TARGET}>/${TARGET}.jlink
    )
endfunction()


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
  family_add_tinyusb(${TARGET} OPT_MCU_DA1469X ${RTOS})
  target_sources(${TARGET}-tinyusb PUBLIC
    ${TOP}/src/portable/dialog/da146xx/dcd_da146xx.c
    )
  target_link_libraries(${TARGET}-tinyusb PUBLIC board_${BOARD})

  # Link dependencies
  target_link_libraries(${TARGET} PUBLIC board_${BOARD} ${TARGET}-tinyusb)

  # Flashing
  family_add_bin_hex(${TARGET})
  family_flash_jlink_dialog(${TARGET})
endfunction()
