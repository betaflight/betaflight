set(MCU_VARIANT stm32f439xx)
set(JLINK_DEVICE stm32f439zi)

set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/STM32F439ZITX_FLASH.ld)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    STM32F439xx
    BOARD_TUD_RHPORT=0
    )
endfunction()
