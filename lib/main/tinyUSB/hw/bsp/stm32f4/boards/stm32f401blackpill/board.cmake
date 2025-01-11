set(MCU_VARIANT stm32f401xc)
set(JLINK_DEVICE stm32f401cc)

set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/STM32F401VCTx_FLASH.ld)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    STM32F405xx
    )
endfunction()
