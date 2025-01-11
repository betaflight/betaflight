set(MCU_VARIANT stm32f405xx)
set(JLINK_DEVICE stm32f405rg)

set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/STM32F405RGTx_FLASH.ld)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    STM32F405xx
    )
endfunction()
