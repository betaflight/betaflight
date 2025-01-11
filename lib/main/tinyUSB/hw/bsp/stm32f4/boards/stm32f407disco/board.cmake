set(MCU_VARIANT stm32f407xx)
set(JLINK_DEVICE stm32f407vg)

set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/STM32F407VGTx_FLASH.ld)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    STM32F407xx
    )
endfunction()
