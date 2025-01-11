set(MCU_VARIANT stm32u545xx)
set(JLINK_DEVICE stm32u545re)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    STM32U545xx
    )
endfunction()
