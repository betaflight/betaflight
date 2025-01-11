set(MCU_VARIANT stm32u585xx)
set(JLINK_DEVICE stm32u585zi)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    STM32U585xx
    )
endfunction()
