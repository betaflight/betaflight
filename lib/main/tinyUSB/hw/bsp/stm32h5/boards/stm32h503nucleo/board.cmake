set(MCU_VARIANT stm32h503xx)
set(JLINK_DEVICE stm32h503rb)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    STM32H503xx
    HSE_VALUE=24000000
    )
endfunction()
