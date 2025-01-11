set(MCU_VARIANT stm32h563xx)
set(JLINK_DEVICE stm32h563zi)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    STM32H563xx
    HSE_VALUE=8000000
    )
endfunction()
