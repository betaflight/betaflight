set(MCU_VARIANT stm32h573xx)
set(JLINK_DEVICE stm32h573ii)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    STM32H573xx
    )
endfunction()
