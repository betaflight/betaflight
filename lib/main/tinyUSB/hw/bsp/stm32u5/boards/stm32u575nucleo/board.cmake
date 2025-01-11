set(MCU_VARIANT stm32u575xx)
set(JLINK_DEVICE stm32u575zi)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    STM32U575xx
    )
endfunction()
