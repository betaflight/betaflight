set(MCU_VARIANT msp432e401y)
set(JLINK_DEVICE ${MCU_VARIANT})

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    __MSP432E401Y__
    )
endfunction()
