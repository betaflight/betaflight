set(MCU_VARIANT stm32h750xx)
set(JLINK_DEVICE stm32h750ibk6_m7)

set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/stm32h750ibkx_flash.ld)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    STM32H750xx
    HSE_VALUE=16000000
    CORE_CM7
    )
endfunction()
