set(MCU_VARIANT stm32h723xx)
set(JLINK_DEVICE stm32h723zg)

set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/../../linker/${MCU_VARIANT}_flash.ld)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    STM32H723xx
    HSE_VALUE=8000000
    )
endfunction()
