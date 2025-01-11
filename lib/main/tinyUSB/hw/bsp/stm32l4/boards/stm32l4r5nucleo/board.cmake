set(MCU_VARIANT stm32l4r5xx)
set(JLINK_DEVICE stm32l4r5zi)

set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/STM32L4RXxI_FLASH.ld)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    STM32L4R5xx
    HSE_VALUE=8000000
    )
endfunction()
