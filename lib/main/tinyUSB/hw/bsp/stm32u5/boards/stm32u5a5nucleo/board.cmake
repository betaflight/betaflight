set(MCU_VARIANT stm32u5a5xx)
set(JLINK_DEVICE stm32u5a5zj)

set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/STM32U5A5ZJTXQ_FLASH.ld)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    STM32U5A5xx
    HSE_VALUE=16000000UL
    )
endfunction()
