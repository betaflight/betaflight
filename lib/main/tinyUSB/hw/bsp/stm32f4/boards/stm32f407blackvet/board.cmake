set(MCU_VARIANT stm32f407xx)
set(JLINK_DEVICE stm32f407ve)

set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/STM32F407VETx_FLASH.ld)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    STM32F407xx
    HSE_VALUE=8000000
    )
endfunction()
