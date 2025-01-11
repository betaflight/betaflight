set(MCU_VARIANT stm32f103xb)
set(JLINK_DEVICE stm32f103rc)

set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/STM32F103XC_FLASH.ld)
set(LD_FILE_IAR ${CMAKE_CURRENT_LIST_DIR}/stm32f103xc_flash.icf)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    STM32F103xB
    HSE_VALUE=8000000U
    )
endfunction()
