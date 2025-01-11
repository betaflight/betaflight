set(MCU_VARIANT stm32f103xb)
set(JLINK_DEVICE stm32f103c8)

set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/STM32F103X8_FLASH.ld)
set(LD_FILE_IAR ${CMAKE_CURRENT_LIST_DIR}/stm32f103x8_flash.icf)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    STM32F103xB
    HSE_VALUE=8000000U
    CFG_EXAMPLE_VIDEO_READONLY
    )
endfunction()
