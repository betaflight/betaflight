set(MCU_VARIANT stm32f072xb)
set(JLINK_DEVICE stm32f072rb)

set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/STM32F072RBTx_FLASH.ld)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    STM32F072xB
    CFG_EXAMPLE_VIDEO_READONLY
    )
endfunction()
