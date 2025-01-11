set(MCU_VARIANT stm32f070xb)
set(JLINK_DEVICE stm32f070rb)

set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/stm32F070rbtx_flash.ld)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    STM32F070xB
    CFG_EXAMPLE_VIDEO_READONLY
    )
endfunction()
