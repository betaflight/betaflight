set(MCU_VARIANT stm32l412xx)
set(JLINK_DEVICE stm32l412kb)

set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/STM32L412KBUx_FLASH.ld)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    STM32L412xx
    )
endfunction()
