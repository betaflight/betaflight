set(MCU_VARIANT stm32f411xe)
set(JLINK_DEVICE stm32f411ve)

set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/STM32F411VETx_FLASH.ld)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    STM32F411xE
    )
endfunction()
