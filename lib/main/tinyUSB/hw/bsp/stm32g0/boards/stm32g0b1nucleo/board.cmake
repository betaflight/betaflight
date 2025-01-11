set(MCU_VARIANT stm32g0b1xx)
set(JLINK_DEVICE stm32g0b1re)

set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/STM32G0B1RETx_FLASH.ld)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    STM32G0B1xx
    )
endfunction()
