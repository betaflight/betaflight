set(MCU_VARIANT stm32c071xx)
set(JLINK_DEVICE stm32c071rb)

set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/STM32C071RBTx_FLASH.ld)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    STM32C071xx
    )
endfunction()
