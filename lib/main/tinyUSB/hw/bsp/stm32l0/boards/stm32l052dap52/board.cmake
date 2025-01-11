set(MCU_VARIANT stm32l052xx)
set(JLINK_DEVICE stm32l052k8)

set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/STM32L052K8Ux_FLASH.ld)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    STM32L052xx
    )
endfunction()
