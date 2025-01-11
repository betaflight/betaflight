set(MCU_VARIANT stm32g491xx)
set(JLINK_DEVICE stm32g491re)

set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/STM32G491RETX_FLASH.ld)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    STM32G491xx
    HSE_VALUE=24000000
    )
endfunction()
