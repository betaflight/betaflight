set(MCU_VARIANT stm32wb55xx)
set(JLINK_DEVICE STM32WB55RG)

set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/stm32wb55xx_flash_cm4.ld)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    STM32WB55xx
    )
endfunction()
