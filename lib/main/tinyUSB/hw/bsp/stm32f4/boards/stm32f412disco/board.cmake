set(MCU_VARIANT stm32f412zx)
set(JLINK_DEVICE stm32f412zg)
# set(JLINK_OPTION "-USB 000771775987")

set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/STM32F412ZGTx_FLASH.ld)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    STM32F412Zx
    )
endfunction()
