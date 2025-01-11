set(MCU_VARIANT stm32l053xx)
set(JLINK_DEVICE stm32l053r8)
#set(JLINK_OPTION "-USB 778921770")

set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/STM32L053C8Tx_FLASH.ld)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    STM32L053xx
    )
endfunction()
