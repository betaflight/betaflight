set(MCU_VARIANT stm32f746xx)
set(JLINK_DEVICE stm32f746xx)

set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/STM32F746ZGTx_FLASH.ld)

if (NOT DEFINED RHPORT_DEVICE)
  set(RHPORT_DEVICE 0)
endif()
if (NOT DEFINED RHPORT_HOST)
  set(RHPORT_HOST 0)
endif()

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    STM32F746xx
    HSE_VALUE=8000000
    )
endfunction()
