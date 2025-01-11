set(MCU_VARIANT stm32f723xx)
set(JLINK_DEVICE stm32f723ie)
#set(JLINK_OPTION "-USB 000776606156")
set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/STM32F723xE_FLASH.ld)

set(RHPORT_SPEED OPT_MODE_FULL_SPEED OPT_MODE_HIGH_SPEED)

# For Hardware test: device default to PORT 0, Host to port 1
if (NOT DEFINED RHPORT_DEVICE)
  set(RHPORT_DEVICE 0)
endif()
if (NOT DEFINED RHPORT_HOST)
  set(RHPORT_HOST 1)
endif()

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    STM32F723xx
    HSE_VALUE=25000000
    )
endfunction()
