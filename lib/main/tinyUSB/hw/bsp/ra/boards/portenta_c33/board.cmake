set(CMAKE_SYSTEM_PROCESSOR cortex-m33 CACHE INTERNAL "System Processor")
set(MCU_VARIANT ra6m5)

set(JLINK_DEVICE R7FA6M5BH)
set(DFU_UTIL_VID_PID 2341:0368)

# device default to PORT 1 High Speed
if (NOT DEFINED RHPORT_DEVICE)
  set(RHPORT_DEVICE 1)
endif()
if (NOT DEFINED RHPORT_HOST)
  set(RHPORT_HOST 0)
endif()

function(update_board TARGET)
endfunction()
