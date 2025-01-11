set(CMAKE_SYSTEM_PROCESSOR cortex-m85 CACHE INTERNAL "System Processor")
set(MCU_VARIANT ra8m1)

set(JLINK_DEVICE R7FA8M1AH)
#set(JLINK_OPTION "-USB 001083115236")

# device default to PORT 1 High Speed
if (NOT DEFINED RHPORT_DEVICE)
  set(RHPORT_DEVICE 1)
endif()
if (NOT DEFINED RHPORT_HOST)
  set(RHPORT_HOST 0)
endif()

function(update_board TARGET)
endfunction()
