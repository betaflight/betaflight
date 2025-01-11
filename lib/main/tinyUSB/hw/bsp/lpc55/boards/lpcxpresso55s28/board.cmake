set(MCU_VARIANT LPC55S28)
set(MCU_CORE LPC55S28)

set(JLINK_DEVICE LPC55S28)
set(PYOCD_TARGET LPC55S28)
set(NXPLINK_DEVICE LPC55S28:LPCXpresso55S28)

# Device port default to PORT1 Highspeed
if (NOT DEFINED PORT)
  set(PORT 1)
endif()

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    CPU_LPC55S28JBD100
    )
endfunction()
