set(MCU_VARIANT LPC55S69)
set(MCU_CORE LPC55S69_cm33_core0)

set(JLINK_DEVICE LPC55S69_M33_0)
set(PYOCD_TARGET LPC55S69)
set(NXPLINK_DEVICE LPC55S69:LPCXpresso55S69)

# Device port default to PORT1 Highspeed
if (NOT DEFINED PORT)
  set(PORT 1)
endif()

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    CPU_LPC55S69JBD100_cm33_core0
    )
endfunction()
