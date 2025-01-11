set(MCU_VARIANT LPC54628)
set(MCU_CORE LPC54628)

set(JLINK_DEVICE LPC54628J512)
set(PYOCD_TARGET LPC54628)
set(NXPLINK_DEVICE LPC54628:LPCXpresso54628)

set(LD_FILE_GNU ${SDK_DIR}/devices/${MCU_VARIANT}/gcc/LPC54628J512_flash.ld)

# Device port default to PORT1 Highspeed
if (NOT DEFINED PORT)
  set(PORT 1)
endif()

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    CPU_LPC54628J512ET180
    )
  target_link_libraries(${TARGET} PUBLIC
    ${SDK_DIR}/devices/${MCU_VARIANT}/gcc/libpower_hardabi.a
    )
endfunction()
