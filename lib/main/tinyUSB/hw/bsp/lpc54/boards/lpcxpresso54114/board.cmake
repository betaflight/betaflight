set(MCU_VARIANT LPC54114)
set(MCU_CORE LPC54114_cm4)

set(JLINK_DEVICE LPC54114J256_M4)
set(PYOCD_TARGET LPC54114)

set(LD_FILE_GNU ${SDK_DIR}/devices/${MCU_VARIANT}/gcc/LPC54114J256_cm4_flash.ld)

# Device port default to PORT1 Highspeed
if (NOT DEFINED PORT)
  set(PORT 1)
endif()

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    CPU_LPC54114J256BD64_cm4
    )
  target_link_libraries(${TARGET} PUBLIC
    ${SDK_DIR}/devices/${MCU_VARIANT}/gcc/libpower_cm4_hardabi.a
    )
endfunction()
