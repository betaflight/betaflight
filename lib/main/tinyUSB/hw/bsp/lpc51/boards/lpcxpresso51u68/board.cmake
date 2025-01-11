set(MCU_VARIANT LPC51U68)

set(JLINK_DEVICE LPC51U68)
set(PYOCD_TARGET LPC51U68)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    CPU_LPC51U68JBD64
    )
  target_link_libraries(${TARGET} PUBLIC
    ${SDK_DIR}/devices/${MCU_VARIANT}/gcc/libpower.a
    )
endfunction()
