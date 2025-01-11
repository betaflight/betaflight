set(LPC_FAMILY 11u6x)
set(JLINK_DEVICE LPC11U68)
set(PYOCD_TARGET LPC11U68)

set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/lpc11u68.ld)

function(update_board TARGET)
  target_sources(${TARGET} PUBLIC
    ${SDK_DIR}/src/gpio_${LPC_FAMILY}.c
    ${SDK_DIR}/src/syscon_${LPC_FAMILY}.c
    )
endfunction()
