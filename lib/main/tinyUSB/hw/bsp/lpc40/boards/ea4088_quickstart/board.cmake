set(JLINK_DEVICE LPC4088)
set(PYOCD_TARGET LPC4088)
set(NXPLINK_DEVICE LPC4088:LPC4088)

set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/lpc4088.ld)

function(update_board TARGET)
  # nothing to do
endfunction()
