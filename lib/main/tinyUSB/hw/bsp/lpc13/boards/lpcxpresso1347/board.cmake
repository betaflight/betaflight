set(JLINK_DEVICE LPC1347)
set(PYOCD_TARGET LPC1347)

set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/lpc1347.ld)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    CFG_EXAMPLE_MSC_READONLY
    CFG_EXAMPLE_VIDEO_READONLY
    )
endfunction()
