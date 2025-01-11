set(JLINK_DEVICE ATSAMD21G18)
set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/${BOARD}.ld)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    __SAMD21G18A__
    CFG_EXAMPLE_VIDEO_READONLY
    )
endfunction()
