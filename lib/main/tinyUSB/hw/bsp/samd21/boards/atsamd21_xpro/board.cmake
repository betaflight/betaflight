set(JLINK_DEVICE ATSAMD21J18)
set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/samd21j18a_flash.ld)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    __SAMD21J18A__
    CFG_EXAMPLE_VIDEO_READONLY
    )
endfunction()
