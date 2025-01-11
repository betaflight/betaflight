set(JLINK_DEVICE atsamd21g17a)
set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/samd21g17a_flash.ld)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    __SAMD21G17A__
    CFG_EXAMPLE_MSC_READONLY
    CFG_EXAMPLE_VIDEO_READONLY
    )
endfunction()
