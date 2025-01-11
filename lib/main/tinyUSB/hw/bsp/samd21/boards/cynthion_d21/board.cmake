set(JLINK_DEVICE ATSAMD21G18)
set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/samd21g18a_flash.ld)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    __SAMD21G18A__
    CFG_EXAMPLE_VIDEO_READONLY
    )
  target_link_options(${TARGET} PUBLIC
    "LINKER:--defsym=BOOTLOADER_SIZE=0x800"
    )
endfunction()
