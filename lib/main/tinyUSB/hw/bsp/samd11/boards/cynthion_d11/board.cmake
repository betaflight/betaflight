set(JLINK_DEVICE ATSAMD11D14)
set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/cynthion_d11.ld)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    __SAMD11D14AM__
    _BOARD_REVISION_MAJOR_=1
    _BOARD_REVISION_MINOR_=0
    )

  target_link_options(${BOARD_TARGET} PUBLIC
    -Wl,--defsym=BOOTLOADER_SIZE=0x800
    )
endfunction()
