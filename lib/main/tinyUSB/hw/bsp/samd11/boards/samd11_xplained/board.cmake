set(JLINK_DEVICE ATSAMD11D14)
set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/samd11d14am_flash.ld)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    __SAMD11D14AM__
    )
endfunction()
