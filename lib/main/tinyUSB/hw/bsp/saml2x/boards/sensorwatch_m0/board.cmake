set(SAM_FAMILY saml22)
set(JLINK_DEVICE ATSAML21J18)
set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/${BOARD}.ld)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    __SAML22J18A__
    )
endfunction()
