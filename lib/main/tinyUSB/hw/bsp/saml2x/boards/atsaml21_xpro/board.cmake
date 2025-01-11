set(SAM_FAMILY saml21)
set(JLINK_DEVICE ATSAML21J18)
set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/saml21j18b_flash.ld)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    __SAML21J18B__
    )
endfunction()
