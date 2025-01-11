set(SAM_FAMILY same54)

set(JLINK_DEVICE ATSAME54P20)
set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/same54p20a_flash.ld)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    __SAME54P20A__
    )
endfunction()
