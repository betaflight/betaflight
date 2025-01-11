set(JLINK_DEVICE ATSAMG55J19)
set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/samg55j19_flash.ld)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    __SAMG55J19__
    )
endfunction()
