# Use the standard, non-secure linker file
set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/max32650.ld)

function(update_board_extras TARGET)
  #No extra arguments
endfunction()

function(prepare_image TARGET_IN)
  #No signing required
endfunction()
