set(SAM_FAMILY samd51)

set(JLINK_DEVICE ATSAMD51J19)
set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/${BOARD}.ld)

# force max3421e for testing with hardware-in-the-loop
set(MAX3421_HOST 1)

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    __SAMD51J19A__
    )
endfunction()
