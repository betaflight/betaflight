set(MCU_SUB_VARIANT 123)

set(JLINK_DEVICE TM4C123GH6PM)
set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/tm4c123.ld)

set(OPENOCD_OPTION "-f board/ti_ek-tm4c123gxl.cfg")
set(UNIFLASH_OPTION "-c ${CMAKE_CURRENT_LIST_DIR}/${BOARD}.ccxml -r 1")

function(update_board TARGET)
  target_compile_definitions(${TARGET} PUBLIC
    TM4C123GH6PM
    )
endfunction()
