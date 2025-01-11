set(MCU_VARIANT msp430f5529)
set(LD_FILE_GNU ${SDK_DIR}/msp430f5529.ld)

function(update_board TARGET)
  target_compile_definitions(${TARGET} INTERFACE
    __MSP430F5529__
    )

endfunction()
