set(MCU_VARIANT LPC4357)

set(JLINK_DEVICE LPC4357_M4)
set(PYOCD_TARGET LPC4357)
set(NXPLINK_DEVICE LPC4357:LPC4357)

set(LD_FILE_GNU ${CMAKE_CURRENT_LIST_DIR}/lpc4357.ld)

function(update_board TARGET)
  # EA4357 use I2C GPIO expander for LED
  target_sources(${TARGET} PRIVATE
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/pca9532.c
    ${SDK_DIR}/src/i2c_18xx_43xx.c
    ${SDK_DIR}/src/i2cm_18xx_43xx.c
    )
endfunction()
