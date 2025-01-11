# EA4357 use I2C GPIO expander for LED
SRC_C += \
  ${BOARD_PATH}/pca9532.c \
	${SDK_DIR}/src/i2c_18xx_43xx.c \
	${SDK_DIR}/src/i2cm_18xx_43xx.c \

LD_FILE = ${BOARD_PATH}/lpc4357.ld

JLINK_DEVICE = LPC4357_M4

flash: flash-jlink
