
LD_FILE = $(BOARD_PATH)/lpc4088.ld

# For flash-jlink target
JLINK_DEVICE = LPC4088

# flash using jlink
flash: flash-jlink
