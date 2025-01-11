LD_FILE = $(BOARD_PATH)/lpc1857.ld

# For flash-jlink target
JLINK_DEVICE = LPC1857

# flash using jlink
flash: flash-jlink
