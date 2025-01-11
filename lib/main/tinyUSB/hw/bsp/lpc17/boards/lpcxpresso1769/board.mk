# All source paths should be relative to the top level.
LD_FILE = $(BOARD_PATH)/lpc1769.ld

# For flash-jlink target
JLINK_DEVICE = LPC1769

# flash using jlink
flash: flash-jlink
