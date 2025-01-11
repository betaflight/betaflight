LD_FILE = $(BOARD_PATH)/lpc1837.ld

# For flash-jlink target
JLINK_DEVICE = LPC18S37

flash: flash-jlink
