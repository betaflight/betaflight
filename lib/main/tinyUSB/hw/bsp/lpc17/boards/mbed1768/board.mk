# All source paths should be relative to the top level.
LD_FILE = $(BOARD_PATH)/lpc1768.ld

# For flash-jlink target
JLINK_DEVICE = LPC1768
PYOCD_TARGET = lpc1768

# flash using pyocd
flash: flash-pyocd
