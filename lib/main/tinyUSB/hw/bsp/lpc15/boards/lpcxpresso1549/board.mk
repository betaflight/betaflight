CFLAGS += -DCFG_EXAMPLE_VIDEO_READONLY
LD_FILE = $(BOARD_PATH)/lpc1549.ld

JLINK_DEVICE = LPC1549

# flash using pyocd
flash: flash-jlink
