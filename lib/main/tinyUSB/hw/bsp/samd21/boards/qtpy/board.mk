# For Adafruit QT Py board

CFLAGS += -D__SAMD21E18A__ -DCFG_EXAMPLE_VIDEO_READONLY

# All source paths should be relative to the top level.
LD_FILE = $(BOARD_PATH)/$(BOARD).ld

# For flash-jlink target
JLINK_DEVICE = ATSAMD21E18

flash: flash-bossac
