CFLAGS += -D__SAMD21J18A__ -DCFG_EXAMPLE_VIDEO_READONLY

# All source paths should be relative to the top level.
LD_FILE = $(BOARD_PATH)/samd21j18a_flash.ld

# For flash-jlink target
JLINK_DEVICE = ATSAMD21J18

# flash using jlink
flash: flash-jlink
