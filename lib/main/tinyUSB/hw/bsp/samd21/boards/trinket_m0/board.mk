CFLAGS += -D__SAMD21E18A__ -DCFG_EXAMPLE_VIDEO_READONLY

# All source paths should be relative to the top level.
LD_FILE = $(BOARD_PATH)/trinket_m0.ld
