SAM_FAMILY = same54

CFLAGS += -D__SAME54P20A__

# All source paths should be relative to the top level.
LD_FILE = $(BOARD_PATH)/same54p20a_flash.ld

# For flash-jlink target
JLINK_DEVICE = ATSAME54P20
