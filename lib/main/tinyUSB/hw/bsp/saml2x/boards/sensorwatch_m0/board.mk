SAM_FAMILY = saml22

CFLAGS += -D__SAML22J18A__

# All source paths should be relative to the top level.
LD_FILE = $(BOARD_PATH)/$(BOARD).ld

# For flash-jlink target
JLINK_DEVICE = ATSAML22J18

flash: flash-bossac
