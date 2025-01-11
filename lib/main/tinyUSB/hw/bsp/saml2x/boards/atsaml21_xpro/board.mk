SAM_FAMILY = saml21

CFLAGS += -D__SAML21J18B__

# All source paths should be relative to the top level.
LD_FILE = $(BOARD_PATH)/saml21j18b_flash.ld

# For flash-jlink target
JLINK_DEVICE = ATSAML21J18

# flash using jlink
flash: flash-jlink
