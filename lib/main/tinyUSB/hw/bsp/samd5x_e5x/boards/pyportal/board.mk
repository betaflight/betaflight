SAM_FAMILY = samd51

CFLAGS += -D__SAMD51J19A__

LD_FILE = $(BOARD_PATH)/$(BOARD).ld

# For flash-jlink target
JLINK_DEVICE = ATSAMD51J19

flash: flash-bossac
