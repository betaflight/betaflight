CFLAGS += -D__SAMD11D14AM__

# All source paths should be relative to the top level.
LD_FILE = $(BOARD_PATH)/samd11d14am_flash.ld

# For flash-jlink target
JLINK_DEVICE = ATSAMD11D14

# flash using edbg
flash: $(BUILD)/$(PROJECT).bin
	edbg -b -t samd11 -e -pv -f $<
