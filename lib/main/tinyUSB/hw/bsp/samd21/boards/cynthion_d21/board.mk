CFLAGS += -D__SAMD21G18A__ -DCFG_EXAMPLE_VIDEO_READONLY

LD_FILE = $(BOARD_PATH)/samd21g18a_flash.ld

# Default bootloader size is now 2K, allow to specify other
ifeq ($(BOOTLOADER_SIZE), )
	BOOTLOADER_SIZE := 0x800
endif
LDFLAGS += -Wl,--defsym=BOOTLOADER_SIZE=$(BOOTLOADER_SIZE)

# For flash-jlink target
JLINK_DEVICE = ATSAMD21G18

# flash using dfu-util
flash: $(BUILD)/$(PROJECT).bin
	dfu-util -a 0 -d 1d50:615c -D $< || dfu-util -a 0 -d 16d0:05a5 -D $<
