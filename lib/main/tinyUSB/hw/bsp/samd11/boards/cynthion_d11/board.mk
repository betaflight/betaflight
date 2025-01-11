BOARD_REVISION_MAJOR ?= 1
BOARD_REVISION_MINOR ?= 0

CFLAGS += -D__SAMD11D14AM__ \
	-D_BOARD_REVISION_MAJOR_=$(BOARD_REVISION_MAJOR) \
	-D_BOARD_REVISION_MINOR_=$(BOARD_REVISION_MINOR)

# All source paths should be relative to the top level.
LD_FILE = $(BOARD_PATH)/cynthion_d11.ld

# Default bootloader size is now 2K, allow to specify other
ifeq ($(BOOTLOADER_SIZE), )
	BOOTLOADER_SIZE := 0x800
endif
LDFLAGS += -Wl,--defsym=BOOTLOADER_SIZE=$(BOOTLOADER_SIZE)

# For flash-jlink target
JLINK_DEVICE = ATSAMD11D14

# flash using dfu-util
flash: $(BUILD)/$(PROJECT).bin
	dfu-util -a 0 -d 1d50:615c -D $< || dfu-util -a 0 -d 16d0:05a5 -D $<
