DEPS_SUBMODULES += hw/mcu/nxp/lpcopen

CFLAGS += \
  -DCFG_TUSB_MEM_SECTION='__attribute__((section(".data.$$RAM2")))'

# All source paths should be relative to the top level.
LD_FILE = $(BOARD_PATH)/lpc1347.ld

# For flash-jlink target
JLINK_DEVICE = LPC1347

# flash using jlink
flash: flash-jlink
