MCU = 11uxx
MCU_DRV = 11xx

CFLAGS += \
  -DCORE_M0 \
  -DCFG_EXAMPLE_MSC_READONLY \
  -DCFG_EXAMPLE_VIDEO_READONLY \
  -DCFG_TUSB_MEM_SECTION='__attribute__((section(".data.$$RAM2")))'

# mcu driver cause following warnings
CFLAGS += \
	-Wno-error=strict-prototypes \
	-Wno-error=unused-parameter \
	-Wno-error=redundant-decls

# All source paths should be relative to the top level.
LD_FILE = $(BOARD_PATH)/lpc11u37.ld

# For flash-jlink target
JLINK_DEVICE = LPC11U37/401
PYOCD_TARGET = lpc11u37

# flash using pyocd
flash: flash-pyocd
