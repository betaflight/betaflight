MCU = 11u6x
MCU_DRV = 11u6x

CFLAGS += \
  -DCORE_M0PLUS \
  -D__VTOR_PRESENT=0 \
  -DCFG_TUSB_MEM_SECTION='__attribute__((section(".data.$$RAM3")))' \

# All source paths should be relative to the top level.
LD_FILE = $(BOARD_PATH)/lpc11u68.ld

# For flash-jlink target
JLINK_DEVICE = LPC11U68
PYOCD_TARGET = lpc11u68

# flash using pyocd
flash: flash-pyocd
