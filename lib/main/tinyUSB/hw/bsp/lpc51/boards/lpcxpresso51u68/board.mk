MCU = LPC51U68

CFLAGS += \
  -DCPU_LPC51U68JBD64 \
  -DCFG_TUSB_MEM_SECTION='__attribute__((section(".data")))'

JLINK_DEVICE = LPC51U68
PYOCD_TARGET = LPC51U68

# flash using pyocd (51u68 is not supported yet)
flash: flash-pyocd
