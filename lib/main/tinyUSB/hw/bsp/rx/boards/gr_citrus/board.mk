CFLAGS += \
  -mcpu=rx610 \
  -misa=v1 \
  -DCFG_TUSB_MCU=OPT_MCU_RX63X

MCU_DIR = hw/mcu/renesas/rx/rx63n

# All source paths should be relative to the top level.
LD_FILE = $(BOARD_PATH)/r5f5631fd.ld

# For freeRTOS port source
FREERTOS_PORTABLE_SRC = $(FREERTOS_PORTABLE_PATH)/RX600

# For flash-jlink target
JLINK_DEVICE = R5F5631F
JLINK_IF     = JTAG

# For flash-pyocd target
PYOCD_TARGET =

# flash using jlink
flash: flash-jlink
