CFLAGS += -D__SAMD21G17A__ -DCFG_EXAMPLE_MSC_READONLY -DCFG_EXAMPLE_VIDEO_READONLY

# All source paths should be relative to the top level.
LD_FILE = $(BOARD_PATH)/samd21g17a_flash.ld

# For flash-jlink target
JLINK_DEVICE = atsamd21g17a

# flash using jlink (options are: jlink/cmsisdap/stlink/dfu)
#flash: flash-jlink

PYOCD_TARGET = atsamd21g17a
PYOCD_OPTION = -O dap_protocol=swd
flash: flash-pyocd
