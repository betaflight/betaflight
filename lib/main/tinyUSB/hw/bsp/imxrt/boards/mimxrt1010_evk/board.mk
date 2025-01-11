CFLAGS += -DCPU_MIMXRT1011DAE5A -DCFG_EXAMPLE_VIDEO_READONLY
MCU_VARIANT = MIMXRT1011

# For flash-jlink target
JLINK_DEVICE = MIMXRT1011xxx5A

# For flash-pyocd target
PYOCD_TARGET = mimxrt1010

# flash using pyocd
flash: flash-pyocd
