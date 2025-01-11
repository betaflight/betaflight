CFLAGS += -DCPU_MIMXRT1015DAF5A -DCFG_EXAMPLE_VIDEO_READONLY
MCU_VARIANT = MIMXRT1015

# For flash-jlink target
JLINK_DEVICE = MIMXRT1015DAF5A

# For flash-pyocd target
PYOCD_TARGET = mimxrt1015

# flash using pyocd
flash: flash-pyocd
