CFLAGS += -DCPU_MIMXRT1024DAG5A
MCU_VARIANT = MIMXRT1024

# warnings caused by mcu driver
CFLAGS += -Wno-error=array-bounds

# For flash-jlink target
JLINK_DEVICE = MIMXRT1024xxx5A

# For flash-pyocd target
PYOCD_TARGET = mimxrt1024

# flash using pyocd
flash: flash-pyocd
