CFLAGS += -DCPU_MIMXRT1021DAG5A
MCU_VARIANT = MIMXRT1021

# For flash-jlink target
JLINK_DEVICE = MIMXRT1021xxx5A

# For flash-pyocd target
PYOCD_TARGET = mimxrt1020

# flash using pyocd
flash: flash-pyocd
