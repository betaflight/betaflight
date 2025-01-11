CFLAGS += -DCPU_MIMXRT1062DVL6A
MCU_VARIANT = MIMXRT1062

# For flash-jlink target
JLINK_DEVICE = MIMXRT1062xxx6A

# For flash-pyocd target
PYOCD_TARGET = mimxrt1060

BOARD_TUD_RHPORT = 0
BOARD_TUH_RHPORT = 1

# flash using pyocd
flash: flash-pyocd
