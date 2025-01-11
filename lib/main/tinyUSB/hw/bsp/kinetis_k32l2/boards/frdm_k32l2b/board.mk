MCU = K32L2B31A

CFLAGS += -DCPU_K32L2B31VLH0A

# mcu driver cause following warnings
CFLAGS += -Wno-error=unused-parameter -Wno-error=redundant-decls

# All source paths should be relative to the top level.
LD_FILE = $(MCU_DIR)/gcc/K32L2B31xxxxA_flash.ld

# For flash-jlink target
JLINK_DEVICE = K32L2B31xxxxA

# For flash-pyocd target
PYOCD_TARGET = K32L2B

# flash using pyocd
flash: flash-pyocd
