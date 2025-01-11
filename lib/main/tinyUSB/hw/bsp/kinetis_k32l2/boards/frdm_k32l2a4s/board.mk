MCU = K32L2A41A

CFLAGS += -DCPU_K32L2A41VLH1A

# mcu driver cause following warnings
CFLAGS += -Wno-error=unused-parameter -Wno-error=redundant-decls -Wno-error=cast-qual

# All source paths should be relative to the top level.
LD_FILE = $(MCU_DIR)/gcc/K32L2A41xxxxA_flash.ld

# For flash-jlink target
JLINK_DEVICE = K32L2A41xxxxA

# For flash-pyocd target
PYOCD_TARGET = K32L2A

# flash using pyocd
flash: flash-pyocd
