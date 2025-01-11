MCU = MKL25Z4

CFLAGS += \
  -DCPU_MKL25Z128VLK4 \
  -DCFG_EXAMPLE_VIDEO_READONLY \
  -DCFG_EXAMPLE_MSC_READONLY

# mcu driver cause following warnings
CFLAGS += -Wno-error=unused-parameter -Wno-error=format -Wno-error=redundant-decls

SRC_C += \
	$(BOARD_PATH)/clock_config.c \

# All source paths should be relative to the top level.
LD_FILE = $(FAMILY_PATH)/gcc/MKL25Z128xxx4_flash.ld

SRC_S += $(FAMILY_PATH)/gcc/startup_$(MCU).S

# For flash-jlink target
JLINK_DEVICE = MKL25Z128xxx4

# For flash-pyocd target
PYOCD_TARGET = mkl25zl128

# flash using pyocd
flash: flash-jlink
