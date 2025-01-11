MCU_VARIANT = MIMXRT1176

ifeq ($(M4), 1)
  MCU_CORE = _cm4
  JLINK_CORE = _M4
  CPU_CORE = cortex-m4
  LD_FILE ?= $(MCU_DIR)/gcc/$(MCU_VARIANT)xxxxx${MCU_CORE}_ram.ld
else
  MCU_CORE = _cm7
  JLINK_CORE = _M7
endif

CFLAGS += -DCPU_MIMXRT1176DVMAA$(MCU_CORE)

# For flash-jlink target
JLINK_DEVICE = MIMXRT1176xxxA$(JLINK_CORE)

# For flash-pyocd target
PYOCD_TARGET = mimxrt1170$(MCU_CORE)

BOARD_TUD_RHPORT = 0
BOARD_TUH_RHPORT = 1

# flash using pyocd
flash: flash-pyocd
