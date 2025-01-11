MCU_VARIANT = MK64F12

CFLAGS += \
  -DCPU_MK64FN1M0VMD12 \

# mcu driver cause following warnings
CFLAGS += -Wno-error=unused-parameter -Wno-error=format -Wno-error=redundant-decls

SRC_C += \
	$(BOARD_PATH)/board/clock_config.c \
	$(BOARD_PATH)/board/pin_mux.c \

LD_FILE = ${SDK_DIR}/devices/${MCU_VARIANT}/gcc/MK64FN1M0xxx12_flash.ld

# For flash-jlink target
JLINK_DEVICE = MK64FN1M0xxx12

# For flash-pyocd target
PYOCD_TARGET = k64f
