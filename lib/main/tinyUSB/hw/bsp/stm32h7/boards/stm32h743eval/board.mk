MCU_VARIANT = stm32h743xx
CFLAGS += -DSTM32H743xx -DHSE_VALUE=25000000

RHPORT_SPEED = OPT_MODE_FULL_SPEED OPT_MODE_HIGH_SPEED
RHPORT_DEVICE ?= 1
RHPORT_HOST ?= 0

LD_FILE_GCC = $(FAMILY_PATH)/linker/${MCU_VARIANT}_flash.ld

SRC_C += \
  ${ST_MFXSTM32L152}/mfxstm32l152.c \
  ${ST_MFXSTM32L152}/mfxstm32l152_reg.c \

INC += $(TOP)/${ST_MFXSTM32L152}

# For flash-jlink target
JLINK_DEVICE = stm32h743xi

# flash target using on-board stlink
flash: flash-stlink
