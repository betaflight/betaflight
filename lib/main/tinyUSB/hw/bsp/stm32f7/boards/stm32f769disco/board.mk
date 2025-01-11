MCU_VARIANT = stm32f769xx

# Only OTG-HS has a connector on this board
RHPORT_SPEED = OPT_MODE_FULL_SPEED OPT_MODE_HIGH_SPEED
RHPORT_DEVICE ?= 1
RHPORT_HOST ?= 1

PORT ?= 1
SPEED ?= high

CFLAGS += \
  -DSTM32F769xx \
  -DHSE_VALUE=25000000 \

# Linker
LD_FILE_GCC = $(BOARD_PATH)/STM32F769ZITx_FLASH.ld

JLINK_DEVICE = stm32f769ni

# flash target using on-board stlink
flash: flash-stlink
