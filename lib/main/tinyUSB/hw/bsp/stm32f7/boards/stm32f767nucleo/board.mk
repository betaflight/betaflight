MCU_VARIANT = stm32f767xx

RHPORT_DEVICE ?= 0
RHPORT_HOST ?= 0

PORT ?= 0
SPEED ?= full

CFLAGS += \
  -DSTM32F767xx \
	-DHSE_VALUE=8000000 \

# Linker
LD_FILE_GCC = $(BOARD_PATH)/STM32F767ZITx_FLASH.ld

# For flash-jlink target
JLINK_DEVICE = stm32f767zi

# flash target using on-board stlink
flash: flash-stlink
