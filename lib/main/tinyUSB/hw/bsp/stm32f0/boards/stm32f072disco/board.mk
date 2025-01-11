MCU_VARIANT = stm32f072xb

CFLAGS += -DSTM32F072xB -DCFG_EXAMPLE_VIDEO_READONLY

# Linker
LD_FILE_GCC = $(BOARD_PATH)/STM32F072RBTx_FLASH.ld

# For flash-jlink target
JLINK_DEVICE = stm32f072rb

# flash target using on-board stlink
flash: flash-stlink
