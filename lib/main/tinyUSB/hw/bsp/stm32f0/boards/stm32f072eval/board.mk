MCU_VARIANT = stm32f072xb

CFLAGS += -DSTM32F072xB -DLSI_VALUE=40000 -DCFG_EXAMPLE_VIDEO_READONLY

# Linker
LD_FILE_GCC = $(BOARD_PATH)/STM32F072VBTx_FLASH.ld

# For flash-jlink target
JLINK_DEVICE = stm32f072vb

# flash target using on-board stlink
flash: flash-stlink
