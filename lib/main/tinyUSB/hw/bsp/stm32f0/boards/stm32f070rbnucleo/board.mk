MCU_VARIANT = stm32f070xb

CFLAGS += -DSTM32F070xB -DCFG_EXAMPLE_VIDEO_READONLY

# Linker
LD_FILE_GCC = $(BOARD_PATH)/stm32F070rbtx_flash.ld

# For flash-jlink target
JLINK_DEVICE = stm32f070rb

# flash target using on-board stlink
flash: flash-stlink
