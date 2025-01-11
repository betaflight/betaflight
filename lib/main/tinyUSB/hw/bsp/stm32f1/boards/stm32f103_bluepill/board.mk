MCU_VARIANT = stm32f103xb

CFLAGS += -DSTM32F103xB -DHSE_VALUE=8000000U -DCFG_EXAMPLE_VIDEO_READONLY

# Linker
LD_FILE_GCC = $(BOARD_PATH)/STM32F103X8_FLASH.ld
LD_FILE_IAR = $(BOARD_PATH)/stm32f103x8_flash.icf

# For flash-jlink target
JLINK_DEVICE = stm32f103c8

# flash target ROM bootloader
flash: flash-dfu-util
