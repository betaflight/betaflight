CFLAGS += -DSTM32F439xx

# GCC
SRC_S_GCC += $(ST_CMSIS)/Source/Templates/gcc/startup_stm32f439xx.s
LD_FILE_GCC = $(BOARD_PATH)/STM32F439ZITX_FLASH.ld

# IAR
SRC_S_IAR += $(ST_CMSIS)/Source/Templates/iar/startup_stm32f439xx.s
LD_FILE_IAR = $(ST_CMSIS)/Source/Templates/iar/linker/stm32f439xx_flash.icf

# For flash-jlink target
JLINK_DEVICE = stm32f439zi

# flash target using on-board stlink
flash: flash-stlink
