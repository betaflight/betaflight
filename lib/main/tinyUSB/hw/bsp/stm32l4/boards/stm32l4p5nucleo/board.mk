CFLAGS += \
  -DSTM32L4P5xx \

# GCC
SRC_S_GCC += $(ST_CMSIS)/Source/Templates/gcc/startup_stm32l4p5xx.s
LD_FILE_GCC = $(BOARD_PATH)/STM32L4P5ZGTX_FLASH.ld

# IAR
SRC_S_IAR += $(ST_CMSIS)/Source/Templates/iar/startup_stm32l4p5xx.s
LD_FILE_IAR = $(ST_CMSIS)/Source/Templates/iar/linker/stm32l4p5xx_flash.icf

# For flash-jlink target
JLINK_DEVICE = stm32l4p5zg
