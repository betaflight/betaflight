CFLAGS += \
  -DSTM32L476xx \

# GCC
SRC_S_GCC += $(ST_CMSIS)/Source/Templates/gcc/startup_stm32l476xx.s
LD_FILE_GCC = $(BOARD_PATH)/STM32L476VGTx_FLASH.ld

# IAR
SRC_S_IAR += $(ST_CMSIS)/Source/Templates/iar/startup_stm32l476xx.s
LD_FILE_IAR = $(ST_CMSIS)/Source/Templates/iar/linker/stm32l476xx_flash.icf

# For flash-jlink target
JLINK_DEVICE = stm32l476vg
