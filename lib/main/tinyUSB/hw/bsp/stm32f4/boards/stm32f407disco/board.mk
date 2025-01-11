CFLAGS += -DSTM32F407xx

# GCC
SRC_S_GCC += $(ST_CMSIS)/Source/Templates/gcc/startup_stm32f407xx.s
LD_FILE_GCC = $(BOARD_PATH)/STM32F407VGTx_FLASH.ld

# IAR
SRC_S_IAR += $(ST_CMSIS)/Source/Templates/iar/startup_stm32f407xx.s
LD_FILE_IAR = $(ST_CMSIS)/Source/Templates/iar/linker/stm32f407xx_flash.icf


# For flash-jlink target
JLINK_DEVICE = stm32f407vg

# flash target using on-board stlink
flash: flash-stlink
