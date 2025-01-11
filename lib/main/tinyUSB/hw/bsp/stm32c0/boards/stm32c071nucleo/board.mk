CFLAGS += \
	-DSTM32C071xx

# GCC
SRC_S_GCC += $(ST_CMSIS)/Source/Templates/gcc/startup_stm32c071xx.s
LD_FILE_GCC = $(BOARD_PATH)/STM32C071RBTx_FLASH.ld

# IAR
SRC_S_IAR += $(ST_CMSIS)/Source/Templates/iar/startup_stm32c071xx.s
LD_FILE_IAR = $(ST_CMSIS)/Source/Templates/iar/linker/stm32c071xx_flash.icf

# For flash-jlink target
JLINK_DEVICE = stm32c071rb
