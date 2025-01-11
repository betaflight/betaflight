CFLAGS += \
	-DSTM32G0B1xx

# GCC
SRC_S_GCC += $(ST_CMSIS)/Source/Templates/gcc/startup_stm32g0b1xx.s
LD_FILE_GCC = $(BOARD_PATH)/STM32G0B1RETx_FLASH.ld

# IAR
SRC_S_IAR += $(ST_CMSIS)/Source/Templates/iar/startup_stm32g0b1xx.s
LD_FILE_IAR = $(ST_CMSIS)/Source/Templates/iar/linker/stm32g0b1xx_flash.icf

# For flash-jlink target
JLINK_DEVICE = stm32g0b1re
