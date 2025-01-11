MCU_VARIANT = stm32g474xx

CFLAGS += \
	-DSTM32G474xx \

# Linker
LD_FILE_GCC = $(BOARD_PATH)/STM32G474RETx_FLASH.ld

# For flash-jlink target
JLINK_DEVICE = stm32g474re
