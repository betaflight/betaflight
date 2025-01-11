MCU_VARIANT = stm32wb55xx

CFLAGS += \
	-DSTM32WB55xx

# For flash-jlink target
JLINK_DEVICE = STM32WB55RG
