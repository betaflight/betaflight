MCU_VARIANT = stm32h563xx

CFLAGS += \
	-DSTM32H563xx \
	-DHSE_VALUE=8000000 \

# For flash-jlink target
JLINK_DEVICE = stm32h563zi
