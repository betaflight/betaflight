MCU_VARIANT = stm32h503xx

CFLAGS += \
	-DSTM32H503xx \
	-DHSE_VALUE=24000000 \

# For flash-jlink target
JLINK_DEVICE = stm32h503rb
