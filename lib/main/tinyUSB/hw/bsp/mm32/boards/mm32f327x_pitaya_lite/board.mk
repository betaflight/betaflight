MCU_VARIANT = mm32f327x

CFLAGS += \
	-DHSE_VALUE=12000000

LD_FILE = $(BOARD_PATH)/flash.ld

# For flash-jlink target
JLINK_DEVICE = MM32F3273G8P

# flash target using on-board stlink
#flash: flash-jlink
