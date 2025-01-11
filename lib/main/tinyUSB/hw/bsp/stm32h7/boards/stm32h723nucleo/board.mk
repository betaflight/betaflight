MCU_VARIANT = stm32h723xx
CFLAGS += -DSTM32H723xx -DHSE_VALUE=8000000

LD_FILE_GCC = $(FAMILY_PATH)/linker/${MCU_VARIANT}_flash.ld

# For flash-jlink target
JLINK_DEVICE = stm32h723zg

# flash target using on-board stlink
flash: flash-stlink
