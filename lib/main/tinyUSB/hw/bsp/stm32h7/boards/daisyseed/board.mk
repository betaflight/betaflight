MCU_VARIANT = stm32h750xx
CFLAGS += -DSTM32H750xx -DCORE_CM7 -DHSE_VALUE=16000000

LD_FILE_GCC = $(BOARD_PATH)/stm32h750ibkx_flash.ld

# For flash-jlink target
JLINK_DEVICE = stm32h750ibk6_m7

# flash target using on-board stlink
flash: flash-stlink
