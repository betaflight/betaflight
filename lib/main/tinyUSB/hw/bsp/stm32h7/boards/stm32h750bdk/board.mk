# STM32H745I-DISCO uses OTG_FS
# FIXME: Reset enumerates, un/replug USB plug does not enumerate
MCU_VARIANT = stm32h750xx
CFLAGS += -DSTM32H750xx -DCORE_CM7 -DHSE_VALUE=25000000

LD_FILE_GCC = $(BOARD_PATH)/stm32h750xx_flash_CM7.ld

# For flash-jlink target
JLINK_DEVICE = stm32h750xb

# flash target using on-board stlink
flash: flash-stlink
