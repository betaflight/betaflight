CFLAGS += \
  -DSTM32L053xx

# All source paths should be relative to the top level.
LD_FILE = $(BOARD_PATH)/STM32L053C8Tx_FLASH.ld

SRC_S += \
  $(ST_CMSIS)/Source/Templates/gcc/startup_stm32l053xx.s

# For flash-jlink target
JLINK_DEVICE = STM32L053R8

# flash target using on-board stlink
flash: flash-stlink
