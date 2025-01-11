CFLAGS += \
  -DSTM32F207xx \

# All source paths should be relative to the top level.
LD_FILE = $(BOARD_PATH)/STM32F207ZGTx_FLASH.ld

SRC_S += \
  $(ST_CMSIS)/Source/Templates/gcc/startup_stm32f207xx.s

# For flash-jlink target
JLINK_DEVICE = stm32f207zg

# flash target using on-board stlink
flash: flash-stlink
