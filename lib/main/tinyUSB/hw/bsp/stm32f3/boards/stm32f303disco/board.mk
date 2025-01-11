CFLAGS += \
  -DSTM32F303xC \

# All source paths should be relative to the top level.
LD_FILE = $(BOARD_PATH)/STM32F303VCTx_FLASH.ld

SRC_S += \
  $(ST_CMSIS)/Source/Templates/gcc/startup_stm32f303xc.s

# For flash-jlink target
JLINK_DEVICE = stm32f303vc

# flash target using on-board stlink
flash: flash-stlink
