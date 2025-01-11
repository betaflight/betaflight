CFLAGS += \
  -DSTM32U5A5xx \
  -DHSE_VALUE=16000000UL \

# All source paths should be relative to the top level.
LD_FILE = ${BOARD_PATH}/STM32U5A5ZJTXQ_FLASH.ld

SRC_S += $(ST_CMSIS)/Source/Templates/gcc/startup_stm32u5a5xx.s

MCU_VARIANT = stm32u5a5xx
# For flash-jlink target
JLINK_DEVICE = stm32u575zi
