CFLAGS += \
  -DSTM32U545xx \

# All source paths should be relative to the top level.
LD_FILE = ${FAMILY_PATH}/linker/STM32U545xx_FLASH.ld

SRC_S += $(ST_CMSIS)/Source/Templates/gcc/startup_stm32u545xx.s

MCU_VARIANT = stm32u545xx
# For flash-jlink target
JLINK_DEVICE = stm32u545re
