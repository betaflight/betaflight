CFLAGS += \
  -DSTM32U575xx \

# All source paths should be relative to the top level.
LD_FILE = ${FAMILY_PATH}/linker/STM32U575xx_FLASH.ld

SRC_S += $(ST_CMSIS)/Source/Templates/gcc/startup_stm32u575xx.s

MCU_VARIANT = stm32u575xx
# For flash-jlink target
JLINK_DEVICE = stm32u575zi
