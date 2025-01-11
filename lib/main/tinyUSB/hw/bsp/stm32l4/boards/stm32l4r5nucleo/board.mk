CFLAGS += \
  -DHSE_VALUE=8000000 \
  -DSTM32L4R5xx \

# GCC
SRC_S_GCC += $(ST_CMSIS)/Source/Templates/gcc/startup_stm32l4r5xx.s
LD_FILE_GCC = $(BOARD_PATH)/STM32L4RXxI_FLASH.ld

# IAR
SRC_S_IAR += $(ST_CMSIS)/Source/Templates/iar/startup_stm32l4r5xx.s
LD_FILE_IAR = $(ST_CMSIS)/Source/Templates/iar/linker/stm32l4r5xx_flash.icf

# For flash-jlink target
JLINK_DEVICE = stm32l4r5zi

# flash target using on-board stlink
flash: flash-stlink
