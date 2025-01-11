CFLAGS += -DSTM32F411xE

# GCC
SRC_S_GCC += $(ST_CMSIS)/Source/Templates/gcc/startup_stm32f411xe.s
LD_FILE_GCC = $(BOARD_PATH)/STM32F411VETx_FLASH.ld

# IAR
SRC_S_IAR += $(ST_CMSIS)/Source/Templates/iar/startup_stm32f411xe.s
LD_FILE_IAR = $(ST_CMSIS)/Source/Templates/iar/linker/stm32f411xe_flash.icf

# For flash-jlink target
JLINK_DEVICE = stm32f411ve

# flash target using on-board stlink
flash: flash-stlink
