CFLAGS += -DSTM32F412Zx

# GCC
SRC_S_GCC += $(ST_CMSIS)/Source/Templates/gcc/startup_stm32f412zx.s
LD_FILE_GCC = $(BOARD_PATH)/STM32F412ZGTx_FLASH.ld

# IAR
SRC_S_IAR += $(ST_CMSIS)/Source/Templates/iar/startup_stm32f412zx.s
LD_FILE_IAR = $(ST_CMSIS)/Source/Templates/iar/linker/stm32f412zx_flash.icf

# For flash-jlink target
JLINK_DEVICE = stm32f412zg

# flash target using on-board stlink
flash: flash-stlink
