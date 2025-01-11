CFLAGS += -DSTM32F407xx

# GCC
GCC_SRC_S += $(ST_CMSIS)/Source/Templates/gcc/startup_stm32f407xx.s
GCC_LD_FILE = $(BOARD_PATH)/STM32F407VETX_FLASH.ld

# IAR
IAR_SRC_S += $(ST_CMSIS)/Source/Templates/iar/startup_stm32f407xx.s
IAR_LD_FILE = $(ST_CMSIS)/Source/Templates/iar/linker/stm32f407xx_flash.icf


# For flash-jlink target
JLINK_DEVICE = stm32f407vg

# flash target using on-board stlink
flash: flash-stlink
