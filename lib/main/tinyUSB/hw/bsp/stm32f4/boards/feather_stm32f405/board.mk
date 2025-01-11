CFLAGS += -DSTM32F405xx

# GCC
SRC_S_GCC += $(ST_CMSIS)/Source/Templates/gcc/startup_stm32f405xx.s
LD_FILE_GCC = $(BOARD_PATH)/STM32F405RGTx_FLASH.ld

# IAR
SRC_S_IAR += $(ST_CMSIS)/Source/Templates/iar/startup_stm32f405xx.s
LD_FILE_IAR = $(ST_CMSIS)/Source/Templates/iar/linker/stm32f405xx_flash.icf

# For flash-jlink target
JLINK_DEVICE = stm32f405rg

# flash target ROM bootloader
flash: $(BUILD)/$(PROJECT).bin
	dfu-util -R -a 0 --dfuse-address 0x08000000 -D $<
