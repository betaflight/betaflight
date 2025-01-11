CROSS_COMPILE = xc32-
CFLAGS_OPTIMIZED = -O2
LIBS_GCC = -lgcc -lm
SKIP_NANOLIB = 1

CFLAGS = \
  -std=c99 \
  -DCFG_TUSB_MCU=OPT_MCU_PIC32MZ

include $(TOP)/$(BOARD_PATH)/board.mk

SRC_C += \
	src/portable/microchip/pic32mz/dcd_pic32mz.c \

INC += \
	$(TOP)/hw/mcu/microchip/pic32mz \
	$(TOP)/$(BOARD_PATH) \

# flash target using jlink
flash: flash-jlink
