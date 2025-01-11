MCU_VARIANT = LPC54608
MCU_CORE = LPC54608

PORT ?= 1

CFLAGS += -DCPU_LPC54608J512ET180
CFLAGS += -Wno-error=double-promotion

LD_FILE = $(MCU_DIR)/gcc/LPC54608J512_flash.ld

LIBS += $(TOP)/$(MCU_DIR)/gcc/libpower_hardabi.a

JLINK_DEVICE = LPC54608J512
PYOCD_TARGET = LPC54608

#flash: flash-pyocd

flash: flash-jlink
