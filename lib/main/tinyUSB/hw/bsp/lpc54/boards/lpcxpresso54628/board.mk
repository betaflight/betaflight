MCU_VARIANT = LPC54628
MCU_CORE = LPC54628

PORT ?= 1

CFLAGS += -DCPU_LPC54628J512ET180
CFLAGS += -Wno-error=double-promotion

LD_FILE = $(MCU_DIR)/gcc/LPC54628J512_flash.ld

LIBS += $(TOP)/$(MCU_DIR)/gcc/libpower_hardabi.a

JLINK_DEVICE = LPC54628J512
PYOCD_TARGET = LPC54628

#flash: flash-pyocd

flash: flash-jlink
