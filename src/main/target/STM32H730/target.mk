TARGET_MCU        := STM32H730xx
TARGET_MCU_FAMILY := STM32H7

HSE_VALUE          = 8000000

ifneq ($(EXST),)
EXST = yes
ifeq ($(EXST_ADJUST_VMA),)
EXST_ADJUST_VMA = 0x90100000
endif
endif

ifneq ($(SPRACING_PIXEL_OSD),)
LD_SCRIPT       = $(LINKER_DIR)/stm32_ram_h730_exst_spracingpixelosd.ld
endif
