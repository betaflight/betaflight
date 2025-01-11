include $(TOP)/$(BOARD_PATH)/board.mk
CPU_CORE ?= cortex-m4

MCU_VARIANT = tm4c${MCU_SUB_VARIANT}
MCU_VARIANT_UPPER = TM4C${MCU_SUB_VARIANT}

SDK_DIR = hw/mcu/ti/${MCU_VARIANT}xx

CFLAGS += \
  -flto \
  -DCFG_TUSB_MCU=OPT_MCU_TM4C123 \
  -uvectors \

# mcu driver cause following warnings
CFLAGS += -Wno-error=strict-prototypes -Wno-error=cast-qual

LDFLAGS_GCC += --specs=nosys.specs --specs=nano.specs

INC += \
	$(TOP)/lib/CMSIS_5/CMSIS/Core/Include \
	$(TOP)/$(SDK_DIR)/Include/${MCU_VARIANT_UPPER} \
	$(TOP)/$(BOARD_PATH)

SRC_C += \
	src/portable/mentor/musb/dcd_musb.c \
	src/portable/mentor/musb/hcd_musb.c \
	$(SDK_DIR)/Source/system_${MCU_VARIANT_UPPER}.c \
	$(SDK_DIR)/Source/GCC/${MCU_VARIANT}_startup.c
