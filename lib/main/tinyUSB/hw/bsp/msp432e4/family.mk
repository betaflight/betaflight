SDK_SIR = hw/mcu/ti/msp432e4

include $(TOP)/$(BOARD_PATH)/board.mk
CPU_CORE ?= cortex-m4

CFLAGS += \
	-flto \
	-mslow-flash-data \
	-DCFG_TUSB_MCU=OPT_MCU_MSP432E4

# mcu driver cause following warnings
CFLAGS += -Wno-error=cast-qual -Wno-error=format=

LDFLAGS_GCC += --specs=nosys.specs --specs=nano.specs

LD_FILE = hw/mcu/ti/msp432e4/Source/${MCU_VARIANT}.ld

SRC_C += \
	src/portable/mentor/musb/dcd_musb.c \
	src/portable/mentor/musb/hcd_musb.c \
	$(SDK_SIR)/Source/system_${MCU_VARIANT}.c

INC += \
	$(TOP)/lib/CMSIS_5/CMSIS/Core/Include \
	$(TOP)/$(SDK_SIR)/Include \
	$(TOP)/$(BOARD_PATH)

SRC_S += $(SDK_SIR)/Source/startup_${MCU_VARIANT}_gcc.S

# For flash-jlink target
JLINK_DEVICE = $(MCU_VARIANT)
