UF2_FAMILY_ID = 0x0
include $(TOP)/$(BOARD_PATH)/board.mk

MCU_VARIANT_UPPER = $(subst mm32f,MM32F,${MCU_VARIANT})
SDK_DIR = hw/mcu/mindmotion/mm32sdk/${MCU_VARIANT_UPPER}

CPU_CORE ?= cortex-m3

CFLAGS += \
  -flto \
  -DCFG_TUSB_MCU=OPT_MCU_MM32F327X \

# suppress warning caused by vendor mcu driver
CFLAGS += -Wno-error=unused-parameter -Wno-error=maybe-uninitialized -Wno-error=cast-qual

LDFLAGS_GCC += \
  -nostdlib -nostartfiles \
  -specs=nosys.specs -specs=nano.specs \

SRC_C += \
	src/portable/mindmotion/mm32/dcd_mm32f327x_otg.c \
	$(SDK_DIR)/Source/system_${MCU_VARIANT}.c \
	$(SDK_DIR)/HAL_Lib/Src/hal_gpio.c \
	$(SDK_DIR)/HAL_Lib/Src/hal_rcc.c \
	$(SDK_DIR)/HAL_Lib/Src/hal_uart.c \

SRC_S += ${SDK_DIR}/Source/GCC_StartAsm/startup_${MCU_VARIANT}_gcc.s

INC += \
	$(TOP)/$(BOARD_PATH) \
	$(TOP)/lib/CMSIS_5/CMSIS/Core/Include \
	$(TOP)/$(SDK_DIR)/Include \
	$(TOP)/$(SDK_DIR)/HAL_Lib/Inc
