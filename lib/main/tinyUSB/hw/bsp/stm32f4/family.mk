UF2_FAMILY_ID = 0x57755a57
ST_FAMILY = f4
DEPS_SUBMODULES += lib/CMSIS_5 hw/mcu/st/cmsis_device_$(ST_FAMILY) hw/mcu/st/stm32$(ST_FAMILY)xx_hal_driver

ST_CMSIS = hw/mcu/st/cmsis_device_$(ST_FAMILY)
ST_HAL_DRIVER = hw/mcu/st/stm32$(ST_FAMILY)xx_hal_driver

include $(TOP)/$(BOARD_PATH)/board.mk
CPU_CORE ?= cortex-m4

# ----------------------
# Port & Speed Selection
# ----------------------
RHPORT_SPEED ?= OPT_MODE_FULL_SPEED OPT_MODE_FULL_SPEED
RHPORT_DEVICE ?= 0
RHPORT_HOST ?= 0

# Determine RHPORT_DEVICE_SPEED if not defined
ifndef RHPORT_DEVICE_SPEED
ifeq ($(RHPORT_DEVICE), 0)
  RHPORT_DEVICE_SPEED = $(firstword $(RHPORT_SPEED))
else
  RHPORT_DEVICE_SPEED = $(lastword $(RHPORT_SPEED))
endif
endif

# Determine RHPORT_HOST_SPEED if not defined
ifndef RHPORT_HOST_SPEED
ifeq ($(RHPORT_HOST), 0)
  RHPORT_HOST_SPEED = $(firstword $(RHPORT_SPEED))
else
  RHPORT_HOST_SPEED = $(lastword $(RHPORT_SPEED))
endif
endif

# --------------
# Compiler Flags
# --------------
CFLAGS += \
  -DCFG_TUSB_MCU=OPT_MCU_STM32F4 \
	-DBOARD_TUD_RHPORT=${RHPORT_DEVICE} \
	-DBOARD_TUD_MAX_SPEED=${RHPORT_DEVICE_SPEED} \
	-DBOARD_TUH_RHPORT=${RHPORT_HOST} \
	-DBOARD_TUH_MAX_SPEED=${RHPORT_HOST_SPEED} \

# GCC Flags
CFLAGS_GCC += \
  -flto \

# suppress warning caused by vendor mcu driver
CFLAGS_GCC += -Wno-error=cast-align

LDFLAGS_GCC += \
  -nostdlib -nostartfiles \
  --specs=nosys.specs --specs=nano.specs

# -----------------
# Sources & Include
# -----------------

SRC_C += \
	src/portable/synopsys/dwc2/dcd_dwc2.c \
	src/portable/synopsys/dwc2/hcd_dwc2.c \
	src/portable/synopsys/dwc2/dwc2_common.c \
	$(ST_CMSIS)/Source/Templates/system_stm32$(ST_FAMILY)xx.c \
	$(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal.c \
	$(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_cortex.c \
	$(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_rcc.c \
	$(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_rcc_ex.c \
	$(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_uart.c \
	$(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_dma.c \
	$(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_gpio.c

INC += \
	$(TOP)/$(BOARD_PATH) \
	$(TOP)/lib/CMSIS_5/CMSIS/Core/Include \
	$(TOP)/$(ST_CMSIS)/Include \
	$(TOP)/$(ST_HAL_DRIVER)/Inc

# flash target using on-board stlink
flash: flash-stlink
