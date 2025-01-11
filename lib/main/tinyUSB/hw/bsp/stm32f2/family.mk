ST_FAMILY = f2

ST_CMSIS = hw/mcu/st/cmsis_device_$(ST_FAMILY)
ST_HAL_DRIVER = hw/mcu/st/stm32$(ST_FAMILY)xx_hal_driver

DEPS_SUBMODULES += \
	lib/CMSIS_5 \
	$(ST_CMSIS) \
	$(ST_HAL_DRIVER)

include $(TOP)/$(BOARD_PATH)/board.mk
CPU_CORE ?= cortex-m3

CFLAGS += \
	-DCFG_TUSB_MCU=OPT_MCU_STM32F2

CFLAGS_GCC += \
  -flto \

# mcu driver cause following warnings
CFLAGS_GCC += -Wno-error=sign-compare

LDFLAGS_GCC += \
  -nostdlib -nostartfiles \
  --specs=nosys.specs --specs=nano.specs

SRC_C += \
  src/portable/synopsys/dwc2/dcd_dwc2.c \
  src/portable/synopsys/dwc2/hcd_dwc2.c \
  src/portable/synopsys/dwc2/dwc2_common.c \
  $(ST_CMSIS)/Source/Templates/system_stm32$(ST_FAMILY)xx.c \
  $(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal.c \
  $(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_cortex.c \
  $(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_rcc.c \
  $(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_rcc_ex.c \
  $(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_gpio.c

INC += \
  $(TOP)/lib/CMSIS_5/CMSIS/Core/Include \
  $(TOP)/$(ST_CMSIS)/Include \
  $(TOP)/$(ST_HAL_DRIVER)/Inc \
  $(TOP)/$(BOARD_PATH)
