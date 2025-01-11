ST_FAMILY = l0
DEPS_SUBMODULES += \
	lib/CMSIS_5 \
	hw/mcu/st/cmsis_device_$(ST_FAMILY) \
	hw/mcu/st/stm32$(ST_FAMILY)xx_hal_driver

ST_CMSIS = hw/mcu/st/cmsis_device_$(ST_FAMILY)
ST_HAL_DRIVER = hw/mcu/st/stm32$(ST_FAMILY)xx_hal_driver

include $(TOP)/$(BOARD_PATH)/board.mk
CPU_CORE ?= cortex-m0plus

CFLAGS += \
  -flto \
  -DCFG_EXAMPLE_MSC_READONLY \
  -DCFG_EXAMPLE_VIDEO_READONLY \
  -DCFG_TUSB_MCU=OPT_MCU_STM32L0

# mcu driver cause following warnings
CFLAGS_GCC += \
	-Wno-error=unused-parameter \
	-Wno-error=redundant-decls \
	-Wno-error=cast-align \

ifeq ($(TOOLCHAIN),gcc)
CFLAGS_GCC += -Wno-error=maybe-uninitialized
endif

CFLAGS_CLANG += \
  -Wno-error=parentheses-equality

LDFLAGS_GCC += \
  -nostdlib -nostartfiles \
  --specs=nosys.specs --specs=nano.specs

SRC_C += \
  src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c \
  $(ST_CMSIS)/Source/Templates/system_stm32$(ST_FAMILY)xx.c \
  $(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal.c \
  $(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_cortex.c \
  $(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_rcc.c \
  $(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_rcc_ex.c \
  $(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_gpio.c \
  $(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_uart.c

INC += \
	$(TOP)/$(BOARD_PATH) \
  $(TOP)/lib/CMSIS_5/CMSIS/Core/Include \
  $(TOP)/$(ST_CMSIS)/Include \
  $(TOP)/$(ST_HAL_DRIVER)/Inc
