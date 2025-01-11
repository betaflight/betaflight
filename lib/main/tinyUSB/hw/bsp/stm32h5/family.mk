ST_FAMILY = h5
ST_CMSIS = hw/mcu/st/cmsis_device_$(ST_FAMILY)
ST_HAL_DRIVER = hw/mcu/st/stm32$(ST_FAMILY)xx_hal_driver

include $(TOP)/$(BOARD_PATH)/board.mk
CPU_CORE ?= cortex-m33

MCU_VARIANT_UPPER = $(subst stm32h,STM32H,$(MCU_VARIANT))

# --------------
# Compiler Flags
# --------------
CFLAGS += \
  -DCFG_TUSB_MCU=OPT_MCU_STM32H5

# GCC Flags
CFLAGS_GCC += \
  -flto \

# suppress warning caused by vendor mcu driver
CFLAGS_GCC += \
  -Wno-error=cast-align \
  -Wno-error=undef \
  -Wno-error=unused-parameter \

CFLAGS_CLANG += \
  -Wno-error=parentheses-equality

LDFLAGS_GCC += \
  -nostdlib -nostartfiles \
  --specs=nosys.specs --specs=nano.specs

# -----------------
# Sources & Include
# -----------------

SRC_C += \
	src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c \
	$(ST_CMSIS)/Source/Templates/system_stm32$(ST_FAMILY)xx.c \
	$(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal.c \
	$(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_cortex.c \
	$(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_pwr.c \
	$(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_pwr_ex.c \
	$(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_rcc.c \
	$(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_rcc_ex.c \
	$(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_uart.c \
	$(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_uart_ex.c \
	$(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_gpio.c \
	$(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_dma.c

INC += \
	$(TOP)/$(BOARD_PATH) \
	$(TOP)/lib/CMSIS_5/CMSIS/Core/Include \
	$(TOP)/$(ST_CMSIS)/Include \
	$(TOP)/$(ST_HAL_DRIVER)/Inc

# Startup
SRC_S_GCC += $(ST_CMSIS)/Source/Templates/gcc/startup_$(MCU_VARIANT).s
SRC_S_IAR += $(ST_CMSIS)/Source/Templates/iar/startup_$(MCU_VARIANT).s

# Linker
LD_FILE_IAR = $(ST_CMSIS)/Source/Templates/iar/linker/$(MCU_VARIANT)_flash.icf
LD_FILE_GCC = $(FAMILY_PATH)/linker/$(MCU_VARIANT_UPPER)_FLASH.ld

# flash target using on-board stlink
flash: flash-stlink
