UF2_FAMILY_ID = 0x4c71240a
ST_FAMILY = g4
ST_CMSIS = hw/mcu/st/cmsis_device_$(ST_FAMILY)
ST_HAL_DRIVER = hw/mcu/st/stm32$(ST_FAMILY)xx_hal_driver

include $(TOP)/$(BOARD_PATH)/board.mk
CPU_CORE ?= cortex-m4

# --------------
# Compiler Flags
# --------------
CFLAGS += \
  -DCFG_TUSB_MCU=OPT_MCU_STM32G4

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
	src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c \
	src/portable/st/typec/typec_stm32.c \
	$(ST_CMSIS)/Source/Templates/system_stm32$(ST_FAMILY)xx.c \
	$(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal.c \
	$(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_cortex.c \
	$(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_pwr_ex.c \
	$(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_rcc.c \
	$(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_rcc_ex.c \
	$(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_uart.c \
	$(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_uart_ex.c \
	$(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_gpio.c

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

# flash target using on-board stlink
flash: flash-stlink
