ST_FAMILY = f1
DEPS_SUBMODULES += lib/CMSIS_5 hw/mcu/st/cmsis_device_${ST_FAMILY} hw/mcu/st/stm32${ST_FAMILY}xx_hal_driver

ST_CMSIS = hw/mcu/st/cmsis_device_${ST_FAMILY}
ST_HAL_DRIVER = hw/mcu/st/stm32${ST_FAMILY}xx_hal_driver

include ${TOP}/${BOARD_PATH}/board.mk
CPU_CORE ?= cortex-m3

# --------------
# Compiler Flags
# --------------
CFLAGS += \
  -DCFG_TUSB_MCU=OPT_MCU_STM32F1

# GCC Flags
CFLAGS_GCC += \
  -flto \

# mcu driver cause following warnings
CFLAGS_GCC += -Wno-error=cast-align

LDFLAGS_GCC += \
  -nostdlib -nostartfiles \
  -specs=nosys.specs -specs=nano.specs

# ------------------------
# All source paths should be relative to the top level.
# ------------------------
SRC_C += \
  src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c \
  ${ST_CMSIS}/Source/Templates/system_stm32${ST_FAMILY}xx.c \
  ${ST_HAL_DRIVER}/Src/stm32${ST_FAMILY}xx_hal.c \
  ${ST_HAL_DRIVER}/Src/stm32${ST_FAMILY}xx_hal_cortex.c \
  ${ST_HAL_DRIVER}/Src/stm32${ST_FAMILY}xx_hal_rcc.c \
  ${ST_HAL_DRIVER}/Src/stm32${ST_FAMILY}xx_hal_rcc_ex.c \
  ${ST_HAL_DRIVER}/Src/stm32${ST_FAMILY}xx_hal_gpio.c \
  ${ST_HAL_DRIVER}/Src/stm32${ST_FAMILY}xx_hal_uart.c

INC += \
  ${TOP}/${BOARD_PATH} \
  ${TOP}/lib/CMSIS_5/CMSIS/Core/Include \
  ${TOP}/${ST_CMSIS}/Include \
  ${TOP}/${ST_HAL_DRIVER}/Inc

# Startup
SRC_S_GCC += ${ST_CMSIS}/Source/Templates/gcc/startup_${MCU_VARIANT}.s
SRC_S_IAR += ${ST_CMSIS}/Source/Templates/iar/startup_${MCU_VARIANT}.s

# flash target ROM bootloader: flash-dfu-util
DFU_UTIL_OPTION = -a 0 --dfuse-address 0x08000000
