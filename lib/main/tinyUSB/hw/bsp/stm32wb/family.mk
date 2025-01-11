UF2_FAMILY_ID = 0x70d16653
ST_FAMILY = wb

ST_PREFIX = stm32${ST_FAMILY}xx
ST_CMSIS = hw/mcu/st/cmsis_device_$(ST_FAMILY)
ST_HAL_DRIVER = hw/mcu/st/stm32$(ST_FAMILY)xx_hal_driver

include $(TOP)/$(BOARD_PATH)/board.mk
CPU_CORE ?= cortex-m4

CFLAGS += \
  -flto \
  -nostdlib -nostartfiles \
  -DCFG_TUSB_MCU=OPT_MCU_STM32WB

# suppress warning caused by vendor mcu driver
CFLAGS += -Wno-error=cast-align -Wno-unused-parameter

LD_FILE ?= ${ST_CMSIS}/Source/Templates/gcc/linker/${MCU_VARIANT}_flash_cm4.ld

LDFLAGS_GCC += -specs=nosys.specs -specs=nano.specs

SRC_C += \
	src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c \
	$(ST_CMSIS)/Source/Templates/system_${ST_PREFIX}.c \
	$(ST_HAL_DRIVER)/Src/${ST_PREFIX}_hal.c \
	$(ST_HAL_DRIVER)/Src/${ST_PREFIX}_hal_cortex.c \
	$(ST_HAL_DRIVER)/Src/${ST_PREFIX}_hal_pwr_ex.c \
	$(ST_HAL_DRIVER)/Src/${ST_PREFIX}_hal_rcc.c \
	$(ST_HAL_DRIVER)/Src/${ST_PREFIX}_hal_rcc_ex.c \
	$(ST_HAL_DRIVER)/Src/${ST_PREFIX}_hal_uart.c \
	$(ST_HAL_DRIVER)/Src/${ST_PREFIX}_hal_gpio.c

SRC_S += $(ST_CMSIS)/Source/Templates/gcc/startup_${MCU_VARIANT}_cm4.s

INC += \
	$(TOP)/$(BOARD_PATH) \
	$(TOP)/lib/CMSIS_5/CMSIS/Core/Include \
	$(TOP)/$(ST_CMSIS)/Include \
	$(TOP)/$(ST_HAL_DRIVER)/Inc

# flash target using on-board stlink
flash: flash-stlink
