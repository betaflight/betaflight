MCU_VARIANT = stm32h743xx
CFLAGS += -DSTM32H743xx -DHSE_VALUE=8000000

RHPORT_SPEED = OPT_MODE_FULL_SPEED OPT_MODE_HIGH_SPEED
RHPORT_DEVICE ?= 1
RHPORT_HOST ?= 0

LD_FILE_GCC = $(FAMILY_PATH)/linker/stm32h743xx_flash.ld

# Use Timer module for ULPI PHY reset
CFLAGS += -DHAL_TIM_MODULE_ENABLED
SRC_C += \
  $(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_tim.c \
  $(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_tim_ex.c

# For flash-jlink target
JLINK_DEVICE = stm32h743ii

# flash target using jlink
flash: flash-jlink
