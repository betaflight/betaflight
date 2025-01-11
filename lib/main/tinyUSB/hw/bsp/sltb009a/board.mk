CFLAGS += \
  -flto \
  -mthumb \
  -mcpu=cortex-m4 \
  -mfloat-abi=hard \
  -mfpu=fpv4-sp-d16 \
  -nostdlib -nostartfiles \
  -D__STARTUP_CLEAR_BSS \
  -D__START=main \
  -DEFM32GG12B810F1024GM64 \
  -DCFG_TUSB_MCU=OPT_MCU_EFM32GG

# mcu driver cause following warnings
#CFLAGS += -Wno-error=unused-parameter

SILABS_FAMILY = efm32gg12b
SILABS_CMSIS = hw/mcu/silabs/cmsis-dfp-$(SILABS_FAMILY)/Device/SiliconLabs/$(shell echo $(SILABS_FAMILY) | tr a-z A-Z)

DEPS_SUBMODULES += hw/mcu/silabs/cmsis-dfp-$(SILABS_FAMILY)
DEPS_SUBMODULES += lib/CMSIS_5

LDFLAGS_GCC += -specs=nosys.specs -specs=nano.specs

# All source paths should be relative to the top level.
LD_FILE = $(SILABS_CMSIS)/Source/GCC/$(SILABS_FAMILY).ld

SRC_C += \
  $(SILABS_CMSIS)/Source/system_$(SILABS_FAMILY).c \
	src/portable/synopsys/dwc2/dcd_dwc2.c \
	src/portable/synopsys/dwc2/hcd_dwc2.c \
	src/portable/synopsys/dwc2/dwc2_common.c \

SRC_S += \
  $(SILABS_CMSIS)/Source/GCC/startup_$(SILABS_FAMILY).S

INC += \
  $(TOP)/lib/CMSIS_5/CMSIS/Core/Include \
  $(TOP)/$(SILABS_CMSIS)/Include \
  $(TOP)/hw/bsp/$(BOARD)

# For freeRTOS port source
FREERTOS_PORTABLE_SRC = $(FREERTOS_PORTABLE_PATH)/ARM_CM4F

# For flash-jlink target
JLINK_DEVICE = EFM32GG12B810F1024

flash: flash-jlink
