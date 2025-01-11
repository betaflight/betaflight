DEPS_SUBMODULES += hw/mcu/nuvoton

CFLAGS += \
  -flto \
  -mthumb \
  -mabi=aapcs-linux \
  -mcpu=cortex-m0 \
  -DCFG_EXAMPLE_MSC_READONLY \
  -DCFG_EXAMPLE_VIDEO_READONLY \
  -DCFG_TUSB_MCU=OPT_MCU_NUC120

LDFLAGS_GCC += -specs=nosys.specs -specs=nano.specs

# All source paths should be relative to the top level.
LD_FILE = hw/bsp/nutiny_sdk_nuc120/nuc120_flash.ld

SRC_C += \
  src/portable/nuvoton/nuc120/dcd_nuc120.c \
  hw/mcu/nuvoton/nuc100_120/Device/Nuvoton/NUC100Series/Source/system_NUC100Series.c \
  hw/mcu/nuvoton/nuc100_120/StdDriver/src/clk.c \
  hw/mcu/nuvoton/nuc100_120/StdDriver/src/gpio.c \
  hw/mcu/nuvoton/nuc100_120/StdDriver/src/sys.c \
  hw/mcu/nuvoton/nuc100_120/StdDriver/src/timer.c \
  hw/mcu/nuvoton/nuc100_120/StdDriver/src/uart.c

SRC_S += \
  hw/mcu/nuvoton/nuc100_120/Device/Nuvoton/NUC100Series/Source/GCC/startup_NUC100Series.S

INC += \
  $(TOP)/hw/mcu/nuvoton/nuc100_120/Device/Nuvoton/NUC100Series/Include \
  $(TOP)/hw/mcu/nuvoton/nuc100_120/StdDriver/inc \
  $(TOP)/hw/mcu/nuvoton/nuc100_120/CMSIS/Include

# For freeRTOS port source
FREERTOS_PORTABLE_SRC = $(FREERTOS_PORTABLE_PATH)/ARM_CM0

# For flash-jlink target
JLINK_DEVICE = NUC120LE3

# Flash using Nuvoton's openocd fork at https://github.com/OpenNuvoton/OpenOCD-Nuvoton
# Please compile and install it from github source
flash: $(BUILD)/$(PROJECT).elf
	openocd -f interface/nulink.cfg -f target/numicroM0.cfg -c "program $< reset exit"
