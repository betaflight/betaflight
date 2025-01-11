DEPS_SUBMODULES += hw/mcu/nuvoton

CFLAGS += \
  -flto \
  -mthumb \
  -mabi=aapcs-linux \
  -mcpu=cortex-m0 \
  -D__ARM_FEATURE_DSP=0 \
  -DUSE_ASSERT=0 \
  -DCFG_EXAMPLE_VIDEO_READONLY \
  -D__CORTEX_SC=0 \
  -DCFG_TUSB_MCU=OPT_MCU_NUC126

# mcu driver cause following warnings
CFLAGS += -Wno-error=redundant-decls

LDFLAGS_GCC += -specs=nosys.specs -specs=nano.specs

# All source paths should be relative to the top level.
LD_FILE = hw/bsp/$(BOARD)/nuc126_flash.ld

SRC_C += \
  src/portable/nuvoton/nuc121/dcd_nuc121.c \
  hw/mcu/nuvoton/nuc126/Device/Nuvoton/NUC126/Source/system_NUC126.c \
  hw/mcu/nuvoton/nuc126/StdDriver/src/clk.c \
  hw/mcu/nuvoton/nuc126/StdDriver/src/crc.c \
  hw/mcu/nuvoton/nuc126/StdDriver/src/gpio.c \
  hw/mcu/nuvoton/nuc126/StdDriver/src/rtc.c \
  hw/mcu/nuvoton/nuc126/StdDriver/src/sys.c \
  hw/mcu/nuvoton/nuc126/StdDriver/src/timer.c \
  hw/mcu/nuvoton/nuc126/StdDriver/src/uart.c

SRC_S += \
  hw/mcu/nuvoton/nuc126/Device/Nuvoton/NUC126/Source/GCC/startup_NUC126.S

INC += \
  $(TOP)/hw/mcu/nuvoton/nuc126/Device/Nuvoton/NUC126/Include \
  $(TOP)/hw/mcu/nuvoton/nuc126/StdDriver/inc \
  $(TOP)/hw/mcu/nuvoton/nuc126/CMSIS/Include

# For freeRTOS port source
FREERTOS_PORTABLE_SRC = $(FREERTOS_PORTABLE_PATH)/ARM_CM0

# For flash-jlink target
JLINK_DEVICE = NUC126VG4AE

# Flash using Nuvoton's openocd fork at https://github.com/OpenNuvoton/OpenOCD-Nuvoton
# Please compile and install it from github source
flash: $(BUILD)/$(PROJECT).elf
	openocd -f interface/nulink.cfg -f target/numicroM0.cfg -c "program $< reset exit"
