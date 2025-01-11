MCU_VARIANT = XMC4700
CFLAGS += \
  -DXMC4700_F144x2048 \

# mcu driver cause following warnings
CFLAGS += -Wno-stringop-overread

LD_FILE = $(SDK_DIR)/CMSIS/Infineon/COMPONENT_$(MCU_VARIANT)/Source/TOOLCHAIN_GCC_ARM/XMC4700x2048.ld

JLINK_DEVICE = XMC4700-2048

flash: flash-jlink
