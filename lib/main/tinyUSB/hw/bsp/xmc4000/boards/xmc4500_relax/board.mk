MCU_VARIANT = XMC4500
CFLAGS += \
  -DXMC4500_F100x1024 \

# mcu driver cause following warnings
CFLAGS += -Wno-stringop-overread

LD_FILE = $(SDK_DIR)/CMSIS/Infineon/COMPONENT_$(MCU_VARIANT)/Source/TOOLCHAIN_GCC_ARM/XMC4500x1024.ld

JLINK_DEVICE = XMC4500-1024

flash: flash-jlink
