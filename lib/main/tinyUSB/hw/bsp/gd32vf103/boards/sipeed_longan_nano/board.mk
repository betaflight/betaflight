LONGAN_NANO_SDK_BSP = $(GD32VF103_SDK_SOC)/Board/gd32vf103c_longan_nano
LINKER_SCRIPTS = $(LONGAN_NANO_SDK_BSP)/Source/GCC

# All source paths should be relative to the top level.
LD_FILE = $(LINKER_SCRIPTS)/gcc_gd32vf103xb_flashxip.ld # Longan Nano 128k ROM 32k RAM
#LD_FILE = $(LINKER_SCRIPTS)/gcc_gd32vf103x8_flashxip.ld # Longan Nano Lite 64k ROM 20k RAM

SRC_C += $(LONGAN_NANO_SDK_BSP)/Source/gd32vf103c_longan_nano.c
INC += $(TOP)/$(LONGAN_NANO_SDK_BSP)/Include

# Longan Nano 128k ROM 32k RAM
JLINK_DEVICE = gd32vf103cbt6
