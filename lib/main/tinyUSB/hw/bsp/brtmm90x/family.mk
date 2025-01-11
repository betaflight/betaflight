# GCC prefix for FT90X compile tools.
CROSS_COMPILE = ft32-elf-
SKIP_NANOLIB = 1

# Set to use FT90X prebuilt libraries.
FT9XX_PREBUILT_LIBS = 0
ifeq ($(FT9XX_PREBUILT_LIBS),1)
# If the FT90X toolchain is installed on Windows systems then the SDK
# include files and prebuilt libraries are at: %FT90X_TOOLCHAIN%/hardware
FT9XX_SDK = $(FT90X_TOOLCHAIN)/hardware
INC += "$(FT9XX_SDK)/include"
else
# The submodule BRTSG-FOSS/ft90x-sdk contains header files and source
# code for the Bridgetek SDK. This can be used instead of the prebuilt
# library.
DEPS_SUBMODULES += hw/mcu/bridgetek/ft9xx/ft90x-sdk
# The SDK can be used to load specific files from the Bridgetek SDK.
FT9XX_SDK = hw/mcu/bridgetek/ft9xx/ft90x-sdk/Source
INC += "$(TOP)/$(FT9XX_SDK)/include"
endif

# Add include files which are within the TinyUSB directory structure.
INC += \
	$(TOP)/$(BOARD_PATH)

# Add required C Compiler flags for FT90X.
CFLAGS += \
	-D__FT900__ \
	-fvar-tracking \
	-fvar-tracking-assignments \
	-fmessage-length=0 \
	-ffunction-sections \
	-DCFG_TUSB_MCU=OPT_MCU_FT90X

# Maximum USB device speed supported by the board
CFLAGS += -DBOARD_TUD_MAX_SPEED=OPT_MODE_HIGH_SPEED

# lwip/src/core/raw.c:334:43: error: declaration of 'recv' shadows a global declaration
CFLAGS += -Wno-error=shadow

# Set Linker flags.
LD_FILE = hw/mcu/bridgetek/ft9xx/scripts/ldscript.ld
LDFLAGS += $(addprefix -L,$(LDINC)) \
	-Xlinker --entry=_start \
	-Wl,-lc

# Additional Source files for FT90X.
SRC_C += src/portable/bridgetek/ft9xx/dcd_ft9xx.c

# Linker library.
ifneq ($(FT9XX_PREBUILT_LIBS),1)
# Optionally add in files from the Bridgetek SDK instead of the prebuilt
# library. These are the minimum required.
SRC_C += $(FT9XX_SDK)/src/sys.c
SRC_C += $(FT9XX_SDK)/src/interrupt.c
SRC_C += $(FT9XX_SDK)/src/delay.c
SRC_C += $(FT9XX_SDK)/src/timers.c
SRC_C += $(FT9XX_SDK)/src/uart_simple.c
SRC_C += $(FT9XX_SDK)/src/gpio.c
else
# Or if using the prebuilt libraries add them.
LDFLAGS += -L"$(FT9XX_SDK)/lib"
LIBS += -lft900
endif

# Not required crt0 file for FT90X. Use compiler built-in file.
#SRC_S += hw/mcu/bridgetek/ft9xx/scripts/crt0.S
