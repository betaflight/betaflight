DEPS_SUBMODULES += lib/CMSIS_5 hw/mcu/analog/max32

# Important locations in the hw support for MCU
MAX32_CMSIS = hw/mcu/analog/max32/Libraries/CMSIS
MAX32_PERIPH = hw/mcu/analog/max32/Libraries/PeriphDrivers

# Add any board specific make rules
include $(TOP)/$(BOARD_PATH)/board.mk

CPU_CORE ?= cortex-m4
PORT ?= 0

# GCC
SRC_S_GCC += $(MAX32_CMSIS)/Device/Maxim/MAX32650/Source/GCC/startup_max32650.S

# --------------
# Compiler Flags
# --------------
# Flags for the MAX32650/1/2 SDK
CFLAGS +=   -DTARGET=MAX32650 \
			-DTARGET_REV=0x4131 \
			-DMXC_ASSERT_ENABLE \
			-DMAX32650 \
			-DIAR_PRAGMAS=0

# Flags for TUSB features
CFLAGS += \
  -DCFG_TUSB_MCU=OPT_MCU_MAX32650 \
  -DBOARD_TUD_MAX_SPEED=OPT_MODE_HIGH_SPEED

# mcu driver cause following warnings
CFLAGS += 	-Wno-error=strict-prototypes \
			-Wno-error=unused-parameter \
			-Wno-error=cast-align \
			-Wno-error=cast-qual \
			-Wno-error=sign-compare

LDFLAGS_GCC += -nostartfiles --specs=nosys.specs --specs=nano.specs

# Configure the flash rule. By default, use JLink.
SIGNED_BUILD ?= 0
DEFAULT_FLASH = flash-jlink

# If the applications needs to be signed (for the MAX32651), sign it first and
# then need to use MSDK's OpenOCD to flash it
# Also need to include the __SLA_FWK__ define to enable the signed header into
# memory
ifeq ($(SIGNED_BUILD), 1)
# Extra definitions to build for the secure part
CFLAGS += -D__SLA_FWK__
DEFAULT_FLASH := sign-build flash-msdk
endif

# For flash-jlink target
JLINK_DEVICE = max32650

# Configure the flash rule
flash: $(DEFAULT_FLASH)

# -----------------
# Sources & Include
# -----------------
PERIPH_SRC = $(TOP)/$(MAX32_PERIPH)/Source
SRC_C += \
	src/portable/mentor/musb/dcd_musb.c \
	$(MAX32_CMSIS)/Device/Maxim/MAX32650/Source/heap.c \
	$(MAX32_CMSIS)/Device/Maxim/MAX32650/Source/system_max32650.c \
	$(MAX32_CMSIS)/Device/Maxim/MAX32650/Source/header_MAX32650.c \
	$(PERIPH_SRC)/SYS/mxc_assert.c \
	$(PERIPH_SRC)/SYS/mxc_delay.c \
	$(PERIPH_SRC)/SYS/mxc_lock.c \
	$(PERIPH_SRC)/SYS/nvic_table.c \
	$(PERIPH_SRC)/SYS/pins_me10.c \
	$(PERIPH_SRC)/SYS/sys_me10.c \
	$(PERIPH_SRC)/FLC/flc_common.c \
	$(PERIPH_SRC)/FLC/flc_me10.c \
	$(PERIPH_SRC)/FLC/flc_reva.c \
	$(PERIPH_SRC)/GPIO/gpio_common.c \
	$(PERIPH_SRC)/GPIO/gpio_me10.c \
	$(PERIPH_SRC)/GPIO/gpio_reva.c \
	$(PERIPH_SRC)/ICC/icc_me10.c \
	$(PERIPH_SRC)/ICC/icc_reva.c \
	$(PERIPH_SRC)/ICC/icc_common.c \
	$(PERIPH_SRC)/TPU/tpu_me10.c \
	$(PERIPH_SRC)/TPU/tpu_reva.c \
	$(PERIPH_SRC)/UART/uart_common.c \
	$(PERIPH_SRC)/UART/uart_me10.c \
	$(PERIPH_SRC)/UART/uart_reva.c \

INC += \
	$(TOP)/$(BOARD_PATH) \
	$(TOP)/$(MAX32_CMSIS)/Include \
	$(TOP)/$(MAX32_CMSIS)/Device/Maxim/MAX32650/Include \
	$(TOP)/$(MAX32_PERIPH)/Include/MAX32650 \
	$(PERIPH_SRC)/SYS \
	$(PERIPH_SRC)/GPIO \
	$(PERIPH_SRC)/ICC \
	$(PERIPH_SRC)/FLC \
	$(PERIPH_SRC)/TPU \
	$(PERIPH_SRC)/UART


# The MAX32651EVKIT is pin for pin identical to the MAX32650EVKIT, however the
# MAX32651 has a secure bootloader which requires the image to be signed before
# loading into flash. All MAX32651EVKIT's have the same key for evaluation
# purposes, so create a special flash rule to sign the binary and flash using
# the MSDK.
MCU_PATH = $(TOP)/hw/mcu/analog/max32/
# Assume no extension for sign utility
SIGN_EXE = sign_app
ifeq ($(OS), Windows_NT)
# Must use .exe extension on Windows, since the binaries
# for Linux may live in the same place.
SIGN_EXE := sign_app.exe
else
UNAME = $(shell uname -s)
ifneq ($(findstring MSYS_NT,$(UNAME)),)
# Must also use .exe extension for MSYS2
SIGN_EXE := sign_app.exe
endif
endif

# Rule to sign the build.  This will in-place modify the existing .elf file
# an populate the .sig section with the signature value
sign-build: $(BUILD)/$(PROJECT).elf
	$(OBJCOPY) $(BUILD)/$(PROJECT).elf -R .sig -O binary $(BUILD)/$(PROJECT).bin
	$(MCU_PATH)/Tools/SBT/bin/$(SIGN_EXE) -c MAX32651 \
		key_file="$(MCU_PATH)/Tools/SBT/devices/MAX32651/keys/maximtestcrk.key" \
		ca=$(BUILD)/$(PROJECT).bin sca=$(BUILD)/$(PROJECT).sbin
	$(OBJCOPY) $(BUILD)/$(PROJECT).elf --update-section .sig=$(BUILD)/$(PROJECT).sig

# Optional flash option when running within an installed MSDK to use OpenOCD
# Mainline OpenOCD does not yet have the MAX32's flash algorithm integrated.
# If the MSDK is installed, flash-msdk can be run to utilize the the modified
# openocd with the algorithms
MAXIM_PATH := $(subst \,/,$(MAXIM_PATH))
flash-msdk: $(BUILD)/$(PROJECT).elf
	$(MAXIM_PATH)/Tools/OpenOCD/openocd -s $(MAXIM_PATH)/Tools/OpenOCD/scripts \
		-f interface/cmsis-dap.cfg -f target/max32650.cfg \
		-c "program $(BUILD)/$(PROJECT).elf verify; init; reset; exit"
