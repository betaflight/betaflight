DEPS_SUBMODULES += lib/CMSIS_5 hw/mcu/analog/max32

# Important locations in the hw support for MCU
MAX32_CMSIS = hw/mcu/analog/max32/Libraries/CMSIS
MAX32_PERIPH = hw/mcu/analog/max32/Libraries/PeriphDrivers

# Add any board specific make rules
include $(TOP)/$(BOARD_PATH)/board.mk

CPU_CORE ?= cortex-m4
PORT ?= 0

# GCC
SRC_S_GCC += $(MAX32_CMSIS)/Device/Maxim/MAX32665/Source/GCC/startup_max32665.S
LD_FILE = $(FAMILY_PATH)/max32666.ld

# --------------
# Compiler Flags
# --------------
# Flags for the MAX32665/6 SDK
CFLAGS +=   -DTARGET=MAX32665 \
			-DTARGET_REV=0x4131 \
			-DMXC_ASSERT_ENABLE \
			-DMAX32665 \
			-DIAR_PRAGMAS=0

# Flags for TUSB features
CFLAGS += \
  -DCFG_TUSB_MCU=OPT_MCU_MAX32666 \
  -DBOARD_TUD_MAX_SPEED=OPT_MODE_HIGH_SPEED

# mcu driver cause following warnings
CFLAGS += 	-Wno-error=strict-prototypes \
			-Wno-error=unused-parameter \
			-Wno-error=cast-align \
			-Wno-error=cast-qual
LDFLAGS_GCC += -nostartfiles --specs=nosys.specs --specs=nano.specs

# For flash-jlink target
JLINK_DEVICE = max32666

# flash target using Jlink by default
flash: flash-jlink

# Optional flash option when running within an installed MSDK to use OpenOCD
# Mainline OpenOCD does not yet have the MAX32's flash algorithm integrated.
# If the MSDK is installed, flash-msdk can be run to utilize the the modified
# openocd with the algorithms
MAXIM_PATH := $(subst \,/,$(MAXIM_PATH))
flash-msdk: $(BUILD)/$(PROJECT).elf
	$(MAXIM_PATH)/Tools/OpenOCD/openocd -s $(MAXIM_PATH)/Tools/OpenOCD/scripts \
		-f interface/cmsis-dap.cfg -f target/max32665.cfg \
		-c "program $(BUILD)/$(PROJECT).elf verify; init; reset; exit"

# -----------------
# Sources & Include
# -----------------
PERIPH_SRC = $(TOP)/$(MAX32_PERIPH)/Source
SRC_C += \
	src/portable/mentor/musb/dcd_musb.c \
	$(MAX32_CMSIS)/Device/Maxim/MAX32665/Source/heap.c \
	$(MAX32_CMSIS)/Device/Maxim/MAX32665/Source/system_max32665.c \
	$(PERIPH_SRC)/SYS/mxc_assert.c \
	$(PERIPH_SRC)/SYS/mxc_delay.c \
	$(PERIPH_SRC)/SYS/mxc_lock.c \
	$(PERIPH_SRC)/SYS/nvic_table.c \
	$(PERIPH_SRC)/SYS/pins_me14.c \
	$(PERIPH_SRC)/SYS/sys_me14.c \
	$(PERIPH_SRC)/FLC/flc_common.c \
	$(PERIPH_SRC)/FLC/flc_me14.c \
	$(PERIPH_SRC)/FLC/flc_reva.c \
	$(PERIPH_SRC)/GPIO/gpio_common.c \
	$(PERIPH_SRC)/GPIO/gpio_me14.c \
	$(PERIPH_SRC)/GPIO/gpio_reva.c \
	$(PERIPH_SRC)/ICC/icc_me14.c \
	$(PERIPH_SRC)/ICC/icc_reva.c \
	$(PERIPH_SRC)/TPU/tpu_me14.c \
    $(PERIPH_SRC)/TPU/tpu_reva.c \
	$(PERIPH_SRC)/UART/uart_common.c \
	$(PERIPH_SRC)/UART/uart_me14.c \
	$(PERIPH_SRC)/UART/uart_reva.c \

INC += \
	$(TOP)/$(BOARD_PATH) \
	$(TOP)/$(MAX32_CMSIS)/Include \
	$(TOP)/$(MAX32_CMSIS)/Device/Maxim/MAX32665/Include \
	$(TOP)/$(MAX32_PERIPH)/Include/MAX32665 \
	$(PERIPH_SRC)/SYS \
	$(PERIPH_SRC)/GPIO \
	$(PERIPH_SRC)/ICC \
	$(PERIPH_SRC)/FLC \
	$(PERIPH_SRC)/TPU \
	$(PERIPH_SRC)/UART
