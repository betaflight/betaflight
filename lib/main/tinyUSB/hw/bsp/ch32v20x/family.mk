# https://www.embecosm.com/resources/tool-chain-downloads/#riscv-stable
#CROSS_COMPILE ?= riscv32-unknown-elf-

# Toolchain from https://nucleisys.com/download.php
#CROSS_COMPILE ?= riscv-nuclei-elf-

# Toolchain from https://github.com/xpack-dev-tools/riscv-none-elf-gcc-xpack
CROSS_COMPILE ?= riscv-none-elf-

CH32_FAMILY = ch32v20x
SDK_DIR = hw/mcu/wch/ch32v20x
SDK_SRC_DIR = $(SDK_DIR)/EVT/EXAM/SRC

include $(TOP)/$(BOARD_PATH)/board.mk
CPU_CORE ?= rv32imac-ilp32

# Port0 use FSDev, Port1 use USBFS
PORT ?= 0

CFLAGS += \
	-mcmodel=medany \
	-ffat-lto-objects \
	-flto \
	-DCH32V20x_${MCU_VARIANT} \
	-DCFG_TUSB_MCU=OPT_MCU_CH32V20X

# https://github.com/openwch/ch32v20x/pull/12
CFLAGS += -Wno-error=strict-prototypes

ifeq ($(PORT),0)
  $(info "Using FSDEV driver")
  CFLAGS += -DCFG_TUD_WCH_USBIP_FSDEV=1
else
  $(info "Using USBFS driver")
  CFLAGS += -DCFG_TUD_WCH_USBIP_USBFS=1
endif

LDFLAGS_GCC += \
	-nostdlib -nostartfiles \
	--specs=nosys.specs --specs=nano.specs \

LD_FILE = $(FAMILY_PATH)/linker/${CH32_FAMILY}.ld

SRC_C += \
	src/portable/wch/dcd_ch32_usbfs.c \
	src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c \
	$(SDK_SRC_DIR)/Core/core_riscv.c \
	$(SDK_SRC_DIR)/Peripheral/src/${CH32_FAMILY}_gpio.c \
	$(SDK_SRC_DIR)/Peripheral/src/${CH32_FAMILY}_misc.c \
	$(SDK_SRC_DIR)/Peripheral/src/${CH32_FAMILY}_rcc.c \
	$(SDK_SRC_DIR)/Peripheral/src/${CH32_FAMILY}_usart.c \

SRC_S += $(SDK_SRC_DIR)/Startup/startup_${CH32_FAMILY}_${MCU_VARIANT}.S

INC += \
	$(TOP)/$(BOARD_PATH) \
	$(TOP)/$(SDK_SRC_DIR)/Core \
	$(TOP)/$(SDK_SRC_DIR)/Peripheral/inc \

FREERTOS_PORTABLE_SRC = $(FREERTOS_PORTABLE_PATH)/RISC-V

OPENOCD_WCH_OPTION=-f $(TOP)/$(FAMILY_PATH)/wch-riscv.cfg
flash: flash-wlink-rs
#flash: flash-openocd-wch
