MCU_DIR = hw/mcu/nxp/lpcopen/lpc$(MCU)/lpc_chip_$(MCU)
include $(TOP)/$(BOARD_PATH)/board.mk
CPU_CORE ?= cortex-m0plus

CFLAGS += \
  -flto \
  -nostdlib \
  -D__USE_LPCOPEN \
  -DCFG_TUSB_MCU=OPT_MCU_LPC11UXX \
  -DCFG_TUSB_MEM_ALIGN='__attribute__((aligned(64)))'

# mcu driver cause following warnings
CFLAGS += \
  -Wno-error=incompatible-pointer-types \

LDFLAGS_GCC += --specs=nosys.specs --specs=nano.specs

SRC_C += \
	src/portable/nxp/lpc_ip3511/dcd_lpc_ip3511.c \
	$(MCU_DIR)/../gcc/cr_startup_lpc$(MCU_DRV).c \
	$(MCU_DIR)/src/chip_$(MCU_DRV).c \
	$(MCU_DIR)/src/clock_$(MCU_DRV).c \
	$(MCU_DIR)/src/iap.c \
	$(MCU_DIR)/src/iocon_$(MCU_DRV).c \
	$(MCU_DIR)/src/sysinit_$(MCU_DRV).c

ifeq ($(MCU),11u6x)
SRC_C += \
	$(MCU_DIR)/src/gpio_$(MCU_DRV).c \
	$(MCU_DIR)/src/syscon_$(MCU_DRV).c \

else

SRC_C += \
	$(MCU_DIR)/src/gpio_$(MCU_DRV)_1.c \
	$(MCU_DIR)/src/sysctl_$(MCU_DRV).c
endif

INC += \
	$(TOP)/$(BOARD_PATH) \
	$(TOP)/$(MCU_DIR)/inc \

# For flash-jlink target
JLINK_DEVICE = LPC11U68
