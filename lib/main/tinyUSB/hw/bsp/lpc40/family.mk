DEPS_SUBMODULES += hw/mcu/nxp/lpcopen

MCU_DIR = hw/mcu/nxp/lpcopen/lpc40xx/lpc_chip_40xx
include $(TOP)/$(BOARD_PATH)/board.mk
CPU_CORE ?= cortex-m4

CFLAGS += \
  -flto \
  -nostdlib \
  -DCORE_M4 \
  -D__USE_LPCOPEN \
  -DCFG_TUSB_MEM_SECTION='__attribute__((section(".data.$$RAM2")))' \
  -DCFG_TUSB_MCU=OPT_MCU_LPC40XX

# mcu driver cause following warnings
CFLAGS += -Wno-error=strict-prototypes -Wno-error=unused-parameter -Wno-error=cast-qual

LDFLAGS_GCC += --specs=nosys.specs --specs=nano.specs

# All source paths should be relative to the top level.
SRC_C += \
	src/portable/nxp/lpc17_40/dcd_lpc17_40.c \
	src/portable/nxp/lpc17_40/hcd_lpc17_40.c \
	src/portable/ohci/ohci.c \
	$(MCU_DIR)/../gcc/cr_startup_lpc40xx.c \
	$(MCU_DIR)/src/chip_17xx_40xx.c \
	$(MCU_DIR)/src/clock_17xx_40xx.c \
	$(MCU_DIR)/src/fpu_init.c \
	$(MCU_DIR)/src/gpio_17xx_40xx.c \
	$(MCU_DIR)/src/iocon_17xx_40xx.c \
	$(MCU_DIR)/src/sysctl_17xx_40xx.c \
	$(MCU_DIR)/src/sysinit_17xx_40xx.c \
	$(MCU_DIR)/src/uart_17xx_40xx.c \

INC += \
	$(TOP)/$(MCU_DIR)/inc \
	$(TOP)/lib/CMSIS_5/CMSIS/Core/Include \
	$(TOP)/$(BOARD_PATH)
