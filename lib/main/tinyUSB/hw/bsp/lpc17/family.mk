DEPS_SUBMODULES += hw/mcu/nxp/lpcopen

MCU_DIR = hw/mcu/nxp/lpcopen/lpc175x_6x/lpc_chip_175x_6x
include $(TOP)/$(BOARD_PATH)/board.mk
CPU_CORE ?= cortex-m3

CFLAGS += \
  -flto \
  -nostdlib \
  -DCORE_M3 \
  -D__USE_LPCOPEN \
  -DCFG_TUSB_MCU=OPT_MCU_LPC175X_6X \
  -DRTC_EV_SUPPORT=0

# lpc_types.h cause following errors
CFLAGS += -Wno-error=strict-prototypes -Wno-error=cast-qual

# caused by freeRTOS port !!
CFLAGS += -Wno-error=maybe-uninitialized

LDFLAGS_GCC += --specs=nosys.specs --specs=nano.specs

SRC_C += \
	src/portable/nxp/lpc17_40/dcd_lpc17_40.c \
	src/portable/nxp/lpc17_40/hcd_lpc17_40.c \
	src/portable/ohci/ohci.c \
	$(MCU_DIR)/../gcc/cr_startup_lpc175x_6x.c \
	$(MCU_DIR)/src/chip_17xx_40xx.c \
	$(MCU_DIR)/src/clock_17xx_40xx.c \
	$(MCU_DIR)/src/gpio_17xx_40xx.c \
	$(MCU_DIR)/src/iocon_17xx_40xx.c \
	$(MCU_DIR)/src/sysctl_17xx_40xx.c \
	$(MCU_DIR)/src/sysinit_17xx_40xx.c \
	$(MCU_DIR)/src/uart_17xx_40xx.c \

INC += \
	$(TOP)/$(BOARD_PATH) \
	$(TOP)/$(MCU_DIR)/inc \
	$(TOP)/lib/CMSIS_5/CMSIS/Core/Include \
