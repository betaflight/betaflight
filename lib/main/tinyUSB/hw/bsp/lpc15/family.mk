DEPS_SUBMODULES += hw/mcu/nxp/lpcopen

include $(TOP)/$(BOARD_PATH)/board.mk
CPU_CORE ?= cortex-m3

CFLAGS += \
  -flto \
  -nostdlib \
  -DCORE_M3 \
  -D__USE_LPCOPEN \
  -DCFG_EXAMPLE_MSC_READONLY \
  -DCFG_TUSB_MCU=OPT_MCU_LPC15XX \
  -DCFG_TUSB_MEM_ALIGN='__attribute__((aligned(64)))'

LDFLAGS_GCC += -specs=nosys.specs -specs=nano.specs

# mcu driver cause following warnings
CFLAGS += -Wno-error=strict-prototypes -Wno-error=unused-parameter -Wno-error=unused-variable -Wno-error=cast-qual

MCU_DIR = hw/mcu/nxp/lpcopen/lpc15xx/lpc_chip_15xx

SRC_C += \
	src/portable/nxp/lpc_ip3511/dcd_lpc_ip3511.c \
	$(MCU_DIR)/../gcc/cr_startup_lpc15xx.c \
	$(MCU_DIR)/src/chip_15xx.c \
	$(MCU_DIR)/src/clock_15xx.c \
	$(MCU_DIR)/src/gpio_15xx.c \
	$(MCU_DIR)/src/iocon_15xx.c \
	$(MCU_DIR)/src/swm_15xx.c \
	$(MCU_DIR)/src/sysctl_15xx.c \
	$(MCU_DIR)/src/sysinit_15xx.c \
	$(MCU_DIR)/src/uart_15xx.c \

INC += \
	$(TOP)/$(BOARD_PATH) \
	$(TOP)/$(MCU_DIR)/inc
