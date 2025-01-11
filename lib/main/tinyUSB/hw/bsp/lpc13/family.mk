DEPS_SUBMODULES += hw/mcu/nxp/lpcopen

MCU_DIR = hw/mcu/nxp/lpcopen/lpc13xx/lpc_chip_13xx
include $(TOP)/$(BOARD_PATH)/board.mk
CPU_CORE ?= cortex-m3

CFLAGS += \
  -flto \
  -nostdlib \
  -DCORE_M3 \
  -D__USE_LPCOPEN \
  -DCFG_EXAMPLE_MSC_READONLY \
  -DCFG_EXAMPLE_VIDEO_READONLY \
  -DCFG_TUSB_MCU=OPT_MCU_LPC13XX \
  -DCFG_TUSB_MEM_ALIGN='__attribute__((aligned(64)))'

LDFLAGS_GCC += -specs=nosys.specs -specs=nano.specs

# startup.c and lpc_types.h cause following errors
CFLAGS += -Wno-error=strict-prototypes -Wno-error=redundant-decls

# caused by freeRTOS port !!
CFLAGS += -Wno-error=maybe-uninitialized

SRC_C += \
	src/portable/nxp/lpc_ip3511/dcd_lpc_ip3511.c \
	$(MCU_DIR)/../gcc/cr_startup_lpc13xx.c \
	$(MCU_DIR)/src/chip_13xx.c \
	$(MCU_DIR)/src/clock_13xx.c \
	$(MCU_DIR)/src/gpio_13xx_1.c \
	$(MCU_DIR)/src/iocon_13xx.c \
	$(MCU_DIR)/src/sysctl_13xx.c \
	$(MCU_DIR)/src/sysinit_13xx.c

INC += \
  $(TOP)/$(BOARD_PATH) \
  $(TOP)/$(MCU_DIR)/inc
