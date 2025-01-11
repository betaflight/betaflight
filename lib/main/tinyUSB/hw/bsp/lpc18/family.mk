DEPS_SUBMODULES += hw/mcu/nxp/lpcopen
MCU_DIR = hw/mcu/nxp/lpcopen/lpc18xx/lpc_chip_18xx

include $(TOP)/$(BOARD_PATH)/board.mk
CPU_CORE ?= cortex-m3

CFLAGS += \
  -flto \
  -nostdlib \
  -DCORE_M3 \
  -D__USE_LPCOPEN \
  -DCFG_TUSB_MCU=OPT_MCU_LPC18XX

# mcu driver cause following warnings
CFLAGS += -Wno-error=unused-parameter -Wno-error=strict-prototypes -Wno-error=cast-qual

LDFLAGS_GCC += --specs=nosys.specs --specs=nano.specs

SRC_C += \
	src/portable/chipidea/ci_hs/dcd_ci_hs.c \
	src/portable/chipidea/ci_hs/hcd_ci_hs.c \
	src/portable/ehci/ehci.c \
	$(MCU_DIR)/../gcc/cr_startup_lpc18xx.c \
	$(MCU_DIR)/src/chip_18xx_43xx.c \
	$(MCU_DIR)/src/clock_18xx_43xx.c \
	$(MCU_DIR)/src/gpio_18xx_43xx.c \
	$(MCU_DIR)/src/sysinit_18xx_43xx.c \
	$(MCU_DIR)/src/uart_18xx_43xx.c

INC += \
	$(TOP)/$(BOARD_PATH) \
	$(TOP)/$(MCU_DIR)/inc \
	$(TOP)/$(MCU_DIR)/inc/config_18xx \
	$(TOP)/lib/CMSIS_5/CMSIS/Core/Include \
