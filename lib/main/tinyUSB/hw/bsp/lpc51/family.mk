SDK_DIR = hw/mcu/nxp/mcux-sdk

include $(TOP)/$(BOARD_PATH)/board.mk
MCU_DIR = $(SDK_DIR)/devices/$(MCU)
CPU_CORE ?= cortex-m0plus

CFLAGS += \
  -flto \
  -D__STARTUP_CLEAR_BSS \
  -DCFG_TUSB_MCU=OPT_MCU_LPC51UXX \
  -DCFG_TUSB_MEM_ALIGN='__attribute__((aligned(64)))'

# mcu driver cause following warnings
CFLAGS += -Wno-error=unused-parameter

LDFLAGS_GCC += \
  -nostartfiles \
  --specs=nosys.specs --specs=nano.specs \

# All source paths should be relative to the top level.
LD_FILE = $(MCU_DIR)/gcc/$(MCU)_flash.ld

SRC_C += \
	src/portable/nxp/lpc_ip3511/dcd_lpc_ip3511.c \
	$(MCU_DIR)/system_$(MCU).c \
	$(MCU_DIR)/drivers/fsl_clock.c \
	$(MCU_DIR)/drivers/fsl_power.c \
	$(MCU_DIR)/drivers/fsl_reset.c \
	$(SDK_DIR)/drivers/lpc_gpio/fsl_gpio.c \
	$(SDK_DIR)/drivers/flexcomm/fsl_flexcomm.c \
	$(SDK_DIR)/drivers/flexcomm/fsl_usart.c

INC += \
  $(TOP)/$(BOARD_PATH) \
  $(TOP)/lib/CMSIS_5/CMSIS/Core/Include \
	$(TOP)/$(MCU_DIR) \
	$(TOP)/$(MCU_DIR)/drivers \
	$(TOP)/$(SDK_DIR)/drivers/common \
	$(TOP)/$(SDK_DIR)/drivers/flexcomm \
	$(TOP)/$(SDK_DIR)/drivers/lpc_iocon \
	$(TOP)/$(SDK_DIR)/drivers/lpc_gpio

SRC_S += $(MCU_DIR)/gcc/startup_$(MCU).S

LIBS += $(TOP)/$(MCU_DIR)/gcc/libpower.a
