UF2_FAMILY_ID = 0x2abc77ec
SDK_DIR = hw/mcu/nxp/mcux-sdk
DEPS_SUBMODULES += lib/CMSIS_5 lib/sct_neopixel $(SDK_DIR)

include $(TOP)/$(BOARD_PATH)/board.mk
CPU_CORE ?= cortex-m33
MCU_DIR = $(SDK_DIR)/devices/$(MCU_VARIANT)

# Default to Highspeed PORT1
PORT ?= 1

CFLAGS += \
  -flto \
  -D__STARTUP_CLEAR_BSS \
  -DCFG_TUSB_MCU=OPT_MCU_LPC55XX \
  -DCFG_TUSB_MEM_ALIGN='__attribute__((aligned(64)))' \
  -DBOARD_TUD_RHPORT=$(PORT) \

ifeq ($(PORT), 1)
  $(info "PORT1 High Speed")
  CFLAGS += -DBOARD_TUD_MAX_SPEED=OPT_MODE_HIGH_SPEED

  # LPC55 Highspeed Port1 can only write to USB_SRAM region
  CFLAGS += -DCFG_TUSB_MEM_SECTION='__attribute__((section("m_usb_global")))'
else
  $(info "PORT0 Full Speed")
endif

# mcu driver cause following warnings
CFLAGS += -Wno-error=unused-parameter -Wno-error=float-equal

LDFLAGS_GCC += \
  -nostartfiles \
  --specs=nosys.specs --specs=nano.specs \

# All source paths should be relative to the top level.
LD_FILE ?= $(MCU_DIR)/gcc/$(MCU_CORE)_flash.ld

SRC_C += \
	src/portable/nxp/lpc_ip3511/dcd_lpc_ip3511.c \
	$(MCU_DIR)/system_$(MCU_CORE).c \
	$(MCU_DIR)/drivers/fsl_clock.c \
	$(MCU_DIR)/drivers/fsl_power.c \
	$(MCU_DIR)/drivers/fsl_reset.c \
	$(SDK_DIR)/drivers/lpc_gpio/fsl_gpio.c \
	$(SDK_DIR)/drivers/common/fsl_common_arm.c \
	$(SDK_DIR)/drivers/flexcomm/fsl_flexcomm.c \
	$(SDK_DIR)/drivers/flexcomm/fsl_usart.c \
	lib/sct_neopixel/sct_neopixel.c

INC += \
	$(TOP)/$(BOARD_PATH) \
	$(TOP)/lib/sct_neopixel \
	$(TOP)/lib/CMSIS_5/CMSIS/Core/Include \
	$(TOP)/$(MCU_DIR) \
	$(TOP)/$(MCU_DIR)/drivers \
	$(TOP)/$(SDK_DIR)/drivers/common \
	$(TOP)/$(SDK_DIR)/drivers/flexcomm \
	$(TOP)/$(SDK_DIR)/drivers/lpc_iocon \
	$(TOP)/$(SDK_DIR)/drivers/lpc_gpio \
	$(TOP)/$(SDK_DIR)/drivers/sctimer

SRC_S += $(MCU_DIR)/gcc/startup_$(MCU_CORE).S

LIBS += $(TOP)/$(MCU_DIR)/gcc/libpower_hardabi.a
