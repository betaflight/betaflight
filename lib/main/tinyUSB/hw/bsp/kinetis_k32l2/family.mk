UF2_FAMILY_ID = 0x7f83e793
SDK_DIR = hw/mcu/nxp/mcux-sdk
MCU_DIR = $(SDK_DIR)/devices/$(MCU)

include $(TOP)/$(BOARD_PATH)/board.mk
CPU_CORE ?= cortex-m0plus

CFLAGS += \
	-DCFG_TUSB_MCU=OPT_MCU_KINETIS_K32L

LDFLAGS_GCC += \
  -nostartfiles \
  -specs=nosys.specs -specs=nano.specs

SRC_C += \
	src/portable/nxp/khci/dcd_khci.c \
	src/portable/nxp/khci/hcd_khci.c \
	$(MCU_DIR)/system_$(MCU).c \
	$(MCU_DIR)/drivers/fsl_clock.c \
	$(SDK_DIR)/drivers/gpio/fsl_gpio.c \
	$(SDK_DIR)/drivers/lpuart/fsl_lpuart.c

INC += \
	$(TOP)/$(BOARD_PATH) \
	$(TOP)/lib/CMSIS_5/CMSIS/Core/Include \
	$(TOP)/$(MCU_DIR) \
	$(TOP)/$(MCU_DIR)/project_template \
	$(TOP)/$(MCU_DIR)/drivers \
	$(TOP)/$(SDK_DIR)/drivers/common \
	$(TOP)/$(SDK_DIR)/drivers/gpio \
	$(TOP)/$(SDK_DIR)/drivers/lpuart \
	$(TOP)/$(SDK_DIR)/drivers/port \
	$(TOP)/$(SDK_DIR)/drivers/smc \

SRC_S += $(MCU_DIR)/gcc/startup_$(MCU).S
