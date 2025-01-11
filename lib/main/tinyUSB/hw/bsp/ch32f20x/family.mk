# Submodules
CH32F20X_SDK = hw/mcu/wch/ch32f20x
DEPS_SUBMODULES += $(CH32F20X_SDK)

# WCH-SDK paths
CH32F20X_SDK_SRC = $(CH32F20X_SDK)/EVT/EXAM/SRC

include $(TOP)/$(BOARD_PATH)/board.mk

CPU_CORE ?= cortex-m3

CFLAGS += \
	-DCFG_TUSB_MCU=OPT_MCU_CH32F20X \
	-DBOARD_TUD_MAX_SPEED=OPT_MODE_HIGH_SPEED

SRC_C += \
	src/portable/wch/dcd_ch32_usbhs.c \
	$(CH32F20X_SDK_SRC)/StdPeriphDriver/src/ch32f20x_gpio.c \
	$(CH32F20X_SDK_SRC)/StdPeriphDriver/src/ch32f20x_misc.c \
	$(CH32F20X_SDK_SRC)/StdPeriphDriver/src/ch32f20x_rcc.c \
	$(CH32F20X_SDK_SRC)/StdPeriphDriver/src/ch32f20x_usart.c

INC += \
	$(TOP)/$(BOARD_PATH) \
	$(TOP)/$(CH32F20X_SDK_SRC)/StdPeriphDriver/inc

# For freeRTOS port source
FREERTOS_PORTABLE_SRC = $(FREERTOS_PORTABLE_PATH)/ARM_CM3

flash: flash-stlink
