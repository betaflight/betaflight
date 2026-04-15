#
# C5 Make file include
#

# Auto-hydrate STM32CubeC5 submodule when building C5 targets
PLATFORM_SDK := stm32c5
PLATFORM_SDK_STAMP := $(STM32C5_SDK_STAMP)

ifeq ($(DEBUG_HARDFAULTS),C5)
CFLAGS          += -DDEBUG_HARDFAULTS
endif

# STM32CubeC5 uses a restructured "Cube 2.0" layout:
#   stm32c5xx_dfp/Include/          - CMSIS device headers
#   stm32c5xx_drivers/hal/          - HAL .c and .h (flat, no Inc/Src subdirs)
#   stm32c5xx_drivers/ll/           - LL .h headers (header-only)
#   arch/cmsis/CMSIS/Core/Include/  - CMSIS Core headers

STM32C5_LIB     := $(LIB_MAIN_DIR)/STM32C5

#STDPERIPH
STDPERIPH_DIR   = $(STM32C5_LIB)/stm32c5xx_drivers/hal
# HAL2 (Cube 2.0) has no separate _ex.c files; extensions are merged into the main source.
STDPERIPH_SRC   = \
            stm32c5xx_hal.c \
            stm32c5xx_hal_adc.c \
            stm32c5xx_hal_cortex.c \
            stm32c5xx_hal_dma.c \
            stm32c5xx_hal_exti.c \
            stm32c5xx_hal_flash.c \
            stm32c5xx_hal_flash_itf.c \
            stm32c5xx_hal_gpio.c \
            stm32c5xx_hal_i2c.c \
            stm32c5xx_hal_pcd.c \
            stm32c5xx_usb_drd_core.c \
            stm32c5xx_hal_pwr.c \
            stm32c5xx_hal_rcc.c \
            stm32c5xx_hal_rng.c \
            stm32c5xx_hal_rtc.c \
            stm32c5xx_hal_spi.c \
            stm32c5xx_hal_tim.c \
            stm32c5xx_hal_tamp.c \
            stm32c5xx_hal_uart.c \
            stm32c5xx_hal_usart.c

#USB
USBCORE_DIR = STM32_USB_Device_Library_HAL/Core
USBCORE_SRC = \
            $(USBCORE_DIR)/Src/usbd_core.c \
            $(USBCORE_DIR)/Src/usbd_ctlreq.c \
            $(USBCORE_DIR)/Src/usbd_ioreq.c

USBCDC_DIR = STM32_USB_Device_Library_HAL/Class/CDC
USBCDC_SRC = \
            $(USBCDC_DIR)/Src/usbd_cdc.c

DEVICE_STDPERIPH_SRC := $(STDPERIPH_SRC) \
                        $(USBCORE_SRC) \
                        $(USBCDC_SRC)

#CMSIS
VPATH           := $(VPATH):$(STDPERIPH_DIR):$(STM32C5_LIB)/stm32c5xx_dfp/Include:$(STM32C5_LIB)/stm32c5xx_dfp/Source/Templates
CMSIS_SRC       :=
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(TARGET_PLATFORM_DIR) \
                   $(TARGET_PLATFORM_DIR)/include \
                   $(TARGET_PLATFORM_DIR)/startup \
                   $(STDPERIPH_DIR) \
                   $(STM32C5_LIB)/stm32c5xx_drivers/ll \
                   $(LIB_MAIN_DIR)/$(USBCORE_DIR)/Inc \
                   $(LIB_MAIN_DIR)/$(USBCDC_DIR)/Inc \
                   $(STM32C5_LIB)/arch/cmsis/CMSIS/Core/Include \
                   $(STM32C5_LIB)/stm32c5xx_dfp/Include \
                   $(TARGET_PLATFORM_DIR)/vcp_hal

#Flags
ARCH_FLAGS      = -mthumb -mcpu=cortex-m33 -mfloat-abi=hard -mfpu=fpv5-sp-d16

# Flags that are used in the STM32 libraries
DEVICE_FLAGS    = -DUSE_HAL_DRIVER -DUSE_FULL_LL_DRIVER

# Suppress warnings in vendor HAL2 sources (ST coding style differences)
SRC_CFLAGS_stm32c5xx_hal_adc.c := -Wno-unused-parameter

ifeq ($(TARGET_MCU),STM32C591xx)
DEVICE_FLAGS       += -DSTM32C591xx
DEFAULT_LD_SCRIPT   = $(LINKER_DIR)/stm32_flash_c5xx_1m.ld
STARTUP_SRC         = STM32/startup/startup_stm32c591xx.s
MCU_FLASH_SIZE     := 1024
DEVICE_FLAGS       += -DMAX_MPU_REGIONS=16
else
$(error Unknown MCU for STM32C5 target)
endif

ifneq ($(DEBUG),GDB)
OPTIMISE_DEFAULT    := -Os
OPTIMISE_SPEED      := -Os
OPTIMISE_SIZE       := -Os
LTO_FLAGS           := $(OPTIMISATION_BASE) $(OPTIMISE_DEFAULT)
endif

ifeq ($(LD_SCRIPT),)
LD_SCRIPT = $(DEFAULT_LD_SCRIPT)
endif

ifneq ($(FIRMWARE_SIZE),)
DEVICE_FLAGS   += -DFIRMWARE_SIZE=$(FIRMWARE_SIZE)
endif

DEVICE_FLAGS    += -DHSE_VALUE=$(HSE_VALUE) -DHSE_STARTUP_TIMEOUT=1000 -DSTM32

VCP_SRC = \
            STM32/vcp_hal/usbd_desc.c \
            STM32/vcp_hal/usbd_conf_stm32c5xx.c \
            STM32/vcp_hal/usbd_cdc_hid.c \
            STM32/vcp_hal/usbd_cdc_interface.c \
            STM32/serial_usb_vcp.c \
            drivers/usb_io.c

# Files that compile cleanly with HAL2, plus HAL2-specific forks.
# Peripheral drivers that deeply use old HAL types (dshot_bitbang_ll, etc.)
# need HAL2 forks and are excluded for now.
# A stub file provides weak symbols for the missing functions.
MCU_COMMON_SRC = \
            STM32/system_stm32c5xx.c \
            STM32/memprot_stm32c5xx.c \
            STM32/persistent_hal2.c \
            STM32/debug.c \
            STM32/io_stm32.c \
            STM32/exti.c \
            STM32/rcc_stm32_hal2.c \
            STM32/timer_hal2.c \
            STM32/timer_stm32c5xx.c \
            STM32/bus_spi_hal2.c \
            STM32/bus_i2c_ll.c \
            STM32/bus_i2c_ll_init.c \
            drivers/bus_i2c_timing.c \
            STM32/serial_uart_ll.c \
            STM32/serial_uart_stm32c5xx.c \
            drivers/adc.c \
            STM32/adc_stm32c5xx.c \
            STM32/dma_stm32c5xx.c \
            STM32/dma_reqmap_mcu.c \
            STM32/stubs_stm32c5xx.c \
            STM32/startup/system_stm32c5xx.c

MSC_SRC =

SPEED_OPTIMISED_SRC +=

SIZE_OPTIMISED_SRC +=

DSP_LIB := $(STM32C5_LIB)/arch/cmsis/CMSIS/DSP
DEVICE_FLAGS += -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -DUNALIGNED_SUPPORT_DISABLE -DARM_MATH_CM33

include $(TARGET_PLATFORM_DIR)/mk/STM32_COMMON.mk
