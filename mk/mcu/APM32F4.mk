#
# APM32F4 Make file include
#

#CMSIS
CMSIS_DIR      := $(ROOT)/lib/main/APM32F4/Libraries/Device
STDPERIPH_DIR   = $(ROOT)/lib/main/APM32F4/Libraries/APM32F4xx_DAL_Driver
STDPERIPH_SRC   = $(notdir $(wildcard $(STDPERIPH_DIR)/Source/*.c))
EXCLUDES        = apm32f4xx_dal_timebase_rtc_alarm_template.c \
                  apm32f4xx_dal_timebase_rtc_wakeup_template.c \
                  apm32f4xx_dal_timebase_tmr_template.c \
                  apm32f4xx_device_cfg_template.c

STDPERIPH_SRC   := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))

VPATH       := $(VPATH):$(STDPERIPH_DIR)/Source

#USB
USBCORE_DIR = $(ROOT)/lib/main/APM32F4/Middlewares/APM32_USB_Library/Device/Core
USBCORE_SRC = $(notdir $(wildcard $(USBCORE_DIR)/Src/*.c))
USBCORE_SRC := $(filter-out ${EXCLUDES}, $(USBCORE_SRC))

USBCDC_DIR = $(ROOT)/lib/main/APM32F4/Middlewares/APM32_USB_Library/Device/Class/CDC
USBCDC_SRC = $(notdir $(wildcard $(USBCDC_DIR)/Src/*.c))
USBCDC_SRC := $(filter-out ${EXCLUDES}, $(USBCDC_SRC))

USBMSC_DIR = $(ROOT)/lib/main/APM32F4/Middlewares/APM32_USB_Library/Device/Class/MSC
USBMSC_SRC = $(notdir $(wildcard $(USBMSC_DIR)/Src/*.c))
USBMSC_SRC := $(filter-out ${EXCLUDES}, $(USBMSC_SRC))

VPATH := $(VPATH):$(USBCDC_DIR)/Src:$(USBCORE_DIR)/Src:$(USBMSC_DIR)/Src

DEVICE_STDPERIPH_SRC := $(STDPERIPH_SRC) \
                        $(USBCORE_SRC) \
                        $(USBCDC_SRC) \
                        $(USBMSC_SRC)
#CMSIS
VPATH           := $(VPATH):$(ROOT)/lib/main/APM32F4/Libraries/Device/Geehy/APM32F4xx

INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(SRC_DIR)/startup/apm32 \
                   $(SRC_DIR)/drivers/mcu/apm32

CMSIS_SRC       :=
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(STDPERIPH_DIR)/Include \
                   $(USBCORE_DIR)/Inc \
                   $(USBCDC_DIR)/Inc \
                   $(USBMSC_DIR)/Inc \
                   $(CMSIS_DIR)/Geehy/APM32F4xx/Include \
                   $(SRC_DIR)/drivers/mcu/apm32/usb/vcp \
                   $(SRC_DIR)/drivers/mcu/apm32/usb/msc \
                   $(SRC_DIR)/drivers/mcu/apm32/usb \
                   $(ROOT)/lib/main/CMSIS/Core/Include \
                   $(SRC_DIR)/msc

#Flags
ARCH_FLAGS      = -mthumb -mcpu=cortex-m4 -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant

ifeq ($(TARGET_MCU),APM32F405xx)
DEVICE_FLAGS    = -DUSE_DAL_DRIVER -DAPM32F405xx -DHSE_VALUE=$(HSE_VALUE) -DAPM32
LD_SCRIPT       = $(LINKER_DIR)/apm32_flash_f405.ld
STARTUP_SRC     = apm32/startup_apm32f405xx.S
MCU_FLASH_SIZE  := 1024

else ifeq ($(TARGET_MCU),APM32F407xx)
DEVICE_FLAGS    = -DUSE_DAL_DRIVER -DAPM32F407xx -DHSE_VALUE=$(HSE_VALUE) -DAPM32
LD_SCRIPT       = $(LINKER_DIR)/apm32_flash_f407.ld
STARTUP_SRC     = apm32/startup_apm32f407xx.S
MCU_FLASH_SIZE  := 1024
else
$(error TARGET_MCU [$(TARGET_MCU] is not supported)
endif

MCU_COMMON_SRC = \
            startup/apm32/system_apm32f4xx.c \
            drivers/inverter.c \
            drivers/dshot_bitbang_decode.c \
            drivers/pwm_output_dshot_shared.c \
            drivers/mcu/apm32/bus_spi_apm32.c \
            drivers/mcu/apm32/bus_i2c_apm32.c \
            drivers/mcu/apm32/bus_i2c_apm32_init.c \
            drivers/mcu/apm32/camera_control.c \
            drivers/mcu/apm32/debug.c \
            drivers/mcu/apm32/dma_reqmap_mcu.c \
            drivers/mcu/apm32/dshot_bitbang.c \
            drivers/mcu/apm32/dshot_bitbang_ddl.c \
            drivers/mcu/apm32/eint_apm32.c \
            drivers/mcu/apm32/io_apm32.c \
            drivers/mcu/apm32/light_ws2811strip_apm32.c \
            drivers/mcu/apm32/persistent_apm32.c \
            drivers/mcu/apm32/pwm_output_apm32.c \
            drivers/mcu/apm32/pwm_output_dshot_apm32.c \
            drivers/mcu/apm32/rcm_apm32.c \
            drivers/mcu/apm32/serial_uart_apm32.c \
            drivers/mcu/apm32/timer_apm32.c \
            drivers/mcu/apm32/transponder_ir_io_apm32.c \
            drivers/mcu/apm32/timer_apm32f4xx.c \
            drivers/mcu/apm32/adc_apm32f4xx.c \
            drivers/mcu/apm32/dma_apm32f4xx.c \
            drivers/mcu/apm32/serial_uart_apm32f4xx.c \
            drivers/mcu/apm32/system_apm32f4xx.c

VCP_SRC = \
            drivers/mcu/apm32/usb/vcp/usbd_cdc_descriptor.c \
            drivers/mcu/apm32/usb/usbd_board_apm32f4.c \
            drivers/mcu/apm32/usb/vcp/usbd_cdc_vcp.c \
            drivers/mcu/apm32/usb/vcp/serial_usb_vcp.c \
            drivers/usb_io.c

MSC_SRC = \
            drivers/usb_msc_common.c \
            drivers/mcu/apm32/usb/msc/usb_msc_apm32f4xx.c \
            drivers/mcu/apm32/usb/msc/usbd_memory.c \
            drivers/mcu/apm32/usb/msc/usbd_msc_descriptor.c \
            msc/usbd_storage.c \
            msc/usbd_storage_emfat.c \
            msc/emfat.c \
            msc/emfat_file.c \
            msc/usbd_storage_sd_spi.c \
            msc/usbd_storage_sdio.c

DSP_LIB := $(ROOT)/lib/main/CMSIS/DSP
DEVICE_FLAGS += -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -DUNALIGNED_SUPPORT_DISABLE -DARM_MATH_CM4 -DUSE_FULL_DDL_DRIVER
