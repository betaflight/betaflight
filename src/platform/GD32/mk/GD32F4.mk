#
# GD32F4 Make file include
#

CMSIS_DIR      := $(LIB_MAIN_DIR)/CMSIS
STDPERIPH_DIR   = $(LIB_MAIN_DIR)/GD32F4/Drivers/GD32F4xx_standard_peripheral
USB_LIB_DIR    := $(LIB_MAIN_DIR)/GD32F4/Middlewares/GD32F4xx_usb_library

STDPERIPH_SRC   = \
            gd32f4xx_adc.c \
            gd32f4xx_dac.c \
            gd32f4xx_dci.c \
            gd32f4xx_dma.c \
            gd32f4xx_exti.c \
            gd32f4xx_fmc.c \
            gd32f4xx_fwdgt.c \
            gd32f4xx_gpio.c \
            gd32f4xx_i2c.c \
            gd32f4xx_ipa.c \
            gd32f4xx_misc.c\
            gd32f4xx_pmu.c \
            gd32f4xx_rcu.c \
            gd32f4xx_rtc.c \
            gd32f4xx_sdio.c \
            gd32f4xx_spi.c \
            gd32f4xx_syscfg.c \
            gd32f4xx_timer.c \
            gd32f4xx_tli.c \
            gd32f4xx_trng.c \
            gd32f4xx_usart.c \
            gd32f4xx_wwdgt.c

VPATH       := $(VPATH):$(STDPERIPH_DIR)/Source

DEVICE_FLAGS = -DGD32F4XX

ifneq ($(TARGET_MCU),$(filter $(TARGET_MCU),GDF450 GD32F470))
STDPERIPH_SRC += gd32f4xx_exmc.c
endif


USBDEVICE_DIR = $(USB_LIB_DIR)/device

USBDCORE_DIR = $(USBDEVICE_DIR)/core
USBCORE_SRC  = \
                 $(USBDCORE_DIR)/Source/usbd_core.c \
                 $(USBDCORE_DIR)/Source/usbd_enum.c \
                 $(USBDCORE_DIR)/Source/usbd_transc.c

USBCDC_DIR   = $(USBDEVICE_DIR)/class/cdc
USBCDC_SRC   = \
               $(USBCDC_DIR)/Source/cdc_acm_core.c

USBMSC_DIR   = $(USBDEVICE_DIR)/class/msc
USBMSC_SRC   = \
               $(USBMSC_DIR)/Source/usbd_msc_bbb.c \
               $(USBMSC_DIR)/Source/usbd_msc_core.c \
               $(USBMSC_DIR)/Source/usbd_msc_scsi.c

USBHID_DIR   = $(USBDEVICE_DIR)/class/hid
USBHID_SRC   = \
               $(USBHID_DIR)/Source/standard_hid_core.c


USBDRV_DIR   = $(USB_LIB_DIR)/driver
USBDRV_SRC   = \
               $(USB_LIB_DIR)/driver/Source/drv_usb_core.c \
               $(USB_LIB_DIR)/driver/Source/drv_usb_dev.c \
               $(USB_LIB_DIR)/driver/Source/drv_usbd_int.c



USBSTD_DIR   = $(USB_LIB_DIR)/ustd


DEVICE_STDPERIPH_SRC := \
            $(STDPERIPH_SRC) \
            $(USBOTG_SRC) \
            $(USBCORE_SRC) \
            $(USBCDC_SRC) \
            $(USBHID_SRC) \
            $(USBMSC_SRC) \
            $(USBDRV_SRC)


VPATH        := $(VPATH):$(CMSIS_DIR)/Core/Include:$(LIB_MAIN_DIR)/GD32F4/Drivers/CMSIS/GD/GD32F4xx/Source


CMSIS_SRC    := \
            gd32f4xx_gpio.c \
            gd32f4xx_rcu.c

INCLUDE_DIRS    := \
            $(INCLUDE_DIRS) \
            $(TARGET_PLATFORM_DIR) \
            $(TARGET_PLATFORM_DIR)/include \
            $(TARGET_PLATFORM_DIR)/startup \
            $(STDPERIPH_DIR)/Include \
            $(USBDEVICE_DIR)/class/msc/Include \
            $(USBHID_DIR)/Include \
            $(USBCDC_DIR)/Include \
            $(USBMSC_DIR)/Include \
            $(USBDCORE_DIR)/Include \
            $(USBDRV_DIR)/Include \
            $(USBSTD_DIR)/common \
            $(USBSTD_DIR)/class/cdc \
            $(USBSTD_DIR)/class/msc \
            $(USBSTD_DIR)/class/hid \
            $(CMSIS_DIR)/Core/Include \
            $(LIB_MAIN_DIR)/GD32F4/Drivers/CMSIS/GD/GD32F4xx/Include \
            $(TARGET_PLATFORM_DIR)/usb_f4

DEVICE_FLAGS += -DUSE_STDPERIPH_DRIVER


#Flags
ARCH_FLAGS      = -mthumb -mcpu=cortex-m4 -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16

ifeq ($(TARGET_MCU),GD32F407xx)
DEVICE_FLAGS    += -DGD32F407
LD_SCRIPT       = $(LINKER_DIR)/gd32_flash_f407_425.ld
STARTUP_SRC     = GD32/startup/startup_gd32f407_427.s
MCU_FLASH_SIZE  := 1024

else ifeq ($(TARGET_MCU),GD32F425xx)
DEVICE_FLAGS    += -DGD32F425
LD_SCRIPT       = $(LINKER_DIR)/gd32f405_425xg_flash.ld
STARTUP_SRC     = GD32/startup/startup_gd32f405_425.S
MCU_FLASH_SIZE  := 1024

else ifeq ($(TARGET_MCU),GD32F460xg)
DEVICE_FLAGS    += -DGD32F460
LD_SCRIPT       = $(LINKER_DIR)/gd32f460xg_flash.ld
STARTUP_SRC     = GD32/startup/startup_gd32f460.S
MCU_FLASH_SIZE  := 1024

else
$(error Unknown MCU for F4 target)
endif
DEVICE_FLAGS    += -DHXTAL_VALUE=$(HSE_VALUE)

DSP_LIB := $(LIB_MAIN_DIR)/CMSIS/DSP
DEVICE_FLAGS += -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -D__FPU_PRESENT=1 -DUNALIGNED_SUPPORT_DISABLE -DARM_MATH_CM4
DEVICE_FLAGS += -DUSE_GDBSP_DRIVER -DUSE_USB_FS
DEVICE_FLAGS += -DVECT_TAB_SRAM

MCU_COMMON_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/dshot_bitbang_decode.c \
            drivers/inverter.c \
            GD32/adc_gd32.c \
            GD32/audio_gd32.c \
            GD32/bus_i2c_gd32.c \
            GD32/bus_spi_gd32.c \
            GD32/camera_control_gd32.c \
            GD32/debug.c \
            GD32/dma_gd32.c \
            GD32/dma_reqmap_mcu.c \
            GD32/dshot_bitbang.c \
            GD32/dshot_bitbang_stdperiph.c \
            GD32/exti_gd32.c \
            GD32/io_gd32.c \
            GD32/light_ws2811strip_stdperiph.c \
            GD32/persistent_gd32.c \
            GD32/rcu_gd32.c \
            GD32/sdio_gdf4xx.c \
            GD32/serial_uart_stdperiph.c \
            GD32/serial_uart_gd32f4xx.c \
            GD32/system_gd32f4xx.c \
            GD32/pwm_output_gd32.c \
            GD32/pwm_output_dshot.c \
            GD32/timer_stdperiph.c \
            GD32/timer_gd32f4xx.c \
            GD32/transponder_ir_io_stdperiph.c \
            drivers/adc.c \
            drivers/bus_spi_config.c \
            drivers/serial_escserial.c \
            drivers/serial_pinconfig.c \
            GD32/usb_f4/usbd_msc_desc.c \
            GD32/startup/system_gd32f4xx.c

VCP_SRC = \
            GD32/usb_f4/gd32f4xx_it.c \
            GD32/usb_f4/usb_bsp.c \
            GD32/usb_f4/usbd_desc.c \
            GD32/usb_f4/usb_cdc_hid.c \
            GD32/usbd_cdc_vcp.c \
            GD32/serial_usb_vcp.c \
            drivers/usb_io.c

MSC_SRC = \
            drivers/usb_msc_common.c \
            GD32/usb_msc_f4xx.c \
            msc/usbd_storage.c \
            msc/usbd_storage_emfat.c \
            msc/emfat.c \
            msc/emfat_file.c \
            msc/usbd_storage_sd_spi.c \
            msc/usbd_storage_sdio.c


INCLUDE_DIRS += $(PLATFORM_DIR)/common/stm32

MCU_COMMON_SRC += \
            common/stm32/bus_i2c_pinconfig.c \
            common/stm32/bus_spi_hw.c \
            common/stm32/bus_spi_pinconfig.c \
            common/stm32/config_flash.c \
            common/stm32/dshot_bitbang_shared.c \
            common/stm32/dshot_dpwm.c \
            common/stm32/mco.c \
            common/stm32/io_impl.c \
            common/stm32/pwm_output_beeper.c \
            common/stm32/pwm_output_dshot_shared.c \
            common/stm32/serial_uart_hw.c \
            common/stm32/serial_uart_pinconfig.c \
            common/stm32/system.c

SPEED_OPTIMISED_SRC += \
            common/stm32/system.c \
            common/stm32/bus_spi_hw.c \
            common/stm32/pwm_output_dshot_shared.c \
            common/stm32/dshot_bitbang_shared.c \
            common/stm32/io_impl.c

SIZE_OPTIMISED_SRC += \
            GD32/serial_usb_vcp.c \
            drivers/inverter.c \
            drivers/serial_escserial.c \
            drivers/serial_pinconfig.c \
            drivers/bus_spi_config.c \
            common/stm32/bus_i2c_pinconfig.c \
            common/stm32/bus_spi_pinconfig.c \
            common/stm32/config_flash.c \
            common/stm32/pwm_output_beeper.c \
            common/stm32/bus_spi_pinconfig.c
