#
# AT32F4 Make file include
#

CMSIS_DIR      := $(LIB_MAIN_DIR)/AT32F43x/cmsis
STDPERIPH_DIR   = $(LIB_MAIN_DIR)/AT32F43x/drivers
MIDDLEWARES_DIR = $(LIB_MAIN_DIR)/AT32F43x/middlewares
STDPERIPH_SRC   = \
        at32f435_437_acc.c \
        at32f435_437_adc.c \
        at32f435_437_can.c \
        at32f435_437_crc.c \
        at32f435_437_crm.c \
        at32f435_437_dac.c \
        at32f435_437_debug.c \
        at32f435_437_dma.c \
        at32f435_437_dvp.c \
        at32f435_437_edma.c \
        at32f435_437_emac.c \
        at32f435_437_ertc.c \
        at32f435_437_exint.c \
        at32f435_437_flash.c \
        at32f435_437_gpio.c \
        at32f435_437_i2c.c \
        at32f435_437_misc.c \
        at32f435_437_pwc.c \
        at32f435_437_qspi.c \
        at32f435_437_scfg.c \
        at32f435_437_sdio.c \
        at32f435_437_spi.c \
        at32f435_437_tmr.c \
        at32f435_437_usart.c \
        at32f435_437_usb.c \
        at32f435_437_wdt.c \
        at32f435_437_wwdt.c \
        at32f435_437_xmc.c \
        usb_drivers/src/usb_core.c \
        usb_drivers/src/usbd_core.c \
        usb_drivers/src/usbd_int.c \
        usb_drivers/src/usbd_sdr.c \
        usb_drivers/src/usbh_core.c \
        usb_drivers/src/usbh_ctrl.c \
        usb_drivers/src/usbh_int.c \
        usbd_class/msc/msc_bot_scsi.c \
        usbd_class/msc/msc_class.c \
        usbd_class/msc/msc_desc.c

STARTUP_SRC     = AT32/startup/startup_at32f435_437.s

VPATH           := $(VPATH):$(LIB_MAIN_DIR)/AT32F43x/cmsis/cm4/core_support:$(STDPERIPH_DIR)/src:$(MIDDLEWARES_DIR)

VCP_SRC =  \
            usbd_class/cdc/cdc_class.c \
            usbd_class/cdc/cdc_desc.c \
            drivers/usb_io.c

VCP_INCLUDES = \
        $(MIDDLEWARES_DIR)/usb_drivers/inc \
        $(MIDDLEWARES_DIR)/usbd_class/cdc

DEVICE_STDPERIPH_SRC = $(STDPERIPH_SRC)

INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(TARGET_PLATFORM_DIR) \
                   $(TARGET_PLATFORM_DIR)/include \
                   $(TARGET_PLATFORM_DIR)/startup \
                   $(PLATFORM_DIR)/common/stm32 \
                   $(STDPERIPH_DIR)/inc \
                   $(CMSIS_DIR)/cm4/core_support \
                   $(CMSIS_DIR)/cm4 \
                   $(MIDDLEWARES_DIR)/i2c_application_library \
                   $(MIDDLEWARES_DIR)/usbd_class/msc \
                   $(VCP_INCLUDES)

ifeq ($(TARGET),AT32F435M)
LD_SCRIPT       = $(LINKER_DIR)/at32_flash_f43xm.ld
else
LD_SCRIPT       = $(LINKER_DIR)/at32_flash_f43xg.ld
endif

ARCH_FLAGS      = -std=c99 -mthumb -mcpu=cortex-m4 -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16
DEVICE_FLAGS   += -DUSE_ATBSP_DRIVER -DAT32F43x -DHSE_VALUE=$(HSE_VALUE) -DAT32 -DUSE_OTG_HOST_MODE

MCU_COMMON_SRC = \
            common/stm32/system.c \
            common/stm32/io_impl.c \
            common/stm32/config_flash.c \
            common/stm32/mco.c \
            AT32/startup/at32f435_437_clock.c \
            AT32/startup/system_at32f435_437.c \
            AT32/adc_at32f43x.c \
            AT32/bus_i2c_atbsp.c \
            AT32/bus_i2c_atbsp_init.c \
            AT32/bus_spi_at32bsp.c \
            AT32/camera_control_at32.c \
            AT32/debug.c \
            AT32/dma_at32f43x.c \
            AT32/dma_reqmap_mcu.c \
            AT32/dshot_bitbang.c \
            AT32/dshot_bitbang_stdperiph.c \
            AT32/exti_at32.c \
            AT32/io_at32.c \
            AT32/light_ws2811strip_at32f43x.c \
            AT32/persistent_at32bsp.c \
            AT32/pwm_output_at32bsp.c \
            AT32/pwm_output_dshot.c \
            AT32/rcc_at32.c \
            AT32/serial_uart_at32bsp.c \
            AT32/serial_uart_at32f43x.c \
            AT32/serial_usb_vcp_at32f4.c \
            AT32/system_at32f43x.c \
            AT32/timer_at32bsp.c \
            AT32/timer_at32f43x.c \
            AT32/usb_msc_at32f43x.c \
            drivers/accgyro/accgyro_mpu.c \
            drivers/dshot_bitbang_decode.c \
            drivers/inverter.c \
            common/stm32/pwm_output_dshot_shared.c \
            common/stm32/dshot_dpwm.c \
            common/stm32/dshot_bitbang_shared.c \
            $(MIDDLEWARES_DIR)/i2c_application_library/i2c_application.c \
            drivers/bus_i2c_timing.c \
            drivers/usb_msc_common.c \
            drivers/adc.c \
            drivers/bus_spi_config.c \
            common/stm32/bus_i2c_pinconfig.c \
            common/stm32/bus_spi_pinconfig.c \
            common/stm32/bus_spi_hw.c \
            common/stm32/serial_uart_hw.c \
            common/stm32/serial_uart_pinconfig.c \
            drivers/serial_escserial.c \
            drivers/serial_pinconfig.c \
            msc/usbd_storage.c \
            msc/usbd_storage_emfat.c \
            msc/emfat.c \
            msc/emfat_file.c \
            msc/usbd_storage_sd_spi.c

SPEED_OPTIMISED_SRC += \
            common/stm32/dshot_bitbang_shared.c \
            common/stm32/pwm_output_dshot_shared.c \
            common/stm32/bus_spi_hw.c \
            common/stm32/system.c

SIZE_OPTIMISED_SRC += \
            drivers/bus_i2c_timing.c \
            drivers/inverter.c \
            drivers/bus_spi_config.c \
            common/stm32/bus_i2c_pinconfig.c \
            common/stm32/bus_spi_pinconfig.c \
            common/stm32/serial_uart_pinconfig.c \
            drivers/serial_escserial.c \
            drivers/serial_pinconfig.c
