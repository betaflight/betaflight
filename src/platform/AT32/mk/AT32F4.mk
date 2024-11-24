#
# AT32F4 Make file include
#

CMSIS_DIR      := $(ROOT)/lib/main/AT32F43x/cmsis
STDPERIPH_DIR   = $(ROOT)/lib/main/AT32F43x/drivers
MIDDLEWARES_DIR = $(ROOT)/lib/main/AT32F43x/middlewares
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

STARTUP_SRC     = startup/startup_at32f435_437.s

VPATH           := $(VPATH):$(ROOT)/lib/main/AT32F43x/cmsis/cm4/core_support:$(STDPERIPH_DIR)/src:$(MIDDLEWARES_DIR):$(SRC_DIR)/startup/at32

VCP_SRC =  \
            usbd_class/cdc/cdc_class.c \
            usbd_class/cdc/cdc_desc.c \
            drivers/usb_io.c

VCP_INCLUDES = \
        $(MIDDLEWARES_DIR)/usb_drivers/inc \
        $(MIDDLEWARES_DIR)/usbd_class/cdc

DEVICE_STDPERIPH_SRC = $(STDPERIPH_SRC)

INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(TARGET_PLATFORM_DIR)/startup \
                   $(TARGET_PLATFORM_DIR) \
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

ARCH_FLAGS      = -std=c99  -mthumb -mcpu=cortex-m4 -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -Wdouble-promotion
DEVICE_FLAGS   += -DUSE_ATBSP_DRIVER -DAT32F43x -DHSE_VALUE=$(HSE_VALUE) -DAT32 -DUSE_OTG_HOST_MODE

MCU_COMMON_SRC = \
            stm32/system.c \
            startup/at32f435_437_clock.c \
            startup/system_at32f435_437.c \
            adc_at32f43x.c \
            bus_i2c_atbsp.c \
            bus_i2c_atbsp_init.c \
            bus_spi_at32bsp.c \
            camera_control_at32.c \
            debug.c \
            dma_at32f43x.c \
            dma_reqmap_mcu.c \
            dshot_bitbang.c \
            dshot_bitbang_stdperiph.c \
            exti_at32.c \
            io_at32.c \
            light_ws2811strip_at32f43x.c \
            persistent_at32bsp.c \
            pwm_output_at32bsp.c \
            pwm_output_dshot.c \
            rcc_at32.c \
            serial_uart_at32bsp.c \
            serial_uart_at32f43x.c \
            serial_usb_vcp_at32f4.c \
            system_at32f43x.c \
            timer_at32bsp.c \
            timer_at32f43x.c \
            usb_msc_at32f43x.c \
            drivers/accgyro/accgyro_mpu.c \
            drivers/dshot_bitbang_decode.c \
            drivers/inverter.c \
            drivers/pwm_output_dshot_shared.c \
            $(MIDDLEWARES_DIR)/i2c_application_library/i2c_application.c \
            drivers/bus_i2c_timing.c \
            drivers/usb_msc_common.c \
            msc/usbd_storage.c \
            msc/usbd_storage_emfat.c \
            msc/emfat.c \
            msc/emfat_file.c

SPEED_OPTIMISED_SRC := $(SPEED_OPTIMISED_SRC) \
            stm32/system.c

MCU_EXCLUDES =
