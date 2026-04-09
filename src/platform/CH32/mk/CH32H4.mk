#
# CH32H4 Make file include
#
CMSIS_DIR       := $(LIB_MAIN_DIR)/CH32H41x/Cmsis
STDPERIPH_DIR   = $(LIB_MAIN_DIR)/CH32H41x/Peripheral
MIDDLEWARES_DIR = $(LIB_MAIN_DIR)/CH32H41x/middlewares
STDPERIPH_SRC   = \
        ch32h417_adc.c \
        ch32h417_can.c \
        ch32h417_crc.c \
        ch32h417_dac.c \
        ch32h417_dbgmcu.c \
        ch32h417_dfsdm.c \
        ch32h417_dma.c \
        ch32h417_dvp.c \
        ch32h417_ecdc.c \
        ch32h417_eth.c \
        ch32h417_exti.c \
        ch32h417_flash.c \
        ch32h417_fmc.c \
        ch32h417_gpha.c \
        ch32h417_gpio.c \
        ch32h417_hsadc.c \
        ch32h417_hsem.c \
        ch32h417_i2c.c \
        ch32h417_i3c.c \
        ch32h417_ipc.c \
        ch32h417_iwdg.c \
        ch32h417_lptim.c \
        ch32h417_ltdc.c \
        ch32h417_opa.c \
        ch32h417_pwr.c  \
        ch32h417_qspi.c \
        ch32h417_rcc.c \
        ch32h417_rng.c \
        ch32h417_rtc.c \
        ch32h417_sai.c \
        ch32h417_sdio.c \
        ch32h417_sdmmc.c \
        ch32h417_spi.c \
        ch32h417_swpmi.c \
        ch32h417_tim.c \
        ch32h417_usart.c \
        ch32h417_wwdg.c

STARTUP_SRC     = CH32/startup/startup_ch32h417_v5f.S

VPATH           := $(VPATH):$(LIB_MAIN_DIR)/CH32H41x/Cmsis/Core:$(LIB_MAIN_DIR)/CH32H41x/Cmsis/Debug:$(STDPERIPH_DIR)/src:$(MIDDLEWARES_DIR)

VCP_SRC =  \
           ch32h41x_hs/usb_ch32h41x_usbhs_reg.c \
           class/cdc/usbd_cdc_acm.c \
           core/usbd_core.c \
           board/cdc_vcp_ch32h41x.c

VCP_INCLUDES = \
        $(MIDDLEWARES_DIR)/ch32h41x_hs \
        $(MIDDLEWARES_DIR)/class/cdc \
        $(MIDDLEWARES_DIR)/common \
        $(MIDDLEWARES_DIR)/board \
        $(MIDDLEWARES_DIR)/core

DEVICE_STDPERIPH_SRC = $(STDPERIPH_SRC)

INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(TARGET_PLATFORM_DIR) \
                   $(TARGET_PLATFORM_DIR)/include \
                   $(TARGET_PLATFORM_DIR)/startup \
                   $(PLATFORM_DIR)/common/stm32 \
                   $(STDPERIPH_DIR)/inc \
                   $(CMSIS_DIR)/Core \
                   $(CMSIS_DIR)/Debug \
                   $(MIDDLEWARES_DIR)/class/msc \
                   $(VCP_INCLUDES)


LD_SCRIPT       = $(LINKER_DIR)/ch32h41x_v5f.ld

############################################################################
ARCH_FLAGS      = -std=c99 -march=rv32imafc_zba_zbb_zbc_zbs_xw -mabi=ilp32f -msmall-data-limit=8 -msave-restore -fmessage-length=0 -fmax-errors=5 -fsigned-char -fsingle-precision-constant -Wunused -Wuninitialized -lprintfloat -g
DEVICE_FLAGS    += -DUSE_CHBSP_DRIVER -DCH32H415 -DCH32H41x -DHSE_VALUE=$(HSE_VALUE) -DCH32 -DUSE_OTG_HOST_MODE

MCU_COMMON_SRC = \
        common/stm32/system.c \
        common/stm32/io_impl.c \
        common/stm32/config_flash.c \
        common/stm32/mco.c \
        CH32/startup/ch32h417_it.c \
        CH32/startup/system_ch32h417.c \
        CH32/adc_ch32h41x.c \
        CH32/bus_i2c_ch32h41x.c \
        CH32/bus_spi_ch32h41x.c \
        CH32/camera_control_ch32.c \
        CH32/debug.c \
        CH32/dma_ch32h41x.c \
        CH32/dma_reqmap_mcu.c \
        CH32/dshot_bitbang.c \
        CH32/dshot_bitbang_stdperiph.c \
        CH32/exti_ch32.c \
        CH32/io_ch32.c \
        CH32/light_ws2811strip_ch32h41x.c \
        CH32/persistent_ch32h41x.c \
        CH32/pwm_output_ch32.c \
        CH32/pwm_output_dshot.c \
        CH32/rcc_ch32.c \
        CH32/serial_uart_ch32bsp.c \
        CH32/serial_uart_ch32h41x.c \
        CH32/serial_usb_vcp_ch32h4.c \
        CH32/system_ch32h41x.c \
        CH32/timer_ch32bsp.c \
        CH32/timer_ch32h41x.c \
        CH32/usb_msc_ch32h41x.c \
        CH32/usb_msc_sdcard_spi_ch32h41x.c \
        drivers/accgyro/accgyro_mpu.c \
        drivers/dshot_bitbang_decode.c \
        drivers/inverter.c \
        common/stm32/pwm_output_beeper.c \
        common/stm32/pwm_output_dshot_shared.c \
        common/stm32/dshot_dpwm.c \
        common/stm32/dshot_bitbang_shared.c \
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
        drivers/usb_io.c \
        class/msc/usbd_msc.c \
        msc/usbd_storage.c \
        msc/usbd_storage_emfat.c \
        msc/emfat.c \
        msc/emfat_file.c \
        common/stm32/ledstrip_ws2811_stm32.c \
        common/stm32/debug_pin.c \
        common/stm32/adc_impl.c
        
#         \
#         msc/usbd_storage_sd_spi.c

SPEED_OPTIMISED_SRC += \
            common/stm32/system.c \
            CH32/exti_ch32.c \
            common/stm32/bus_spi_hw.c \
            common/stm32/pwm_output_dshot_shared.c \
            common/stm32/dshot_bitbang_shared.c \
            common/stm32/io_impl.c


SIZE_OPTIMISED_SRC += \
            drivers/bus_i2c_timing.c \
            drivers/inverter.c \
            drivers/bus_spi_config.c \
            common/stm32/bus_i2c_pinconfig.c \
            common/stm32/bus_spi_pinconfig.c \
            common/stm32/pwm_output_beeper.c \
            common/stm32/serial_uart_pinconfig.c \
            drivers/serial_escserial.c \
            drivers/serial_pinconfig.c
