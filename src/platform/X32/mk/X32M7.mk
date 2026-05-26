#
# X32M7 Make file include
#

# Auto-hydrate X32M7 submodule when building X32 targets
PLATFORM_SDK := x32m7
PLATFORM_SDK_STAMP := $(X32M7_SDK_STAMP)

#CMSIS
CMSIS_DIR      := $(LIB_MODULES_DIR)/X32M7/Drivers/CMSIS

#STDPERIPH
STDPERIPH_DIR   = $(LIB_MODULES_DIR)/X32M7/Drivers/X32M7xx_StdPeriph_Driver
STDPERIPH_SRC   = \
            misc.c \
            sdmmc_host.c \
            x32m7xx_adc.c \
            x32m7xx_comp.c \
            x32m7xx_cordic.c \
            x32m7xx_crc.c \
            x32m7xx_dac.c \
            x32m7xx_dbg.c \
            x32m7xx_dcmu.c \
            x32m7xx_dma.c \
            x32m7xx_dmamux.c \
            x32m7xx_dsmu.c \
            x32m7xx_dvp.c \
            x32m7xx_eccmon.c \
            x32m7xx_exti.c \
            x32m7xx_fdcan.c \
            x32m7xx_femc.c \
            x32m7xx_fmac.c \
            x32m7xx_gpio.c \
            x32m7xx_i2c.c \
            x32m7xx_i2s.c \
            x32m7xx_iwdg.c \
            x32m7xx_jpeg.c \
            x32m7xx_lptim.c \
            x32m7xx_lpuart.c \
            x32m7xx_mdma.c \
            x32m7xx_mmu.c \
            x32m7xx_otpc.c \
            x32m7xx_pwr.c \
            x32m7xx_rcc.c \
            x32m7xx_rtc.c \
            x32m7xx_sdram.c \
            x32m7xx_sdmmc.c \
            x32m7xx_shrtim.c \
            x32m7xx_smu.c \
            x32m7xx_spi.c \
            x32m7xx_tim.c \
            x32m7xx_usart.c \
            x32m7xx_vrefbuf.c \
            x32m7xx_wwdg.c

#USB

USBHSDRIVER_DIR = X32M7/Middlewares/X32M7xx_usbhs_driver/driver
USBHSDRIVER_SRC          = \
            $(USBHSDRIVER_DIR)/src/usbhs_core.c \
            $(USBHSDRIVER_DIR)/src/usbhs_dcd.c \
            $(USBHSDRIVER_DIR)/src/usbhs_dcd_int.c

USBHSCORE_DIR = X32M7/Middlewares/X32M7xx_usbhs_driver/device/core
USBHSCORE_SRC = \
            $(USBHSCORE_DIR)/src/usbd_core.c \
            $(USBHSCORE_DIR)/src/usbd_ioreq.c \
            $(USBHSCORE_DIR)/src/usbd_req.c

USBHSCDC_DIR = X32M7/Middlewares/X32M7xx_usbhs_driver/device/class/cdc
USBHSCDC_SRC = \
            $(USBHSCDC_DIR)/src/usbd_cdc_core.c

USBHSMSC_DIR = X32M7/Middlewares/X32M7xx_usbhs_driver/device/class/msc
USBHSMSC_SRC = \
            $(USBHSMSC_DIR)/src/usbd_msc_bot.c \
            $(USBHSMSC_DIR)/src/usbd_msc_core.c \
            $(USBHSMSC_DIR)/src/usbd_msc_data.c \
            $(USBHSMSC_DIR)/src/usbd_msc_scsi.c

DEVICE_STDPERIPH_SRC := $(STDPERIPH_SRC) \
                        $(USBHSDRIVER_SRC) \
                        $(USBHSCORE_SRC) \
                        $(USBHSCDC_SRC) \
                        $(USBHSMSC_SRC)

#CMSIS
VPATH           := $(VPATH):$(LIB_MODULES_DIR)/Core/Include:$(CMSIS_DIR)/Device/X32M7xx:$(STDPERIPH_DIR)/src
CMSIS_SRC       :=
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(TARGET_PLATFORM_DIR) \
                   $(TARGET_PLATFORM_DIR)/include \
                   $(TARGET_PLATFORM_DIR)/startup \
                   $(TARGET_PLATFORM_DIR)/usbhs \
                   $(TARGET_PLATFORM_DIR)/usbhs/vcp \
                   $(TARGET_PLATFORM_DIR)/usbhs/msc \
                   $(PLATFORM_DIR)/common/stm32 \
                   $(LIB_MAIN_DIR)/CMSIS/Core/Include \
                   $(CMSIS_DIR)/Device/X32M7xx/Include \
                   $(STDPERIPH_DIR)/inc \
                   $(LIB_MODULES_DIR)/$(USBHSDRIVER_DIR)/inc \
                   $(LIB_MODULES_DIR)/$(USBHSCORE_DIR)/inc \
                   $(LIB_MODULES_DIR)/$(USBHSCDC_DIR)/inc \
                   $(LIB_MODULES_DIR)/$(USBHSMSC_DIR)/inc

#Flags
ARCH_FLAGS      = -mthumb -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-sp-d16

DEVICE_FLAGS    = -DUSE_STDPERIPH_DRIVER -DSYSCLK_SRC=SYSCLK_USE_HSE_PLL -DUSE_USB_HS_IN_FS -DX32M7XX

ifeq ($(TARGET_MCU),X32M7B)
DEVICE_FLAGS    += -DX32M7B -DTCM_SIZE_VALUE=0x03
LD_SCRIPT       = $(LINKER_DIR)/x32_flash_m7b_2m.ld
STARTUP_SRC     = X32/startup/startup_x32m7b.s
MCU_FLASH_SIZE  := 2048

else
$(error TARGET_MCU [$(TARGET_MCU)] is not supported for X32M7xx)
endif

DEVICE_FLAGS    += -DHSE_VALUE=$(HSE_VALUE)

VCP_SRC = \
            X32/serial_usb_vcp.c \
            X32/usbhs/usbd_desc.c \
            X32/usbhs/usbd_user.c \
            X32/usbhs/usbhs_bsp.c \
            X32/usbhs/x32m7xx_it.c \
            X32/usbhs/vcp/usbd_cdc_vcp.c \
            drivers/usb_io.c

MCU_COMMON_SRC = \
            X32/startup/soc.c \
            common/stm32/system.c \
            common/stm32/io_impl.c \
            common/stm32/adc_impl.c \
            common/stm32/bus_i2c_pinconfig.c \
            common/stm32/config_flash.c \
            common/stm32/debug_pin.c \
            common/stm32/expresslrs_driver_hw.c \
            common/stm32/fault_handlers.c \
            common/stm32/ledstrip_ws2811_stm32.c \
            common/stm32/pwm_output_beeper.c \
            common/stm32/pwm_output_dshot_shared.c \
            common/stm32/dshot_dpwm.c \
            common/stm32/dshot_bitbang_shared.c \
            common/stm32/serial_uart_hw.c \
            common/stm32/serial_uart_pinconfig.c \
            common/stm32/bus_spi_pinconfig.c \
            drivers/adc.c \
            drivers/bus_i2c_timing.c \
            drivers/bus_spi_config.c \
            drivers/dshot_bitbang_decode.c \
            drivers/serial_escserial.c \
            drivers/serial_pinconfig.c \
            X32/adc_x32.c \
            X32/bus_i2c_x32.c \
            X32/bus_spi_x32.c \
            X32/debug.c \
            X32/dma_reqmap_mcu.c \
            X32/dma_x32.c \
            X32/dshot_bitbang.c \
            X32/dshot_bitbang_stdperiph.c \
            X32/exti_x32.c \
            X32/io_x32.c \
            X32/light_ws2811strip_x32.c \
            X32/persistent_x32.c \
            X32/pwm_output_x32.c \
            X32/pwm_output_dshot_x32.c \
            X32/rcc_x32.c \
            X32/serial_uart_stdperiph.c \
            X32/serial_uart_x32m7xx.c \
            X32/sdio_x32.c \
            X32/system_x32m7.c \
            X32/timer_x32bsp.c \
            X32/timer_x32.c \
            X32/startup/system_x32m7xx.c

MSC_SRC = \
            X32/usb_msc_hs.c \
            X32/usbhs/msc/usbd_msc_desc.c \
            drivers/usb_msc_common.c \
            msc/usbd_storage.c \
            msc/usbd_storage_emfat.c \
            msc/emfat.c \
            msc/emfat_file.c \
            msc/usbd_storage_sdio.c \
            msc/usbd_storage_sd_spi.c \
            common/stm32/msc_sdio_storage.c

DSP_LIB := $(LIB_MAIN_DIR)/CMSIS/DSP
DEVICE_FLAGS += -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -D__FPU_PRESENT=1 -DUNALIGNED_SUPPORT_DISABLE -DARM_MATH_CM7
DEVICE_FLAGS += -D__ICACHE_PRESENT=1 -D__DCACHE_PRESENT=1
DEVICE_FLAGS += -DUSING_TCM
DEVICE_FLAGS += -DCORE_CM7
