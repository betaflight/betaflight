#
# X32M7 Make file include
#

# Auto-hydrate X32M7 submodule when building X32 targets
PLATFORM_SDK := x32m7
PLATFORM_SDK_STAMP := $(X32M7_SDK_STAMP)

#CMSIS
CMSIS_DIR      := $(LIB_MAIN_DIR)/X32M7/Drivers/CMSIS

#STDPERIPH
STDPERIPH_DIR   = $(LIB_MAIN_DIR)/X32M7/Drivers/X32M7xx_StdPeriph_Driver
STDPERIPH_SRC   = \
            misc.c \
            x32m7xx_dma.c \
            x32m7xx_dmamux.c \
            x32m7xx_exti.c \
            x32m7xx_gpio.c \
            x32m7xx_pwr.c \
            x32m7xx_rcc.c \
            x32m7xx_rtc.c \
            x32m7xx_spi.c \
            x32m7xx_tim.c \
            x32m7xx_adc.c \
            x32m7xx_usart.c \
            x32m7xx_dac.c \
            x32m7xx_sdmmc.c \
            sdmmc_host.c \
            x32m7xx_i2c.c \
            x32m7xx_smu.c 

#USB

USBHSDRIVER_DIR = X32M7/Middlewares/X32M7xx_usbhs_driver/driver
USBHSDRIVER_SRC          = \
            $(USBHSDRIVER_DIR)/src/usbhs_core.c \
            $(USBHSDRIVER_DIR)/src/usbhs_dcd.c \
            $(USBHSDRIVER_DIR)/src/usbhs_dcd_int.c

USBCORE_DIR = X32M7/Middlewares/X32M7xx_usbhs_driver/device/core
USBCORE_SRC = \
            $(USBCORE_DIR)/src/usbd_core.c \
            $(USBCORE_DIR)/src/usbd_ioreq.c \
            $(USBCORE_DIR)/src/usbd_req.c

USBCDC_DIR = X32M7/Middlewares/X32M7xx_usbhs_driver/device/class/cdc
USBCDC_SRC = \
            $(USBCDC_DIR)/src/usbd_cdc_core.c

USBMSC_DIR = X32M7/Middlewares/X32M7xx_usbhs_driver/device/class/msc
USBMSC_SRC = \
            $(USBMSC_DIR)/src/usbd_msc_bot.c  \
            $(USBMSC_DIR)/src/usbd_msc_core.c \
            $(USBMSC_DIR)/src/usbd_msc_data.c \
            $(USBMSC_DIR)/src/usbd_msc_scsi.c

DEVICE_STDPERIPH_SRC := $(STDPERIPH_SRC) \
                        $(USBHSDRIVER_SRC) \
                        $(USBCORE_SRC) \
                        $(USBCDC_SRC) \
                        $(USBMSC_SRC)

#CMSIS
VPATH           := $(VPATH):$(LIB_MAIN_DIR)/Core/Include:$(CMSIS_DIR)/Device/X32M7xx:$(STDPERIPH_DIR)/src
CMSIS_SRC       :=
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(TARGET_PLATFORM_DIR) \
                   $(TARGET_PLATFORM_DIR)/include \
                   $(TARGET_PLATFORM_DIR)/startup \
                   $(STDPERIPH_DIR)/inc \
                   $(LIB_MAIN_DIR)/CMSIS/Core/Include \
                   $(LIB_MAIN_DIR)/$(USBHSDRIVER_DIR)/inc \
                   $(LIB_MAIN_DIR)/$(USBCORE_DIR)/inc \
                   $(LIB_MAIN_DIR)/$(USBCDC_DIR)/inc \
                   $(LIB_MAIN_DIR)/$(USBMSC_DIR)/inc \
                   $(CMSIS_DIR)/Device/X32M7xx/Include \
                   $(PLATFORM_DIR)/common/stm32 \
                   $(TARGET_PLATFORM_DIR)/usbhs \
                   $(TARGET_PLATFORM_DIR)/usbhs/msc \
                   $(TARGET_PLATFORM_DIR)/usbhs/vcp

#Flags
ARCH_FLAGS      = -mthumb -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-sp-d16

DEVICE_FLAGS    = -DUSE_STDPERIPH_DRIVER -DUSE_X32STD_DRIVER -DSYSCLK_SRC=SYSCLK_USE_HSE_PLL -DUSE_USB_HS_IN_HS -DX32M7XX

ifeq ($(TARGET_MCU),X32M7BxI)
DEVICE_FLAGS    += -DX32M7B
LD_SCRIPT       = $(LINKER_DIR)/x32_flash_m7b_2m.ld
STARTUP_SRC     = X32/startup/startup_x32m7b.s
MCU_FLASH_SIZE  := 1920

else
$(error TARGET_MCU [$(TARGET_MCU)] is not supported)
endif

DEVICE_FLAGS    += -DHSE_VALUE=$(HSE_VALUE)

VCP_SRC = \
            X32/usbhs/vcp/usbd_cdc_vcp.c \
            X32/usbhs/x32m7xx_it.c \
            X32/usbhs/usbd_desc.c \
            X32/usbhs/usbd_user.c \
            X32/usbhs/usbhs_bsp.c \
            X32/serial_usb_vcp.c \
            drivers/usb_io.c

MCU_COMMON_SRC = \
            X32/exti_x32.c \
            X32/adc_x32.c \
            X32/audio_x32.c \
            X32/io_x32.c \
            X32/dma_x32.c \
            X32/dma_reqmap_mcu.c \
            X32/persistent_x32.c \
            X32/dshot_bitbang.c \
            X32/dshot_bitbang_stdperiph.c \
            X32/pwm_output_x32.c \
            X32/pwm_output_dshot_x32.c \
            X32/rcc_x32.c \
            X32/bus_i2c_x32.c \
            X32/bus_spi_x32.c \
            X32/bus_spi_pinconfig_x32.c \
            X32/serial_uart_stdperiph.c \
            X32/serial_uart_x32m7xx.c \
            X32/startup/system_x32m7xx.c \
            X32/startup/soc.c \
            X32/system_x32m7.c \
            X32/timer_x32.c \
            X32/minimal_stubs.c \
            X32/debug.c \
            common/stm32/serial_uart_hw.c \
            common/stm32/serial_uart_pinconfig.c \
            common/stm32/dshot_dpwm.c \
            common/stm32/pwm_output_dshot_shared.c \
			common/stm32/pwm_output_beeper.c \
            common/stm32/config_flash.c \
            drivers/bus_spi_config.c \
            drivers/serial_pinconfig.c \
            drivers/dshot_bitbang_decode.c \
	        drivers/adc.c \
            drivers/bus_i2c_timing.c \
            X32/sdio_x32.c \
            common/stm32/io_impl.c \
            common/stm32/adc_impl.c \
            common/stm32/bus_i2c_pinconfig.c \
            common/stm32/system.c 

MSC_SRC = \
            drivers/usb_msc_common.c \
            X32/usb_msc_hs.c \
            X32/usbhs/msc/usbd_msc_desc.c \
            msc/usbd_storage.c \
            msc/usbd_storage_emfat.c \
            msc/emfat.c \
            msc/emfat_file.c \
            msc/usbd_storage_sd_spi.c \
            msc/usbd_storage_sdio.c

DSP_LIB := $(LIB_MAIN_DIR)/CMSIS/DSP
DEVICE_FLAGS += -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -D__FPU_PRESENT=1 -DUNALIGNED_SUPPORT_DISABLE -DARM_MATH_CM7
DEVICE_FLAGS += -D__ICACHE_PRESENT=1 -D__DCACHE_PRESENT=1
DEVICE_FLAGS += -DUSE_X32BSP_DRIVER
DEVICE_FLAGS += -DUSING_TCM
DEVICE_FLAGS += -DCORE_CM7
