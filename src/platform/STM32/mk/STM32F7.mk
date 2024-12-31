#
# F7 Make file include
#

ifeq ($(DEBUG_HARDFAULTS),F7)
CFLAGS               += -DDEBUG_HARDFAULTS
endif

#CMSIS
CMSIS_DIR      := $(LIB_MAIN_DIR)/CMSIS

#STDPERIPH
STDPERIPH_DIR   = $(LIB_MAIN_DIR)/STM32F7/Drivers/STM32F7xx_HAL_Driver
STDPERIPH_SRC   = \
            stm32f7xx_hal_adc.c \
            stm32f7xx_hal_adc_ex.c \
            stm32f7xx_hal.c \
            stm32f7xx_hal_cortex.c \
            stm32f7xx_hal_dac.c \
            stm32f7xx_hal_dac_ex.c \
            stm32f7xx_hal_dma.c \
            stm32f7xx_hal_dma_ex.c \
            stm32f7xx_hal_exti.c \
            stm32f7xx_hal_flash.c \
            stm32f7xx_hal_flash_ex.c \
            stm32f7xx_hal_gpio.c \
            stm32f7xx_hal_i2c.c \
            stm32f7xx_hal_i2c_ex.c \
            stm32f7xx_hal_pcd.c \
            stm32f7xx_hal_pcd_ex.c \
            stm32f7xx_hal_pwr.c \
            stm32f7xx_hal_pwr_ex.c \
            stm32f7xx_hal_rcc.c \
            stm32f7xx_hal_rcc_ex.c \
            stm32f7xx_hal_rtc.c \
            stm32f7xx_hal_rtc_ex.c \
            stm32f7xx_hal_spi.c \
            stm32f7xx_hal_spi_ex.c \
            stm32f7xx_hal_tim.c \
            stm32f7xx_hal_tim_ex.c \
            stm32f7xx_hal_uart.c \
            stm32f7xx_hal_uart_ex.c \
            stm32f7xx_hal_usart.c \
            stm32f7xx_ll_dma2d.c \
            stm32f7xx_ll_dma.c \
            stm32f7xx_ll_gpio.c \
            stm32f7xx_ll_rcc.c \
            stm32f7xx_ll_spi.c \
            stm32f7xx_ll_tim.c \
            stm32f7xx_ll_usb.c \
            stm32f7xx_ll_utils.c

#USB
USBCORE_DIR = STM32F7/Middlewares/ST/STM32_USB_Device_Library/Core
USBCORE_SRC = \
            $(USBCORE_DIR)/Src/usbd_core.c \
            $(USBCORE_DIR)/Src/usbd_ctlreq.c \
            $(USBCORE_DIR)/Src/usbd_ioreq.c

USBCDC_DIR = STM32F7/Middlewares/ST/STM32_USB_Device_Library/Class/CDC
USBCDC_SRC = \
            $(USBCDC_DIR)/Src/usbd_cdc.c

USBHID_DIR = STM32F7/Middlewares/ST/STM32_USB_Device_Library/Class/HID
USBHID_SRC = \
            $(USBHID_DIR)/Src/usbd_hid.c

USBMSC_DIR = STM32F7/Middlewares/ST/STM32_USB_Device_Library/Class/MSC
USBMSC_SRC = \
            $(USBMSC_DIR)/Src/usbd_msc_bot.c \
            $(USBMSC_DIR)/Src/usbd_msc.c \
            $(USBMSC_DIR)/Src/usbd_msc_data.c \
            $(USBMSC_DIR)/Src/usbd_msc_scsi.c

DEVICE_STDPERIPH_SRC := \
            $(STDPERIPH_SRC) \
            $(USBCORE_SRC) \
            $(USBCDC_SRC) \
            $(USBHID_SRC) \
            $(USBMSC_SRC)

#CMSIS
VPATH           := $(VPATH):$(CMSIS_DIR)/Include:$(CMSIS_DIR)/Device/ST/STM32F7xx:$(STDPERIPH_DIR)/Src

CMSIS_SRC       :=
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(TARGET_PLATFORM_DIR) \
                   $(TARGET_PLATFORM_DIR)/startup \
                   $(STDPERIPH_DIR)/Inc \
                   $(LIB_MAIN_DIR)/$(USBCORE_DIR)/Inc \
                   $(LIB_MAIN_DIR)/$(USBCDC_DIR)/Inc \
                   $(LIB_MAIN_DIR)/$(USBHID_DIR)/Inc \
                   $(LIB_MAIN_DIR)/$(USBMSC_DIR)/Inc \
                   $(CMSIS_DIR)/Core/Include \
                   $(LIB_MAIN_DIR)/STM32F7/Drivers/CMSIS/Device/ST/STM32F7xx/Include \
                   $(TARGET_PLATFORM_DIR)/vcp_hal

#Flags
ARCH_FLAGS      = -mthumb -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-sp-d16 -fsingle-precision-constant

# Flags that are used in the STM32 libraries
DEVICE_FLAGS    = -DUSE_HAL_DRIVER -DUSE_FULL_LL_DRIVER

ifeq ($(TARGET_MCU),STM32F765xx)
DEVICE_FLAGS   += -DSTM32F765xx
LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_f765.ld
STARTUP_SRC     = STM32/startup/startup_stm32f765xx.s
MCU_FLASH_SIZE	:= 2048
else ifeq ($(TARGET_MCU),STM32F745xx)
DEVICE_FLAGS   += -DSTM32F745xx
LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_f74x.ld
STARTUP_SRC     = STM32/startup/startup_stm32f745xx.s
MCU_FLASH_SIZE  := 1024
else ifeq ($(TARGET_MCU),STM32F746xx)
DEVICE_FLAGS   += -DSTM32F746xx
LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_f74x.ld
STARTUP_SRC     = STM32/startup/startup_stm32f746xx.s
MCU_FLASH_SIZE  := 1024
else ifeq ($(TARGET_MCU),STM32F722xx)
DEVICE_FLAGS   += -DSTM32F722xx
ifndef LD_SCRIPT
LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_f722.ld
endif
STARTUP_SRC     = STM32/startup/startup_stm32f722xx.s
MCU_FLASH_SIZE  := 512
# Override the OPTIMISE_SPEED compiler setting to save flash space on these 512KB targets.
# Performance is only slightly affected but around 50 kB of flash are saved.
OPTIMISE_SPEED = -O2
else
$(error Unknown MCU for F7 target)
endif
DEVICE_FLAGS    += -DHSE_VALUE=$(HSE_VALUE) -DSTM32

VCP_SRC = \
            STM32/vcp_hal/usbd_desc.c \
            STM32/vcp_hal/usbd_conf_stm32f7xx.c \
            STM32/vcp_hal/usbd_cdc_hid.c \
            STM32/vcp_hal/usbd_cdc_interface.c \
            STM32/serial_usb_vcp.c \
            drivers/usb_io.c

MCU_COMMON_SRC = \
            common/stm32/system.c \
            drivers/accgyro/accgyro_mpu.c \
            drivers/bus_i2c_timing.c \
            drivers/dshot_bitbang_decode.c \
            drivers/pwm_output_dshot_shared.c \
            STM32/adc_stm32f7xx.c \
            STM32/audio_stm32f7xx.c \
            STM32/bus_i2c_hal_init.c \
            STM32/bus_i2c_hal.c \
            STM32/bus_spi_ll.c \
            STM32/debug.c \
            STM32/dma_reqmap_mcu.c \
            STM32/dma_stm32f7xx.c \
            STM32/dshot_bitbang_ll.c \
            STM32/dshot_bitbang.c \
            STM32/exti.c \
            STM32/io_stm32.c \
            STM32/light_ws2811strip_hal.c \
            STM32/persistent.c \
            STM32/pwm_output.c \
            STM32/pwm_output_dshot_hal.c \
            STM32/rcc_stm32.c \
            STM32/sdio_f7xx.c \
            STM32/serial_uart_hal.c \
            STM32/serial_uart_stm32f7xx.c \
            STM32/system_stm32f7xx.c \
            STM32/timer_hal.c \
            STM32/timer_stm32f7xx.c \
            STM32/transponder_ir_io_hal.c \
            STM32/camera_control_stm32.c \
            drivers/adc.c \
            drivers/bus_i2c_config.c \
            drivers/bus_spi_config.c \
            drivers/bus_spi_pinconfig.c \
            drivers/serial_escserial.c \
            drivers/serial_pinconfig.c \
            drivers/serial_uart_pinconfig.c \
            STM32/startup/system_stm32f7xx.c

MSC_SRC = \
            drivers/usb_msc_common.c \
            STM32/usb_msc_hal.c \
            msc/usbd_storage.c \
            msc/usbd_storage_emfat.c \
            msc/emfat.c \
            msc/emfat_file.c \
            msc/usbd_storage_sdio.c \
            msc/usbd_storage_sd_spi.c

SPEED_OPTIMISED_SRC += \
            common/stm32/system.c \
            STM32/bus_i2c_hal.c \
            STM32/bus_spi_ll.c \
            drivers/max7456.c \
            drivers/pwm_output_dshot_shared.c \
            STM32/pwm_output_dshot_hal.c \
            STM32/exti.c

SIZE_OPTIMISED_SRC += \
            drivers/bus_i2c_timing.c \
            STM32/bus_i2c_hal_init.c \
            STM32/serial_usb_vcp.c \
            drivers/bus_i2c_config.c \
            drivers/bus_spi_config.c \
            drivers/bus_spi_pinconfig.c \
            drivers/serial_escserial.c \
            drivers/serial_pinconfig.c \
            drivers/serial_uart_pinconfig.c

DSP_LIB := $(LIB_MAIN_DIR)/CMSIS/DSP
DEVICE_FLAGS += -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -D__FPU_PRESENT=1 -DUNALIGNED_SUPPORT_DISABLE -DARM_MATH_CM7

include $(TARGET_PLATFORM_DIR)/mk/STM32_COMMON.mk
