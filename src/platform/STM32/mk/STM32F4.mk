#
# F4 Make file include
#

#CMSIS
ifeq ($(PERIPH_DRIVER), HAL)
CMSIS_DIR      := $(LIB_MAIN_DIR)/STM32F4/Drivers/CMSIS
STDPERIPH_DIR   = $(LIB_MAIN_DIR)/STM32F4/Drivers/STM32F4xx_HAL_Driver
STDPERIPH_SRC   = $(notdir $(wildcard $(STDPERIPH_DIR)/Src/*.c))
EXCLUDES        =

VPATH       := $(VPATH):$(STDPERIPH_DIR)/Src

else
CMSIS_DIR      := $(LIB_MAIN_DIR)/CMSIS
STDPERIPH_DIR   = $(LIB_MAIN_DIR)/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver
STDPERIPH_SRC   = \
            misc.c \
            stm32f4xx_adc.c \
            stm32f4xx_dac.c \
            stm32f4xx_dcmi.c \
            stm32f4xx_dfsdm.c \
            stm32f4xx_dma2d.c \
            stm32f4xx_dma.c \
            stm32f4xx_exti.c \
            stm32f4xx_flash.c \
            stm32f4xx_gpio.c \
            stm32f4xx_i2c.c \
            stm32f4xx_iwdg.c \
            stm32f4xx_ltdc.c \
            stm32f4xx_pwr.c \
            stm32f4xx_rcc.c \
            stm32f4xx_rng.c \
            stm32f4xx_rtc.c \
            stm32f4xx_sdio.c \
            stm32f4xx_spi.c \
            stm32f4xx_syscfg.c \
            stm32f4xx_tim.c \
            stm32f4xx_usart.c \
            stm32f4xx_wwdg.c

VPATH       := $(VPATH):$(STDPERIPH_DIR)/src
endif

ifneq ($(TARGET_MCU),$(filter $(TARGET_MCU),STM32F411xE STM32F446xx))
STDPERIPH_SRC += stm32f4xx_fsmc.c
endif

ifeq ($(PERIPH_DRIVER), HAL)
#USB
USBCORE_DIR = $(LIB_MAIN_DIR)/STM32F4/Middlewares/ST/STM32_USB_Device_Library/Core
USBCORE_SRC = $(notdir $(wildcard $(USBCORE_DIR)/Src/*.c))
EXCLUDES    = usbd_conf_template.c
USBCORE_SRC := $(filter-out ${EXCLUDES}, $(USBCORE_SRC))

USBCDC_DIR = $(LIB_MAIN_DIR)/STM32F4/Middlewares/ST/STM32_USB_Device_Library/Class/CDC
USBCDC_SRC = $(notdir $(wildcard $(USBCDC_DIR)/Src/*.c))
EXCLUDES   = usbd_cdc_if_template.c
USBCDC_SRC := $(filter-out ${EXCLUDES}, $(USBCDC_SRC))

VPATH := $(VPATH):$(USBCDC_DIR)/Src:$(USBCORE_DIR)/Src

DEVICE_STDPERIPH_SRC := \
            $(STDPERIPH_SRC) \
            $(USBCORE_SRC) \
            $(USBCDC_SRC)
else
USBCORE_DIR = STM32_USB_Device_Library/Core
USBCORE_SRC = \
            $(USBCORE_DIR)/src/usbd_core.c \
            $(USBCORE_DIR)/src/usbd_ioreq.c \
            $(USBCORE_DIR)/src/usbd_req.c

USBOTG_DIR  = STM32_USB_OTG_Driver
USBOTG_SRC  = \
            $(USBOTG_DIR)/src/usb_core.c \
            $(USBOTG_DIR)/src/usb_dcd.c \
            $(USBOTG_DIR)/src/usb_dcd_int.c

USBCDC_DIR  = STM32_USB_Device_Library/Class/cdc
USBCDC_SRC  = \
            $(USBCDC_DIR)/src/usbd_cdc_core.c

USBMSC_DIR  = STM32_USB_Device_Library/Class/msc
USBMSC_SRC  = \
            $(USBMSC_DIR)/src/usbd_msc_bot.c \
            $(USBMSC_DIR)/src/usbd_msc_core.c \
            $(USBMSC_DIR)/src/usbd_msc_data.c \
            $(USBMSC_DIR)/src/usbd_msc_scsi.c

USBHID_DIR  = STM32_USB_Device_Library/Class/hid
USBHID_SRC  = \
            $(USBHID_DIR)/src/usbd_hid_core.c

USBWRAPPER_DIR  = STM32_USB_Device_Library/Class/hid_cdc_wrapper
USBWRAPPER_SRC  = \
            $(USBWRAPPER_DIR)/src/usbd_hid_cdc_wrapper.c


DEVICE_STDPERIPH_SRC := \
            $(STDPERIPH_SRC) \
            $(USBOTG_SRC) \
            $(USBCORE_SRC) \
            $(USBCDC_SRC) \
            $(USBHID_SRC) \
            $(USBWRAPPER_SRC) \
            $(USBMSC_SRC)
endif

#CMSIS
VPATH        := $(VPATH):$(CMSIS_DIR)/Core/Include:$(LIB_MAIN_DIR)/STM32F4/Drivers/CMSIS/Device/ST/STM32F4xx

ifeq ($(PERIPH_DRIVER), HAL)
CMSIS_SRC       :=
INCLUDE_DIRS    := \
            $(INCLUDE_DIRS) \
            $(TARGET_PLATFORM_DIR) \
            $(TARGET_PLATFORM_DIR)/startup \
            $(STDPERIPH_DIR)/Inc \
            $(LIB_MAIN_DIR)/$(USBCORE_DIR)/Inc \
            $(LIB_MAIN_DIR)/$(USBCDC_DIR)/Inc \
            $(CMSIS_DIR)/Include \
            $(CMSIS_DIR)/Device/ST/STM32F4xx/Include \
            $(TARGET_PLATFORM_DIR)/vcp_hal
else
CMSIS_SRC       := \
            stm32f4xx_gpio.c \
            stm32f4xx_rcc.c

INCLUDE_DIRS    := \
            $(INCLUDE_DIRS) \
            $(TARGET_PLATFORM_DIR) \
            $(TARGET_PLATFORM_DIR)/startup \
            $(STDPERIPH_DIR)/inc \
            $(LIB_MAIN_DIR)/$(USBOTG_DIR)/inc \
            $(LIB_MAIN_DIR)/$(USBCORE_DIR)/inc \
            $(LIB_MAIN_DIR)/$(USBCDC_DIR)/inc \
            $(LIB_MAIN_DIR)/$(USBHID_DIR)/inc \
            $(LIB_MAIN_DIR)/$(USBWRAPPER_DIR)/inc \
            $(LIB_MAIN_DIR)/$(USBMSC_DIR)/inc \
            $(CMSIS_DIR)/Core/Include \
            $(LIB_MAIN_DIR)/STM32F4/Drivers/CMSIS/Device/ST/STM32F4xx \
            $(TARGET_PLATFORM_DIR)/vcpf4
endif

#Flags
ARCH_FLAGS      = -mthumb -mcpu=cortex-m4 -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant

ifeq ($(TARGET_MCU),STM32F411xE)
DEVICE_FLAGS    = -DSTM32F411xE -finline-limit=20
LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_f411.ld
STARTUP_SRC     = STM32/startup/startup_stm32f411xe.s
MCU_FLASH_SIZE  := 512

else ifeq ($(TARGET_MCU),STM32F405xx)
DEVICE_FLAGS    = -DSTM32F40_41xxx -DSTM32F405xx
LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_f405.ld
STARTUP_SRC     = STM32/startup/startup_stm32f40xx.s
MCU_FLASH_SIZE  := 1024

else ifeq ($(TARGET_MCU),STM32F446xx)
DEVICE_FLAGS    = -DSTM32F446xx
LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_f446.ld
STARTUP_SRC     = STM32/startup/startup_stm32f446xx.s
MCU_FLASH_SIZE  := 512

else
$(error Unknown MCU for F4 target)
endif
DEVICE_FLAGS    += -DHSE_VALUE=$(HSE_VALUE) -DSTM32

MCU_COMMON_SRC = \
            common/stm32/system.c \
            common/stm32/config_flash.c \
            drivers/accgyro/accgyro_mpu.c \
            drivers/dshot_bitbang_decode.c \
            drivers/inverter.c \
            STM32/pwm_output_dshot.c \
            STM32/adc_stm32f4xx.c \
            STM32/bus_i2c_stm32f4xx.c \
            STM32/bus_spi_stdperiph.c \
            STM32/debug.c \
            STM32/dma_reqmap_mcu.c \
            STM32/dma_stm32f4xx.c \
            STM32/dshot_bitbang.c \
            STM32/dshot_bitbang_stdperiph.c \
            STM32/exti.c \
            STM32/io_stm32.c \
            STM32/light_ws2811strip_stdperiph.c \
            STM32/persistent.c \
            STM32/pwm_output.c \
            STM32/rcc_stm32.c \
            STM32/sdio_f4xx.c \
            STM32/serial_uart_stdperiph.c \
            STM32/serial_uart_stm32f4xx.c \
            STM32/system_stm32f4xx.c \
            STM32/timer_stdperiph.c \
            STM32/timer_stm32f4xx.c \
            STM32/transponder_ir_io_stdperiph.c \
            STM32/usbd_msc_desc.c \
            STM32/camera_control_stm32.c \
            drivers/adc.c \
            drivers/bus_i2c_config.c \
            drivers/bus_spi_config.c \
            drivers/serial_escserial.c \
            drivers/serial_pinconfig.c \
            drivers/serial_uart_pinconfig.c \
            STM32/startup/system_stm32f4xx.c

SPEED_OPTIMISED_SRC += \
            common/stm32/system.c \
            STM32/exti.c

SIZE_OPTIMISED_SRC += \
            STM32/serial_usb_vcp.c \
            drivers/inverter.c \
            drivers/bus_i2c_config.c \
            drivers/bus_spi_config.c \
            drivers/serial_escserial.c \
            drivers/serial_pinconfig.c

ifeq ($(PERIPH_DRIVER), HAL)
VCP_SRC = \
            STM32/vcp_hal/usbd_desc.c \
            STM32/vcp_hal/usbd_conf.c \
            STM32/vcp_hal/usbd_cdc_interface.c \
            STM32/serial_usb_vcp.c \
            drivers/usb_io.c
else
VCP_SRC = \
            STM32/vcpf4/stm32f4xx_it.c \
            STM32/vcpf4/usb_bsp.c \
            STM32/vcpf4/usbd_desc.c \
            STM32/vcpf4/usbd_usr.c \
            STM32/vcpf4/usbd_cdc_vcp.c \
            STM32/vcpf4/usb_cdc_hid.c \
            STM32/serial_usb_vcp.c \
            drivers/usb_io.c
endif

MSC_SRC = \
            drivers/usb_msc_common.c \
            STM32/usb_msc_f4xx.c \
            msc/usbd_storage.c \
            msc/usbd_storage_emfat.c \
            msc/emfat.c \
            msc/emfat_file.c \
            msc/usbd_storage_sd_spi.c \
            msc/usbd_storage_sdio.c

DSP_LIB := $(LIB_MAIN_DIR)/CMSIS/DSP
DEVICE_FLAGS += -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -D__FPU_PRESENT=1 -DUNALIGNED_SUPPORT_DISABLE -DARM_MATH_CM4

include $(TARGET_PLATFORM_DIR)/mk/STM32_COMMON.mk
