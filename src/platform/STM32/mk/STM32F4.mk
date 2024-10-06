#
# F4 Make file include
#

#CMSIS
ifeq ($(PERIPH_DRIVER), HAL)
CMSIS_DIR      := $(ROOT)/lib/main/STM32F4/Drivers/CMSIS
STDPERIPH_DIR   = $(ROOT)/lib/main/STM32F4/Drivers/STM32F4xx_HAL_Driver
STDPERIPH_SRC   = $(notdir $(wildcard $(STDPERIPH_DIR)/Src/*.c))
EXCLUDES        =

VPATH       := $(VPATH):$(STDPERIPH_DIR)/Src

else
CMSIS_DIR      := $(ROOT)/lib/main/CMSIS
STDPERIPH_DIR   = $(ROOT)/lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver
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
USBCORE_DIR = $(ROOT)/lib/main/STM32F4/Middlewares/ST/STM32_USB_Device_Library/Core
USBCORE_SRC = $(notdir $(wildcard $(USBCORE_DIR)/Src/*.c))
EXCLUDES    = usbd_conf_template.c
USBCORE_SRC := $(filter-out ${EXCLUDES}, $(USBCORE_SRC))

USBCDC_DIR = $(ROOT)/lib/main/STM32F4/Middlewares/ST/STM32_USB_Device_Library/Class/CDC
USBCDC_SRC = $(notdir $(wildcard $(USBCDC_DIR)/Src/*.c))
EXCLUDES   = usbd_cdc_if_template.c
USBCDC_SRC := $(filter-out ${EXCLUDES}, $(USBCDC_SRC))

VPATH := $(VPATH):$(USBCDC_DIR)/Src:$(USBCORE_DIR)/Src

DEVICE_STDPERIPH_SRC := \
            $(STDPERIPH_SRC) \
            $(USBCORE_SRC) \
            $(USBCDC_SRC)
else
USBCORE_DIR = $(ROOT)/lib/main/STM32_USB_Device_Library/Core
USBCORE_SRC = \
            usbd_core.c \
            usbd_ioreq.c \
            usbd_req.c

USBOTG_DIR  = $(ROOT)/lib/main/STM32_USB_OTG_Driver
USBOTG_SRC  = \
            usb_core.c \
            usb_dcd.c \
            usb_dcd_int.c

USBCDC_DIR  = $(ROOT)/lib/main/STM32_USB_Device_Library/Class/cdc
USBCDC_SRC  = usbd_cdc_core.c

USBMSC_DIR  = $(ROOT)/lib/main/STM32_USB_Device_Library/Class/msc
USBMSC_SRC  = \
            usbd_msc_bot.c \
            usbd_msc_core.c \
            usbd_msc_data.c \
            usbd_msc_scsi.c

USBHID_DIR  = $(ROOT)/lib/main/STM32_USB_Device_Library/Class/hid
USBHID_SRC  = usbd_hid_core.c

USBWRAPPER_DIR  = $(ROOT)/lib/main/STM32_USB_Device_Library/Class/hid_cdc_wrapper
USBWRAPPER_SRC  = usbd_hid_cdc_wrapper.c

VPATH       := $(VPATH):$(USBOTG_DIR)/src:$(USBCORE_DIR)/src:$(USBCDC_DIR)/src:$(USBMSC_DIR)/src:$(USBHID_DIR)/src:$(USBWRAPPER_DIR)/src

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
VPATH        := $(VPATH):$(CMSIS_DIR)/Core/Include:$(ROOT)/lib/main/STM32F4/Drivers/CMSIS/Device/ST/STM32F4xx

INCLUDE_DIRS := \
            $(INCLUDE_DIRS) \
            $(SRC_DIR)/startup/stm32 \
            $(SRC_DIR)/drivers/mcu/stm32

ifeq ($(PERIPH_DRIVER), HAL)
CMSIS_SRC       :=
INCLUDE_DIRS    := \
            $(INCLUDE_DIRS) \
            $(TARGET_PLATFORM_DIR) \
            $(TARGET_PLATFORM_DIR)/startup \
            $(STDPERIPH_DIR)/Inc \
            $(USBCORE_DIR)/Inc \
            $(USBCDC_DIR)/Inc \
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
            $(USBOTG_DIR)/inc \
            $(USBCORE_DIR)/inc \
            $(USBCDC_DIR)/inc \
            $(USBHID_DIR)/inc \
            $(USBWRAPPER_DIR)/inc \
            $(USBMSC_DIR)/inc \
            $(CMSIS_DIR)/Core/Include \
            $(ROOT)/lib/main/STM32F4/Drivers/CMSIS/Device/ST/STM32F4xx \
            $(TARGET_PLATFORM_DIR)/vcpf4
endif

#Flags
ARCH_FLAGS      = -mthumb -mcpu=cortex-m4 -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant

ifeq ($(TARGET_MCU),STM32F411xE)
DEVICE_FLAGS    = -DSTM32F411xE -finline-limit=20
LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_f411.ld
STARTUP_SRC     = startup/startup_stm32f411xe.s
MCU_FLASH_SIZE  := 512

else ifeq ($(TARGET_MCU),STM32F405xx)
DEVICE_FLAGS    = -DSTM32F40_41xxx -DSTM32F405xx
LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_f405.ld
STARTUP_SRC     = startup/startup_stm32f40xx.s
MCU_FLASH_SIZE  := 1024

else ifeq ($(TARGET_MCU),STM32F446xx)
DEVICE_FLAGS    = -DSTM32F446xx
LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_f446.ld
STARTUP_SRC     = startup/startup_stm32f446xx.s
MCU_FLASH_SIZE  := 512

else
$(error Unknown MCU for F4 target)
endif
DEVICE_FLAGS    += -DHSE_VALUE=$(HSE_VALUE) -DSTM32

MCU_COMMON_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/dshot_bitbang_decode.c \
            drivers/inverter.c \
            drivers/pwm_output_dshot_shared.c \
            pwm_output_dshot.c \
            adc_stm32f4xx.c \
            bus_i2c_stm32f4xx.c \
            bus_spi_stdperiph.c \
            debug.c \
            dma_reqmap_mcu.c \
            dma_stm32f4xx.c \
            dshot_bitbang.c \
            dshot_bitbang_stdperiph.c \
            exti.c \
            io_stm32.c \
            light_ws2811strip_stdperiph.c \
            persistent.c \
            pwm_output.c \
            rcc_stm32.c \
            sdio_f4xx.c \
            serial_uart_stdperiph.c \
            serial_uart_stm32f4xx.c \
            system_stm32f4xx.c \
            timer_stdperiph.c \
            timer_stm32f4xx.c \
            transponder_ir_io_stdperiph.c \
            usbd_msc_desc.c \
            camera_control.c \
            startup/system_stm32f4xx.c

ifeq ($(PERIPH_DRIVER), HAL)
VCP_SRC = \
            vcp_hal/usbd_desc.c \
            vcp_hal/usbd_conf.c \
            vcp_hal/usbd_cdc_interface.c \
            serial_usb_vcp.c \
            drivers/usb_io.c
else
VCP_SRC = \
            vcpf4/stm32f4xx_it.c \
            vcpf4/usb_bsp.c \
            vcpf4/usbd_desc.c \
            vcpf4/usbd_usr.c \
            vcpf4/usbd_cdc_vcp.c \
            vcpf4/usb_cdc_hid.c \
            serial_usb_vcp.c \
            drivers/usb_io.c
endif

MSC_SRC = \
            drivers/usb_msc_common.c \
            usb_msc_f4xx.c \
            msc/usbd_storage.c \
            msc/usbd_storage_emfat.c \
            msc/emfat.c \
            msc/emfat_file.c \
            msc/usbd_storage_sd_spi.c \
            msc/usbd_storage_sdio.c

DSP_LIB := $(ROOT)/lib/main/CMSIS/DSP
DEVICE_FLAGS += -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -D__FPU_PRESENT=1 -DUNALIGNED_SUPPORT_DISABLE -DARM_MATH_CM4
