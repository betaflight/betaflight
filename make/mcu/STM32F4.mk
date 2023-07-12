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
STDPERIPH_SRC   = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))
EXCLUDES        = stm32f4xx_crc.c \
                  stm32f4xx_can.c \
                  stm32f4xx_fmc.c \
                  stm32f4xx_sai.c \
                  stm32f4xx_cec.c \
                  stm32f4xx_dsi.c \
                  stm32f4xx_flash_ramfunc.c \
                  stm32f4xx_fmpi2c.c \
                  stm32f4xx_lptim.c \
                  stm32f4xx_qspi.c \
                  stm32f4xx_spdifrx.c \
                  stm32f4xx_cryp.c \
                  stm32f4xx_cryp_aes.c \
                  stm32f4xx_hash_md5.c \
                  stm32f4xx_cryp_des.c \
                  stm32f4xx_hash.c \
                  stm32f4xx_dbgmcu.c \
                  stm32f4xx_cryp_tdes.c \
                  stm32f4xx_hash_sha1.c

VPATH       := $(VPATH):$(STDPERIPH_DIR)/src
endif

ifeq ($(TARGET_MCU),$(filter $(TARGET_MCU),STM32F411xE STM32F446xx))
EXCLUDES        += stm32f4xx_fsmc.c
endif

STDPERIPH_SRC   := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))

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

DEVICE_STDPERIPH_SRC := $(STDPERIPH_SRC) \
                        $(USBCORE_SRC) \
                        $(USBCDC_SRC)
else
USBCORE_DIR = $(ROOT)/lib/main/STM32_USB_Device_Library/Core
USBCORE_SRC = $(notdir $(wildcard $(USBCORE_DIR)/src/*.c))
USBOTG_DIR  = $(ROOT)/lib/main/STM32_USB_OTG_Driver
USBOTG_SRC  = $(notdir $(wildcard $(USBOTG_DIR)/src/*.c))
EXCLUDES    = usb_bsp_template.c \
              usb_conf_template.c \
              usb_hcd_int.c \
              usb_hcd.c \
              usb_otg.c

USBOTG_SRC  := $(filter-out ${EXCLUDES}, $(USBOTG_SRC))
USBCDC_DIR  = $(ROOT)/lib/main/STM32_USB_Device_Library/Class/cdc
USBCDC_SRC  = $(notdir $(wildcard $(USBCDC_DIR)/src/*.c))
EXCLUDES    = usbd_cdc_if_template.c
USBCDC_SRC  := $(filter-out ${EXCLUDES}, $(USBCDC_SRC))
USBMSC_DIR  = $(ROOT)/lib/main/STM32_USB_Device_Library/Class/msc
USBMSC_SRC  = $(notdir $(wildcard $(USBMSC_DIR)/src/*.c))
EXCLUDES    = usbd_storage_template.c
USBMSC_SRC  := $(filter-out ${EXCLUDES}, $(USBMSC_SRC))
USBHID_DIR  = $(ROOT)/lib/main/STM32_USB_Device_Library/Class/hid
USBHID_SRC  = $(notdir $(wildcard $(USBHID_DIR)/src/*.c))
USBWRAPPER_DIR  = $(ROOT)/lib/main/STM32_USB_Device_Library/Class/hid_cdc_wrapper
USBWRAPPER_SRC  = $(notdir $(wildcard $(USBWRAPPER_DIR)/src/*.c))
VPATH       := $(VPATH):$(USBOTG_DIR)/src:$(USBCORE_DIR)/src:$(USBCDC_DIR)/src:$(USBMSC_DIR)/src:$(USBHID_DIR)/src:$(USBWRAPPER_DIR)/src

DEVICE_STDPERIPH_SRC := $(STDPERIPH_SRC) \
                        $(USBOTG_SRC) \
                        $(USBCORE_SRC) \
                        $(USBCDC_SRC) \
                        $(USBHID_SRC) \
                        $(USBWRAPPER_SRC) \
                        $(USBMSC_SRC)
endif

#CMSIS
VPATH           := $(VPATH):$(CMSIS_DIR)/Core/Include:$(ROOT)/lib/main/STM32F4/Drivers/CMSIS/Device/ST/STM32F4xx

INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(ROOT)/src/main/drivers/stm32

ifeq ($(PERIPH_DRIVER), HAL)
CMSIS_SRC       :=
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(STDPERIPH_DIR)/Inc \
                   $(USBCORE_DIR)/Inc \
                   $(USBCDC_DIR)/Inc \
                   $(CMSIS_DIR)/Include \
                   $(CMSIS_DIR)/Device/ST/STM32F4xx/Include \
                   $(ROOT)/src/main/drivers/stm32/vcp_hal
else
CMSIS_SRC       := $(notdir $(wildcard $(CMSIS_DIR)/CoreSupport/*.c \
                   $(ROOT)/lib/main/STM32F4/Drivers/CMSIS/Device/ST/STM32F4xx/*.c))
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(STDPERIPH_DIR)/inc \
                   $(USBOTG_DIR)/inc \
                   $(USBCORE_DIR)/inc \
                   $(USBCDC_DIR)/inc \
                   $(USBHID_DIR)/inc \
                   $(USBWRAPPER_DIR)/inc \
                   $(USBMSC_DIR)/inc \
                   $(CMSIS_DIR)/Core/Include \
                   $(ROOT)/lib/main/STM32F4/Drivers/CMSIS/Device/ST/STM32F4xx \
                   $(ROOT)/src/main/drivers/stm32/vcpf4
endif

#Flags
ARCH_FLAGS      = -mthumb -mcpu=cortex-m4 -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant

ifeq ($(TARGET_MCU),STM32F411xE)
DEVICE_FLAGS    = -DSTM32F411xE -finline-limit=20
LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_f411.ld
STARTUP_SRC     = startup_stm32f411xe.s
MCU_FLASH_SIZE  := 512

else ifeq ($(TARGET_MCU),STM32F405xx)
DEVICE_FLAGS    = -DSTM32F40_41xxx -DSTM32F405xx
LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_f405.ld
STARTUP_SRC     = startup_stm32f40xx.s
MCU_FLASH_SIZE  := 1024

else ifeq ($(TARGET_MCU),STM32F446xx)
DEVICE_FLAGS    = -DSTM32F446xx
LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_f446.ld
STARTUP_SRC     = startup_stm32f446xx.s
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
            drivers/stm32/pwm_output_dshot.c \
            drivers/stm32/adc_stm32f4xx.c \
            drivers/stm32/bus_i2c_stm32f4xx.c \
            drivers/stm32/bus_spi_stdperiph.c \
            drivers/stm32/debug.c \
            drivers/stm32/dma_reqmap_mcu.c \
            drivers/stm32/dma_stm32f4xx.c \
            drivers/stm32/dshot_bitbang.c \
            drivers/stm32/dshot_bitbang_stdperiph.c \
            drivers/stm32/exti.c \
            drivers/stm32/io_stm32.c \
            drivers/stm32/light_ws2811strip_stdperiph.c \
            drivers/stm32/persistent.c \
            drivers/stm32/pwm_output.c \
            drivers/stm32/rcc_stm32.c \
            drivers/stm32/sdio_f4xx.c \
            drivers/stm32/serial_uart_stdperiph.c \
            drivers/stm32/serial_uart_stm32f4xx.c \
            drivers/stm32/system_stm32f4xx.c \
            drivers/stm32/timer_stdperiph.c \
            drivers/stm32/timer_stm32f4xx.c \
            drivers/stm32/transponder_ir_io_stdperiph.c \
            drivers/stm32/usbd_msc_desc.c \
            drivers/stm32/camera_control.c \
            startup/system_stm32f4xx.c

ifeq ($(PERIPH_DRIVER), HAL)
VCP_SRC = \
            drivers/stm32/vcp_hal/usbd_desc.c \
            drivers/stm32/vcp_hal/usbd_conf.c \
            drivers/stm32/vcp_hal/usbd_cdc_interface.c \
            drivers/stm32/serial_usb_vcp.c \
            drivers/usb_io.c
else
VCP_SRC = \
            drivers/stm32/vcpf4/stm32f4xx_it.c \
            drivers/stm32/vcpf4/usb_bsp.c \
            drivers/stm32/vcpf4/usbd_desc.c \
            drivers/stm32/vcpf4/usbd_usr.c \
            drivers/stm32/vcpf4/usbd_cdc_vcp.c \
            drivers/stm32/vcpf4/usb_cdc_hid.c \
            drivers/stm32/serial_usb_vcp.c \
            drivers/usb_io.c
endif

MSC_SRC = \
            drivers/usb_msc_common.c \
            drivers/stm32/usb_msc_f4xx.c \
            msc/usbd_storage.c \
            msc/usbd_storage_emfat.c \
            msc/emfat.c \
            msc/emfat_file.c \
            msc/usbd_storage_sd_spi.c \
            msc/usbd_storage_sdio.c

DSP_LIB := $(ROOT)/lib/main/CMSIS/DSP
DEVICE_FLAGS += -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -D__FPU_PRESENT=1 -DUNALIGNED_SUPPORT_DISABLE -DARM_MATH_CM4
