#
# G4 Make file include
#

ifeq ($(DEBUG_HARDFAULTS),G4)
CFLAGS          += -DDEBUG_HARDFAULTS
endif

#CMSIS
CMSIS_DIR      := $(ROOT)/lib/main/CMSIS

#STDPERIPH
STDPERIPH_DIR   = $(ROOT)/lib/main/STM32G4/Drivers/STM32G4xx_HAL_Driver
STDPERIPH_SRC   = \
            stm32g4xx_hal_adc.c \
            stm32g4xx_hal_adc_ex.c \
            stm32g4xx_hal.c \
            stm32g4xx_hal_cordic.c \
            stm32g4xx_hal_cortex.c \
            stm32g4xx_hal_dma.c \
            stm32g4xx_hal_exti.c \
            stm32g4xx_hal_fdcan.c \
            stm32g4xx_hal_flash.c \
            stm32g4xx_hal_flash_ex.c \
            stm32g4xx_hal_fmac.c \
            stm32g4xx_hal_gpio.c \
            stm32g4xx_hal_i2c.c \
            stm32g4xx_hal_i2c_ex.c \
            stm32g4xx_hal_pcd.c \
            stm32g4xx_hal_pcd_ex.c \
            stm32g4xx_hal_pwr.c \
            stm32g4xx_hal_pwr_ex.c \
            stm32g4xx_hal_rcc.c \
            stm32g4xx_hal_rcc_ex.c \
            stm32g4xx_hal_rtc.c \
            stm32g4xx_hal_rtc_ex.c \
            stm32g4xx_hal_tim.c \
            stm32g4xx_hal_tim_ex.c \
            stm32g4xx_hal_uart.c \
            stm32g4xx_hal_uart_ex.c \
            stm32g4xx_ll_dma.c \
            stm32g4xx_ll_spi.c \
            stm32g4xx_ll_tim.c \
            stm32g4xx_ll_usb.c

#USB
USBCORE_DIR = $(ROOT)/lib/main/STM32G4/Middlewares/ST/STM32_USB_Device_Library/Core
USBCORE_SRC = \
            usbd_core.c \
            usbd_ctlreq.c \
            usbd_ioreq.c

USBCDC_DIR = $(ROOT)/lib/main/STM32G4/Middlewares/ST/STM32_USB_Device_Library/Class/CDC
USBCDC_SRC = usbd_cdc.c

USBHID_DIR = $(ROOT)/lib/main/STM32G4/Middlewares/ST/STM32_USB_Device_Library/Class/HID
USBHID_SRC = usbd_hid.c

USBMSC_DIR = $(ROOT)/lib/main/STM32G4/Middlewares/ST/STM32_USB_Device_Library/Class/MSC
USBMSC_SRC = \
            usbd_msc_bot.c \
            usbd_msc.c \
            usbd_msc_data.c \
            usbd_msc_scsi.c

VPATH := $(VPATH):$(USBCDC_DIR)/Src:$(USBCORE_DIR)/Src:$(USBHID_DIR)/Src:$(USBMSC_DIR)/Src:$(STDPERIPH_DIR)/src

DEVICE_STDPERIPH_SRC := \
            $(STDPERIPH_SRC) \
            $(USBCORE_SRC) \
            $(USBCDC_SRC) \
            $(USBHID_SRC) \
            $(USBMSC_SRC)

#CMSIS
VPATH           := $(VPATH):$(CMSIS_DIR)/Include:$(CMSIS_DIR)/Device/ST/STM32G4xx
VPATH           := $(VPATH):$(STDPERIPH_DIR)/Src
CMSIS_SRC       :=
INCLUDE_DIRS    := \
            $(INCLUDE_DIRS) \
            $(TARGET_PLATFORM_DIR) \
            $(TARGET_PLATFORM_DIR)/startup \
            $(SRC_DIR)/startup/stm32 \
            $(STDPERIPH_DIR)/Inc \
            $(USBCORE_DIR)/Inc \
            $(USBCDC_DIR)/Inc \
            $(USBHID_DIR)/Inc \
            $(USBMSC_DIR)/Inc \
            $(CMSIS_DIR)/Core/Include \
            $(ROOT)/lib/main/STM32G4/Drivers/CMSIS/Device/ST/STM32G4xx/Include \
            $(TARGET_PLATFORM_DIR)/vcp_hal

#Flags
ARCH_FLAGS      = -mthumb -mcpu=cortex-m4 -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant

DEVICE_FLAGS    = -DUSE_HAL_DRIVER -DUSE_FULL_LL_DRIVER -DUSE_DMA_RAM -DMAX_MPU_REGIONS=16

# G47X_TARGETS includes G47{3,4}{RE,CE,CEU}

ifeq ($(TARGET_MCU),STM32G474xx)
DEVICE_FLAGS    += -DSTM32G474xx
LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_g474.ld
STARTUP_SRC     = startup/startup_stm32g474xx.s
MCU_FLASH_SIZE  = 512
# Override the OPTIMISE_SPEED compiler setting to save flash space on these 512KB targets.
# Performance is only slightly affected but around 50 kB of flash are saved.
OPTIMISE_SPEED  = -O2
else
$(error Unknown MCU for G4 target)
endif
DEVICE_FLAGS    += -DHSE_VALUE=$(HSE_VALUE) -DSTM32

VCP_SRC = \
            vcp_hal/usbd_desc.c \
            vcp_hal/usbd_conf_stm32g4xx.c \
            vcp_hal/usbd_cdc_hid.c \
            vcp_hal/usbd_cdc_interface.c \
            serial_usb_vcp.c \
            drivers/usb_io.c

MCU_COMMON_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/bus_i2c_timing.c \
            drivers/dshot_bitbang_decode.c \
            drivers/pwm_output_dshot_shared.c \
            adc_stm32g4xx.c \
            bus_i2c_hal_init.c \
            bus_i2c_hal.c \
            bus_spi_ll.c \
            debug.c \
            dma_reqmap_mcu.c \
            dma_stm32g4xx.c \
            dshot_bitbang_ll.c \
            dshot_bitbang.c \
            exti.c \
            io_stm32.c \
            light_ws2811strip_hal.c \
            memprot_hal.c \
            memprot_stm32g4xx.c \
            persistent.c \
            pwm_output.c \
            pwm_output_dshot_hal.c \
            rcc_stm32.c \
            serial_uart_hal.c \
            serial_uart_stm32g4xx.c \
            system_stm32g4xx.c \
            timer_hal.c \
            timer_stm32g4xx.c \
            transponder_ir_io_hal.c \
            camera_control.c \
            startup/system_stm32g4xx.c

MCU_EXCLUDES = \
            drivers/bus_i2c.c

# G4's MSC use the same driver layer file with F7
MSC_SRC = \
            drivers/usb_msc_common.c \
            usb_msc_hal.c \
            msc/usbd_storage.c \
            msc/usbd_storage_emfat.c \
            msc/emfat.c \
            msc/emfat_file.c \
            msc/usbd_storage_sdio.c \
            msc/usbd_storage_sd_spi.c

DSP_LIB := $(ROOT)/lib/main/CMSIS/DSP
DEVICE_FLAGS += -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -D__FPU_PRESENT=1 -DUNALIGNED_SUPPORT_DISABLE -DARM_MATH_CM4
