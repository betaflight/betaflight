#
# H5 Make file include
#

ifeq ($(DEBUG_HARDFAULTS),H5)
CFLAGS          += -DDEBUG_HARDFAULTS
endif

#CMSIS
CMSIS_DIR      := $(ROOT)/lib/main/CMSIS

#STDPERIPH
STDPERIPH_DIR   = $(ROOT)/lib/main/STM32H5/Drivers/STM32H5xx_HAL_Driver
STDPERIPH_SRC   = $(notdir $(wildcard $(STDPERIPH_DIR)/Src/*.c))

EXCLUDES        = \
                stm32h5xx_hal_cec.c \
                stm32h5xx_hal_comp.c \
                stm32h5xx_hal_crc.c \
                stm32h5xx_hal_crc_ex.c \
                stm32h5xx_hal_cryp.c \
                stm32h5xx_hal_cryp_ex.c \
                stm32h5xx_hal_dcmi.c \
                stm32h5xx_hal_dfsdm.c \
                stm32h5xx_hal_dma2d.c \
                stm32h5xx_hal_dsi.c \
                stm32h5xx_hal_eth.c \
                stm32h5xx_hal_eth_ex.c \
                stm32h5xx_hal_fdcan.c \
                stm32h5xx_hal_hash.c \
                stm32h5xx_hal_hash_ex.c \
                stm32h5xx_hal_hcd.c \
                stm32h5xx_hal_hrtim.c \
                stm32h5xx_hal_hsem.c \
                stm32h5xx_hal_i2s.c \
                stm32h5xx_hal_i2s_ex.c \
                stm32h5xx_hal_irda.c \
                stm32h5xx_hal_iwdg.c \
                stm32h5xx_hal_jpeg.c \
                stm32h5xx_hal_lptim.c \
                stm32h5xx_hal_ltdc.c \
                stm32h5xx_hal_ltdc_ex.c \
                stm32h5xx_hal_mdios.c \
                stm32h5xx_hal_mdma.c \
                stm32h5xx_hal_mmc.c \
                stm32h5xx_hal_mmc_ex.c \
                stm32h5xx_hal_msp_template.c \
                stm32h5xx_hal_nand.c \
                stm32h5xx_hal_nor.c \
                stm32h5xx_hal_opamp.c \
                stm32h5xx_hal_opamp_ex.c \
                stm32h5xx_hal_ramecc.c \
                stm32h5xx_hal_rng.c \
                stm32h5xx_hal_rtc.c \
                stm32h5xx_hal_sai.c \
                stm32h5xx_hal_sai_ex.c \
                stm32h5xx_hal_sd_ex.c \
                stm32h5xx_hal_sdram.c \
                stm32h5xx_hal_smartcard.c \
                stm32h5xx_hal_smartcard_ex.c \
                stm32h5xx_hal_smbus.c \
                stm32h5xx_hal_spdifrx.c \
                stm32h5xx_hal_spi.c \
                stm32h5xx_hal_sram.c \
                stm32h5xx_hal_swpmi.c \
                stm32h5xx_hal_usart.c \
                stm32h5xx_hal_usart_ex.c \
                stm32h5xx_hal_wwdg.c \
                stm32h5xx_ll_adc.c \
                stm32h5xx_ll_bdma.c \
                stm32h5xx_ll_comp.c \
                stm32h5xx_ll_crc.c \
                stm32h5xx_ll_dac.c \
                stm32h5xx_ll_delayblock.c \
                stm32h5xx_ll_dma2d.c \
                stm32h5xx_ll_exti.c \
                stm32h5xx_ll_fmc.c \
                stm32h5xx_ll_gpio.c \
                stm32h5xx_ll_hrtim.c \
                stm32h5xx_ll_i2c.c \
                stm32h5xx_ll_lptim.c \
                stm32h5xx_ll_lpuart.c \
                stm32h5xx_ll_mdma.c \
                stm32h5xx_ll_opamp.c \
                stm32h5xx_ll_pwr.c \
                stm32h5xx_ll_rcc.c \
                stm32h5xx_ll_rng.c \
                stm32h5xx_ll_rtc.c \
                stm32h5xx_ll_swpmi.c \
                stm32h5xx_ll_usart.c \
                stm32h5xx_ll_utils.c

STDPERIPH_SRC   := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))

#USB
USBCORE_DIR = $(ROOT)/lib/main/STM32H5/Middlewares/ST/STM32_USB_Device_Library/Core
USBCORE_SRC = $(notdir $(wildcard $(USBCORE_DIR)/Src/*.c))
EXCLUDES    = usbd_conf_template.c
USBCORE_SRC := $(filter-out ${EXCLUDES}, $(USBCORE_SRC))

USBCDC_DIR = $(ROOT)/lib/main/STM32H5/Middlewares/ST/STM32_USB_Device_Library/Class/CDC
USBCDC_SRC = $(notdir $(wildcard $(USBCDC_DIR)/Src/*.c))
EXCLUDES   = usbd_cdc_if_template.c
USBCDC_SRC := $(filter-out ${EXCLUDES}, $(USBCDC_SRC))

USBHID_DIR = $(ROOT)/lib/main/STM32H5/Middlewares/ST/STM32_USB_Device_Library/Class/HID
USBHID_SRC = $(notdir $(wildcard $(USBHID_DIR)/Src/*.c))

USBMSC_DIR = $(ROOT)/lib/main/STM32H5/Middlewares/ST/STM32_USB_Device_Library/Class/MSC
USBMSC_SRC = $(notdir $(wildcard $(USBMSC_DIR)/Src/*.c))
EXCLUDES   = usbd_msc_storage_template.c
USBMSC_SRC := $(filter-out ${EXCLUDES}, $(USBMSC_SRC))

VPATH := $(VPATH):$(USBCDC_DIR)/Src:$(USBCORE_DIR)/Src:$(USBHID_DIR)/Src:$(USBMSC_DIR)/Src:$(STDPERIPH_DIR)/src

DEVICE_STDPERIPH_SRC := $(STDPERIPH_SRC) \
                        $(USBCORE_SRC) \
                        $(USBCDC_SRC) \
                        $(USBHID_SRC) \
                        $(USBMSC_SRC)

#CMSIS
VPATH           := $(VPATH):$(CMSIS_DIR)/Include:$(CMSIS_DIR)/Device/ST/stm32h5xx
VPATH           := $(VPATH):$(STDPERIPH_DIR)/Src
CMSIS_SRC       :=
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(STDPERIPH_DIR)/Inc \
                   $(USBCORE_DIR)/Inc \
                   $(USBCDC_DIR)/Inc \
                   $(USBHID_DIR)/Inc \
                   $(USBMSC_DIR)/Inc \
                   $(CMSIS_DIR)/Core/Include \
                   $(ROOT)/lib/main/STM32H5/Drivers/CMSIS/Device/ST/stm32h5xx/Include \
                   $(ROOT)/src/main/drivers/stm32 \
                   $(ROOT)/src/main/drivers/stm32/vcp_hal

#Flags
ARCH_FLAGS      = -mthumb -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-sp-d16 -fsingle-precision-constant

# Flags that are used in the STM32 libraries
DEVICE_FLAGS    = -DUSE_HAL_DRIVER -DUSE_FULL_LL_DRIVER

#
# H563xx : 2M FLASH, 640KB SRAM
#
ifeq ($(TARGET_MCU),STM32H563xx)
DEVICE_FLAGS       += -DSTM32H563xx
DEFAULT_LD_SCRIPT   = $(LINKER_DIR)/stm32_flash_h563_2m.ld
STARTUP_SRC         = startup_stm32h563xx.s
MCU_FLASH_SIZE     := 2048

# end H563xx


ifneq ($(DEBUG),GDB)
OPTIMISE_DEFAULT    := -Os
OPTIMISE_SPEED      := -Os
OPTIMISE_SIZE       := -Os

LTO_FLAGS           := $(OPTIMISATION_BASE) $(OPTIMISE_DEFAULT)
endif

else
$(error Unknown MCU for STM32H5 target)
endif

ifeq ($(LD_SCRIPT),)
LD_SCRIPT = $(DEFAULT_LD_SCRIPT)
endif

ifneq ($(FIRMWARE_SIZE),)
DEVICE_FLAGS   += -DFIRMWARE_SIZE=$(FIRMWARE_SIZE)
endif

DEVICE_FLAGS    += -DHSE_VALUE=$(HSE_VALUE) -DHSE_STARTUP_TIMEOUT=1000 -DSTM32

VCP_SRC = \
            drivers/stm32/vcp_hal/usbd_desc.c \
            drivers/stm32/vcp_hal/usbd_conf_stm32h5xx.c \
            drivers/stm32/vcp_hal/usbd_cdc_hid.c \
            drivers/stm32/vcp_hal/usbd_cdc_interface.c \
            drivers/stm32/serial_usb_vcp.c \
            drivers/usb_io.c

MCU_COMMON_SRC = \
            drivers/bus_i2c_timing.c \
            drivers/bus_quadspi.c \
            drivers/dshot_bitbang_decode.c \
            drivers/pwm_output_dshot_shared.c \
            drivers/stm32/adc_stm32h5xx.c \
            drivers/stm32/audio_stm32h5xx.c \
            drivers/stm32/bus_i2c_hal_init.c \
            drivers/stm32/bus_i2c_hal.c \
            drivers/stm32/bus_spi_ll.c \
            drivers/stm32/bus_quadspi_hal.c \
            drivers/stm32/bus_octospi_stm32h5xx.c \
            drivers/stm32/debug.c \
            drivers/stm32/dma_reqmap_mcu.c \
            drivers/stm32/dma_stm32h5xx.c \
            drivers/stm32/dshot_bitbang_ll.c \
            drivers/stm32/dshot_bitbang.c \
            drivers/stm32/exti.c \
            drivers/stm32/io_stm32.c \
            drivers/stm32/light_ws2811strip_hal.c \
            drivers/stm32/memprot_hal.c \
            drivers/stm32/memprot_stm32h5xx.c \
            drivers/stm32/persistent.c \
            drivers/stm32/pwm_output.c \
            drivers/stm32/pwm_output_dshot_hal.c \
            drivers/stm32/rcc_stm32.c \
            drivers/stm32/sdio_h5xx.c \
            drivers/stm32/serial_uart_hal.c \
            drivers/stm32/serial_uart_stm32h5xx.c \
            drivers/stm32/system_stm32h5xx.c \
            drivers/stm32/timer_hal.c \
            drivers/stm32/timer_stm32h5xx.c \
            drivers/stm32/transponder_ir_io_hal.c \
            drivers/stm32/camera_control.c \
            startup/system_stm32h5xx.c

MCU_EXCLUDES = \
            drivers/bus_i2c.c

MSC_SRC = \
            drivers/stm32/usb_msc_h5xx.c \
            drivers/usb_msc_common.c \
            msc/usbd_storage.c \
            msc/usbd_storage_emfat.c \
            msc/emfat.c \
            msc/emfat_file.c \
            msc/usbd_storage_sd_spi.c \
            msc/usbd_storage_sdio.c

DSP_LIB := $(ROOT)/lib/main/CMSIS/DSP
DEVICE_FLAGS += -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -D__FPU_PRESENT=1 -DUNALIGNED_SUPPORT_DISABLE -DARM_MATH_CM7
