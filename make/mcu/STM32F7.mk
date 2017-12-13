#
# F7 Make file include
#

ifeq ($(DEBUG_HARDFAULTS),F7)
CFLAGS               += -DDEBUG_HARDFAULTS
endif

#CMSIS
CMSIS_DIR      := $(ROOT)/lib/main/STM32F7/Drivers/CMSIS

#STDPERIPH
STDPERIPH_DIR   = $(ROOT)/lib/main/STM32F7/Drivers/STM32F7xx_HAL_Driver
STDPERIPH_SRC   = $(notdir $(wildcard $(STDPERIPH_DIR)/Src/*.c))
EXCLUDES        = stm32f7xx_hal_can.c \
                  stm32f7xx_hal_cec.c \
                  stm32f7xx_hal_crc.c \
                  stm32f7xx_hal_crc_ex.c \
                  stm32f7xx_hal_cryp.c \
                  stm32f7xx_hal_cryp_ex.c \
                  stm32f7xx_hal_dac.c \
                  stm32f7xx_hal_dac_ex.c \
                  stm32f7xx_hal_dcmi.c \
                  stm32f7xx_hal_dcmi_ex.c \
                  stm32f7xx_hal_dfsdm.c \
                  stm32f7xx_hal_dma2d.c \
                  stm32f7xx_hal_dsi.c \
                  stm32f7xx_hal_eth.c \
                  stm32f7xx_hal_hash.c \
                  stm32f7xx_hal_hash_ex.c \
                  stm32f7xx_hal_hcd.c \
                  stm32f7xx_hal_i2s.c \
                  stm32f7xx_hal_irda.c \
                  stm32f7xx_hal_iwdg.c \
                  stm32f7xx_hal_jpeg.c \
                  stm32f7xx_hal_lptim.c \
                  stm32f7xx_hal_ltdc.c \
                  stm32f7xx_hal_ltdc_ex.c \
                  stm32f7xx_hal_mdios.c \
                  stm32f7xx_hal_mmc.c \
                  stm32f7xx_hal_msp_template.c \
                  stm32f7xx_hal_nand.c \
                  stm32f7xx_hal_nor.c \
                  stm32f7xx_hal_qspi.c \
                  stm32f7xx_hal_rng.c \
                  stm32f7xx_hal_rtc.c \
                  stm32f7xx_hal_rtc_ex.c \
                  stm32f7xx_hal_sai.c \
                  stm32f7xx_hal_sai_ex.c \
                  stm32f7xx_hal_sd.c \
                  stm32f7xx_hal_sdram.c \
                  stm32f7xx_hal_smartcard.c \
                  stm32f7xx_hal_smartcard_ex.c \
                  stm32f7xx_hal_smbus.c \
                  stm32f7xx_hal_spdifrx.c \
                  stm32f7xx_hal_sram.c \
                  stm32f7xx_hal_timebase_rtc_alarm_template.c \
                  stm32f7xx_hal_timebase_rtc_wakeup_template.c \
                  stm32f7xx_hal_timebase_tim_template.c \
                  stm32f7xx_hal_wwdg.c \
                  stm32f7xx_ll_adc.c \
                  stm32f7xx_ll_crc.c \
                  stm32f7xx_ll_dac.c \
                  stm32f7xx_ll_exti.c \
                  stm32f7xx_ll_fmc.c \
                  stm32f7xx_ll_i2c.c \
                  stm32f7xx_ll_lptim.c \
                  stm32f7xx_ll_pwr.c \
                  stm32f7xx_ll_rng.c \
                  stm32f7xx_ll_rtc.c \
                  stm32f7xx_ll_sdmmc.c \
                  stm32f7xx_ll_usart.c

STDPERIPH_SRC   := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))

#USB
USBCORE_DIR = $(ROOT)/lib/main/STM32F7/Middlewares/ST/STM32_USB_Device_Library/Core
USBCORE_SRC = $(notdir $(wildcard $(USBCORE_DIR)/Src/*.c))
EXCLUDES    = usbd_conf_template.c
USBCORE_SRC := $(filter-out ${EXCLUDES}, $(USBCORE_SRC))

USBCDC_DIR = $(ROOT)/lib/main/STM32F7/Middlewares/ST/STM32_USB_Device_Library/Class/CDC
USBCDC_SRC = $(notdir $(wildcard $(USBCDC_DIR)/Src/*.c))
EXCLUDES   = usbd_cdc_if_template.c
USBCDC_SRC := $(filter-out ${EXCLUDES}, $(USBCDC_SRC))

VPATH := $(VPATH):$(USBCDC_DIR)/Src:$(USBCORE_DIR)/Src

DEVICE_STDPERIPH_SRC := $(STDPERIPH_SRC) \
                        $(USBCORE_SRC) \
                        $(USBCDC_SRC)

#CMSIS
VPATH           := $(VPATH):$(CMSIS_DIR)/Include:$(CMSIS_DIR)/Device/ST/STM32F7xx
VPATH           := $(VPATH):$(STDPERIPH_DIR)/Src
CMSIS_SRC       :=
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(STDPERIPH_DIR)/Inc \
                   $(USBCORE_DIR)/Inc \
                   $(USBCDC_DIR)/Inc \
                   $(CMSIS_DIR)/Include \
                   $(CMSIS_DIR)/Device/ST/STM32F7xx/Include \
                   $(ROOT)/src/main/vcp_hal

ifneq ($(filter SDCARD,$(FEATURES)),)
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(FATFS_DIR)
VPATH           := $(VPATH):$(FATFS_DIR)
endif

#Flags
ARCH_FLAGS      = -mthumb -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-sp-d16 -fsingle-precision-constant -Wdouble-promotion

DEVICE_FLAGS    = -DUSE_HAL_DRIVER -DUSE_FULL_LL_DRIVER
ifeq ($(TARGET),$(filter $(TARGET),$(F7X5XG_TARGETS)))
DEVICE_FLAGS   += -DSTM32F745xx
LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_f745.ld
STARTUP_SRC     = startup_stm32f745xx.s
TARGET_FLASH   := 2048
else ifeq ($(TARGET),$(filter $(TARGET),$(F7X6XG_TARGETS)))
DEVICE_FLAGS   += -DSTM32F746xx
LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_f746.ld
STARTUP_SRC     = startup_stm32f746xx.s
TARGET_FLASH   := 2048
else ifeq ($(TARGET),$(filter $(TARGET),$(F7X2RE_TARGETS)))
DEVICE_FLAGS   += -DSTM32F722xx
ifndef LD_SCRIPT
LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_f722.ld
endif
STARTUP_SRC     = startup_stm32f722xx.s
TARGET_FLASH   := 512
else
$(error Unknown MCU for F7 target)
endif
DEVICE_FLAGS    += -DHSE_VALUE=$(HSE_VALUE)

TARGET_FLAGS    = -D$(TARGET)

VCP_SRC = \
            vcp_hal/usbd_desc.c \
            vcp_hal/usbd_conf.c \
            vcp_hal/usbd_cdc_interface.c \
            drivers/serial_usb_vcp.c \
            drivers/usb_io.c

MCU_COMMON_SRC = \
            target/system_stm32f7xx.c \
            drivers/accgyro/accgyro_mpu.c \
            drivers/adc_stm32f7xx.c \
            drivers/bus_i2c_hal.c \
            drivers/dma_stm32f7xx.c \
            drivers/light_ws2811strip_hal.c \
            drivers/bus_spi_ll.c \
            drivers/pwm_output_dshot_hal.c \
            drivers/timer_hal.c \
            drivers/timer_stm32f7xx.c \
            drivers/system_stm32f7xx.c \
            drivers/serial_uart_stm32f7xx.c \
            drivers/serial_uart_hal.c

MCU_EXCLUDES = \
            drivers/bus_i2c.c \
            drivers/timer.c \
            drivers/serial_uart.c

DSP_LIB := $(ROOT)/lib/main/DSP_Lib
DEVICE_FLAGS += -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -D__FPU_PRESENT=1 -DUNALIGNED_SUPPORT_DISABLE -DARM_MATH_CM7
