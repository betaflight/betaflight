#
# N6 Make file include
#

ifeq ($(DEBUG_HARDFAULTS),N6)
CFLAGS          += -DDEBUG_HARDFAULTS
endif

#CMSIS
CMSIS_DIR      := $(LIB_MAIN_DIR)/STM32N6/Drivers/CMSIS

#STDPERIPH
STDPERIPH_DIR   = $(LIB_MAIN_DIR)/STM32N6/Drivers/STM32N6xx_HAL_Driver
STDPERIPH_SRC   = \
            stm32n6xx_hal_adc.c \
            stm32n6xx_hal_adc_ex.c \
            stm32n6xx_hal.c \
            stm32n6xx_hal_cortex.c \
            stm32n6xx_hal_dma.c \
            stm32n6xx_hal_dma_ex.c \
            stm32n6xx_hal_dts.c \
            stm32n6xx_hal_exti.c \
            stm32n6xx_hal_gpio.c \
            stm32n6xx_hal_i2c.c \
            stm32n6xx_hal_i2c_ex.c \
            stm32n6xx_hal_pcd.c \
            stm32n6xx_hal_pcd_ex.c \
            stm32n6xx_hal_pwr.c \
            stm32n6xx_hal_pwr_ex.c \
            stm32n6xx_hal_rcc.c \
            stm32n6xx_hal_rcc_ex.c \
            stm32n6xx_hal_rng_ex.c \
            stm32n6xx_hal_rtc.c \
            stm32n6xx_hal_rtc_ex.c \
            stm32n6xx_hal_sd.c \
            stm32n6xx_hal_spi.c \
            stm32n6xx_hal_spi_ex.c \
            stm32n6xx_hal_tim.c \
            stm32n6xx_hal_tim_ex.c \
            stm32n6xx_hal_uart.c \
            stm32n6xx_hal_uart_ex.c \
            stm32n6xx_ll_dma.c \
            stm32n6xx_ll_exti.c \
            stm32n6xx_ll_gpio.c \
            stm32n6xx_ll_rcc.c \
            stm32n6xx_ll_sdmmc.c \
            stm32n6xx_ll_spi.c \
            stm32n6xx_ll_tim.c \
            stm32n6xx_ll_usb.c \
            stm32n6xx_ll_utils.c


#USB
USBCORE_DIR = STM32_USB_Device_Library_HAL/Core
USBCORE_SRC = \
            $(USBCORE_DIR)/Src/usbd_core.c \
            $(USBCORE_DIR)/Src/usbd_ctlreq.c \
            $(USBCORE_DIR)/Src/usbd_ioreq.c

USBCDC_DIR = STM32_USB_Device_Library_HAL/Class/CDC
USBCDC_SRC = \
            $(USBCDC_DIR)/Src/usbd_cdc.c

DEVICE_STDPERIPH_SRC := $(STDPERIPH_SRC) \
                        $(USBCORE_SRC) \
                        $(USBCDC_SRC)

#CMSIS
VPATH           := $(VPATH):$(CMSIS_DIR)/Include:$(CMSIS_DIR)/Device/ST/STM32N6xx:$(STDPERIPH_DIR)/Src
CMSIS_SRC       :=
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(TARGET_PLATFORM_DIR) \
                   $(TARGET_PLATFORM_DIR)/include \
                   $(TARGET_PLATFORM_DIR)/startup \
                   $(STDPERIPH_DIR)/Inc \
                   $(LIB_MAIN_DIR)/$(USBCORE_DIR)/Inc \
                   $(LIB_MAIN_DIR)/$(USBCDC_DIR)/Inc \
                   $(LIB_MAIN_DIR)/STM32N6/Drivers/CMSIS/Core/Include \
                   $(LIB_MAIN_DIR)/STM32N6/Drivers/CMSIS/Device/ST/STM32N6xx/Include \
                   $(LIB_MAIN_DIR)/STM32N6/Drivers/CMSIS/Device/ST/STM32N6xx/Include/Templates \
                   $(TARGET_PLATFORM_DIR)/vcp_hal

#Flags
ARCH_FLAGS      = -mthumb -mcpu=cortex-m55 -mfloat-abi=hard -mfpu=fpv5-d16

# Flags that are used in the STM32 libraries
DEVICE_FLAGS    = -DUSE_HAL_DRIVER -DUSE_FULL_LL_DRIVER

# Suppress old-style-definition warning in vendor HAL source (ST bug: empty () instead of (void))
SRC_CFLAGS_stm32n6xx_hal_rcc_ex.c := -Wno-old-style-definition

ifeq ($(TARGET_MCU),STM32N657xx)
DEVICE_FLAGS       += -DSTM32N657xx
DEFAULT_LD_SCRIPT   = $(LINKER_DIR)/STM32N657XX_LRUN.ld
STARTUP_SRC         = STM32/startup/startup_stm32n657xx.s
MCU_FLASH_SIZE     := 2048
DEVICE_FLAGS       += -DMAX_MPU_REGIONS=16
else
$(error Unknown MCU for N6 target)
endif

ifeq ($(LD_SCRIPT),)
LD_SCRIPT = $(DEFAULT_LD_SCRIPT)
endif

DEVICE_FLAGS    += -DHSE_VALUE=$(HSE_VALUE) -DHSE_STARTUP_TIMEOUT=1000 -DSTM32

VCP_SRC = \
            STM32/vcp_hal/usbd_desc.c \
            STM32/vcp_hal/usbd_conf_stm32n6xx.c \
            STM32/vcp_hal/usbd_cdc_interface.c \
            STM32/serial_usb_vcp.c \
            drivers/usb_io.c

MCU_COMMON_SRC = \
            drivers/bus_i2c_timing.c \
            drivers/dshot_bitbang_decode.c \
            STM32/adc_stm32n6xx.c \
            STM32/bus_i2c_hal_init.c \
            STM32/bus_i2c_hal.c \
            STM32/bus_spi_ll.c \
            STM32/debug.c \
            STM32/dma_reqmap_mcu.c \
            STM32/dma_stm32n6xx.c \
            STM32/dshot_bitbang_ll.c \
            STM32/dshot_bitbang.c \
            STM32/exti.c \
            STM32/io_stm32.c \
            STM32/light_ws2811strip_hal.c \
            STM32/persistent.c \
            STM32/pwm_output_dshot_hal.c \
            STM32/rcc_stm32.c \
            STM32/serial_uart_hal.c \
            STM32/serial_uart_stm32n6xx.c \
            STM32/system_stm32n6xx.c \
            STM32/timer_hal.c \
            STM32/timer_stm32n6xx.c \
            drivers/adc.c \
            drivers/serial_escserial.c \
            STM32/startup/system_stm32n6xx_s.c

MSC_SRC = \

SPEED_OPTIMISED_SRC += \

SIZE_OPTIMISED_SRC += \
            drivers/bus_i2c_timing.c \
            STM32/bus_i2c_hal_init.c \
            STM32/serial_usb_vcp.c \
            drivers/serial_escserial.c

DSP_LIB := $(LIB_MAIN_DIR)/STM32N6/Drivers/CMSIS/DSP
DEVICE_FLAGS += -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -DUNALIGNED_SUPPORT_DISABLE -DARM_MATH_CM55

include $(TARGET_PLATFORM_DIR)/mk/STM32_COMMON.mk
