
PLATFORM_SDK := arm

#CMSIS
CMSIS_DIR      := $(LIB_MAIN_DIR)/UM324xx/Drivers/CMSIS
#STDPERIPH
STDPERIPH_DIR   = $(LIB_MAIN_DIR)/UM324xx/Drivers/UM324xx_HAL_Driver
STDPERIPH_SRC   = $(notdir $(wildcard $(STDPERIPH_DIR)/Src/*.c))
EXCLUDES        = \
                um324xx_hal_acmp.c \
                um324xx_hal_aes.c \
                um324xx_hal_aes_ex.c \
                um324xx_hal_can.c \
                um324xx_hal_canfd.c \
                um324xx_hal_cordic.c \
                um324xx_hal_crc.c \
                um324xx_hal_ctm.c \
                um324xx_hal_dac.c \
                um324xx_hal_emac.c \
                um324xx_hal_i2c.c \
                um324xx_hal_i2s.c \
                um324xx_hal_lptim.c \
                um324xx_hal_lpuart.c \
                um324xx_hal_iwdt.c \
                um324xx_hal_pcd.c \
                um324xx_hal_pcd_ex.c \
                um324xx_hal_rng.c \
                um324xx_hal_spi.c \
                um324xx_hal_uart.c \
                um324xx_hal_vref.c \
                um324xx_hal_usart.c \
                um324xx_hal_wwdt.c

STDPERIPH_SRC   := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))
VPATH       := $(VPATH):$(STDPERIPH_DIR)/Src


#USB
USBCHERY_DIR := $(ROOT)/lib/main/UM324xx/Middlewares/UM/CherryUSB-1.6.0

USBCORE_DIR = $(USBCHERY_DIR)/core
USBCORE_SRC = $(notdir $(wildcard $(USBCORE_DIR)/*.c))
EXCLUDES    =
USBCORE_SRC := $(filter-out ${EXCLUDES}, $(USBCORE_SRC))

USBCDC_DIR = $(USBCHERY_DIR)/class/cdc
USBCDC_SRC = $(notdir $(wildcard $(USBCDC_DIR)/*.c))
EXCLUDES   =
USBCDC_SRC := $(filter-out ${EXCLUDES}, $(USBCDC_SRC))

USBHID_DIR =
USBHID_SRC = 

USBMSC_DIR = $(USBCHERY_DIR)/class/msc
USBMSC_SRC = $(notdir $(wildcard $(USBMSC_DIR)/*.c))
EXCLUDES   = usbh_msc.c \
            msc_ram_template.c
USBMSC_SRC := $(filter-out ${EXCLUDES}, $(USBMSC_SRC))

USBPORT_DIR = $(USBCHERY_DIR)/port/um
USBPORT_SRC = $(notdir $(wildcard $(USBPORT_DIR)/*.c))
EXCLUDES   =
USBPORT_SRC := $(filter-out ${EXCLUDES}, $(USBPORT_SRC))

VPATH := $(VPATH):$(USBCDC_DIR):$(USBCORE_DIR):$(USBHID_DIR):$(USBMSC_DIR):$(USBPORT_DIR)

DEVICE_STDPERIPH_SRC := $(STDPERIPH_SRC) \
                        $(USBCORE_SRC) \
                        $(USBCDC_SRC) \
                        $(USBHID_SRC) \
                        $(USBMSC_SRC) \
                        $(USBPORT_SRC)

#CMSIS
VPATH := $(VPATH):$(LIB_MAIN_DIR)/UM324xx/Drivers/CMSIS/Device/UM/UM324xx

CMSIS_SRC       :=
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(TARGET_PLATFORM_DIR) \
                   $(TARGET_PLATFORM_DIR)/include \
                   $(TARGET_PLATFORM_DIR)/startup \
                   $(STDPERIPH_DIR)/Inc \
                   $(USBCHERY_DIR)/common \
                   $(USBCORE_DIR) \
                   $(USBCDC_DIR)  \
                   $(USBHID_DIR)  \
                   $(USBMSC_DIR)  \
                   $(USBPORT_DIR) \
                   $(CMSIS_DIR)/Include \
                   $(CMSIS_DIR)/Device/UM/UM324xx/Include \
                   $(LIB_MAIN_DIR)/CMSIS/Core/Include \
                   $(TARGET_PLATFORM_DIR)/vcp_hal \
                   $(PLATFORM_DIR)/common/stm32

#Flags
ARCH_FLAGS      = -mthumb -mcpu=cortex-m4 -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16

DEVICE_FLAGS    = -DUSE_HAL_DRIVER -DUSE_FULL_LL_DRIVER

ifeq ($(TARGET_MCU),UM324xF)
DEVICE_FLAGS    += -DUM324xF
LD_SCRIPT       = $(LINKER_DIR)/um32_flash_4xf.ld
STARTUP_SRC     = UM32/startup/startup_um324xf.s
MCU_FLASH_SIZE  = 1024
# Override the OPTIMISE_SIZE.
else
$(error Unknown MCU for target)
endif

DEVICE_FLAGS    += -DHSE_VALUE=$(HSE_VALUE)

VCP_SRC = \
            UM32/vcp_hal/cdc_acm.c \
            UM32/serial_usb_vcp.c \
            drivers/usb_io.c

MCU_COMMON_SRC = \
            UM32/rcc_um32.c \
            UM32/io_um32.c \
            UM32/exti.c \
            UM32/debug.c \
            UM32/persistent.c \
            UM32/adc_um324xx.c \
            UM32/bus_i2c_hal.c \
            UM32/bus_i2c_um324xx.c \
            UM32/bus_spi_hal.c \
            UM32/bus_quadspi_hal.c \
            UM32/dma_reqmap_mcu.c \
            UM32/dma_um324xx.c \
            UM32/serial_uart_hal.c \
            UM32/serial_uart_um324xx.c \
            UM32/timer_hal.c \
            UM32/timer_um324xx.c \
            UM32/dshot_bitbang.c \
            UM32/dshot_bitbang_hal.c \
            UM32/pwm_output_dshot_hal.c \
            UM32/pwm_output_hw.c \
            UM32/light_ws2811strip_hal.c \
            UM32/transponder_ir_io_hal.c \
            UM32/camera_control_um32.c \
            UM32/exflash_um324xx_hal.c \
            UM32/sysfunc_um324xx.c \
            UM32/startup/system_um324xx.c \
            drivers/accgyro/accgyro_mpu.c \
            drivers/dshot_bitbang_decode.c \
            drivers/inverter.c \
            drivers/bus_i2c_timing.c \
            drivers/usb_msc_common.c \
            drivers/adc.c \
            drivers/bus_spi_config.c \
            drivers/serial_escserial.c \
            drivers/serial_pinconfig.c \
            common/stm32/system.c \
            common/stm32/io_impl.c \
            common/stm32/config_flash.c \
            common/stm32/mco.c \
            common/stm32/rx_pwm_hw.c \
            common/stm32/pwm_output_beeper.c \
            common/stm32/pwm_output_dshot_shared.c \
            common/stm32/dshot_dpwm.c \
            common/stm32/dshot_bitbang_shared.c \
            common/stm32/bus_i2c_pinconfig.c \
            common/stm32/bus_spi_pinconfig.c \
            common/stm32/bus_spi_hw.c \
            common/stm32/camera_control.c \
            common/stm32/serial_uart_hw.c \
            common/stm32/serial_uart_pinconfig.c \
            common/stm32/ledstrip_ws2811_stm32.c \
            common/stm32/debug_pin.c \
            common/stm32/adc_impl.c \
            common/stm32/expresslrs_driver_hw.c \
            common/stm32/fault_handlers.c

MSC_SRC = \
            UM32/usb_msc_hal.c \
            drivers/usb_msc_common.c \
            msc/usbd_storage.c \
            msc/usbd_storage_emfat.c \
            msc/emfat.c \
            msc/emfat_file.c \
            msc/usbd_storage_sdio.c \
            msc/usbd_storage_sd_spi.c

SIZE_OPTIMISED_SRC += \
            UM32/serial_usb_vcp.c \
            drivers/inverter.c \
            drivers/bus_spi_config.c \
            drivers/serial_escserial.c \
            drivers/serial_pinconfig.c \
            common/stm32/bus_i2c_pinconfig.c \
            common/stm32/config_flash.c \
            common/stm32/bus_spi_pinconfig.c \
            common/stm32/pwm_output_beeper.c \
            common/stm32/pwm_output_dshot_shared.c \
            common/stm32/serial_uart_pinconfig.c \
            msp/msp.c \
            telemetry/mavlink.c \
            io/gps.c \
            blackbox/blackbox.c \
            io/ledstrip.c \
            cms/cms_menu_imu.c \
            telemetry/crsf.c \
            config/config.c

DSP_LIB := $(LIB_MAIN_DIR)/CMSIS/DSP
DEVICE_FLAGS += -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -DUNALIGNED_SUPPORT_DISABLE -DARM_MATH_CM4
