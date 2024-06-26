#
# AT32F4 Make file include
#

CMSIS_DIR      := $(ROOT)/lib/main/AT32F43x/cmsis
STDPERIPH_DIR   = $(ROOT)/lib/main/AT32F43x/drivers
MIDDLEWARES_DIR = $(ROOT)/lib/main/AT32F43x/middlewares
STDPERIPH_SRC   = $(wildcard $(STDPERIPH_DIR)/src/*.c) \
                  $(wildcard $(MIDDLEWARES_DIR)/usb_drivers/src/*.c) \
                  $(wildcard $(MIDDLEWARES_DIR)/usbd_class/msc/*.c)

EXCLUDES        = at32f435_437_dvp.c \
                  at32f435_437_can.c \
                  at32f435_437_xmc.c \
                  at32f435_437_emac

STARTUP_SRC     = at32/startup_at32f435_437.s
STDPERIPH_SRC   := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))

VPATH           := $(VPATH):$(ROOT)/lib/main/AT32F43x/cmsis/cm4/core_support:$(STDPERIPH_DIR)/src:$(STDPERIPH_DIR)/inc:$(SRC_DIR)/startup/at32

VCP_SRC =  \
            $(ROOT)/lib/main/AT32F43x/middlewares/usbd_class/cdc/cdc_class.c \
            $(ROOT)/lib/main/AT32F43x/middlewares/usbd_class/cdc/cdc_desc.c \
            drivers/usb_io.c

VCP_INCLUDES =     $(ROOT)/lib/main/AT32F43x/middlewares/usb_drivers/inc \
                   $(ROOT)/lib/main/AT32F43x/middlewares/usbd_class/cdc

DEVICE_STDPERIPH_SRC = $(STDPERIPH_SRC)

INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(SRC_DIR)/startup/at32 \
                   $(SRC_DIR)/drivers \
                   $(SRC_DIR)/drivers/mcu/at32 \
                   $(STDPERIPH_DIR)/inc \
                   $(CMSIS_DIR)/cm4/core_support \
                   $(CMSIS_DIR)/cm4 \
                   $(MIDDLEWARES_DIR)/i2c_application_library \
                   $(MIDDLEWARES_DIR)/usbd_class/msc \
                   $(VCP_INCLUDES)

ifeq ($(TARGET),AT32F435M)
LD_SCRIPT       = $(LINKER_DIR)/at32_flash_f43xm.ld
else
LD_SCRIPT       = $(LINKER_DIR)/at32_flash_f43xg.ld
endif

ARCH_FLAGS      = -std=c99  -mthumb -mcpu=cortex-m4 -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -Wdouble-promotion
DEVICE_FLAGS   += -DUSE_ATBSP_DRIVER -DAT32F43x -DHSE_VALUE=$(HSE_VALUE) -DAT32 -DUSE_OTG_HOST_MODE

MCU_COMMON_SRC = \
            startup/at32/at32f435_437_clock.c \
            startup/at32/system_at32f435_437.c \
            drivers/mcu/at32/adc_at32f43x.c \
            drivers/mcu/at32/bus_i2c_atbsp.c \
            drivers/mcu/at32/bus_i2c_atbsp_init.c \
            drivers/mcu/at32/bus_spi_at32bsp.c \
            drivers/mcu/at32/camera_control.c \
            drivers/mcu/at32/debug.c \
            drivers/mcu/at32/dma_at32f43x.c \
            drivers/mcu/at32/dma_reqmap_mcu.c \
            drivers/mcu/at32/dshot_bitbang.c \
            drivers/mcu/at32/dshot_bitbang_stdperiph.c \
            drivers/mcu/at32/exti_at32.c \
            drivers/mcu/at32/io_at32.c \
            drivers/mcu/at32/light_ws2811strip_at32f43x.c \
            drivers/mcu/at32/persistent_at32bsp.c \
            drivers/mcu/at32/pwm_output_at32bsp.c \
            drivers/mcu/at32/pwm_output_dshot.c \
            drivers/mcu/at32/rcc_at32.c \
            drivers/mcu/at32/serial_uart_at32bsp.c \
            drivers/mcu/at32/serial_uart_at32f43x.c \
            drivers/mcu/at32/serial_usb_vcp_at32f4.c \
            drivers/mcu/at32/system_at32f43x.c \
            drivers/mcu/at32/timer_at32bsp.c \
            drivers/mcu/at32/timer_at32f43x.c \
            drivers/mcu/at32/usb_msc_at32f43x.c \
            drivers/accgyro/accgyro_mpu.c \
            drivers/dshot_bitbang_decode.c \
            drivers/inverter.c \
            drivers/pwm_output_dshot_shared.c \
            $(MIDDLEWARES_DIR)/i2c_application_library/i2c_application.c \
            drivers/bus_i2c_timing.c \
            drivers/usb_msc_common.c \
            msc/usbd_storage.c \
            msc/usbd_storage_emfat.c \
            msc/emfat.c \
            msc/emfat_file.c

MCU_EXCLUDES =
