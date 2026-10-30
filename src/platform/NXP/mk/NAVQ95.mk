#
# NXP i.MX 95 Cortex-M7 (NavQ95) Make file include
#
# The top level Makefile adds $(MCU_COMMON_SRC) and $(DEVICE_STDPERIPH_SRC) to SRC collection.
#

PLATFORM_SDK := arm

ARCH_FLAGS      = -mthumb -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16

DEVICE_FLAGS    += \
            -DIMX95_M7 \
            -DNAVQ95 \
            -DARM_MATH_CM7 \
            -D__FPU_PRESENT=1 \
            -DUNALIGNED_SUPPORT_DISABLE \
            -DMAX_MPU_REGIONS=16

# These must be defined before common_pre.h, which platform.h includes ahead of
# target.h. Defining USE_BARO skips the default barometer list, which only has
# the SPI BMP5xx variants.
DEVICE_FLAGS    += \
            -DUSE_BARO \
            -DUSE_BARO_BMP581 \
            -DDEFAULT_BARO_BMP581 \
            -DDEFAULT_BARO_DEVICE=BARO_BMP581

DEVICE_FLAGS    += \
            -DUSE_SPI \
            -DUSE_SPI_DEVICE_1 \
            -DUSE_I2C

INCLUDE_DIRS    += \
            $(TARGET_PLATFORM_DIR) \
            $(TARGET_PLATFORM_DIR)/include \
            $(TARGET_PLATFORM_DIR)/include/platform \
            $(TARGET_PLATFORM_DIR)/startup \
            $(LIB_MAIN_DIR)/CMSIS/Core/Include

LD_SCRIPT       = $(LINKER_DIR)/navq95_flexspi.ld
STARTUP_SRC     = NXP/startup/startup_navq95.s

MCU_COMMON_SRC  = \
            NXP/platform_stubs.c \
            NXP/system.c \
            NXP/io_navq95.c \
            NXP/serial_uart_navq95.c \
            NXP/bus_spi_navq95.c \
            NXP/config_navq95.c \
            NXP/sdio_usdhc_navq95.c \
            NXP/pwm_navq95.c \
            NXP/flexspi_nor_imx95.c \
            NXP/config_flash_navq95.c \
            NXP/bus_i2c_navq95.c \
            drivers/bus_spi_config.c \
            drivers/serial_pinconfig.c

DEVICE_STDPERIPH_SRC :=

MCU_EXCLUDES += \
            drivers/sdcard_spi.c \
            msc/usbd_storage.c \
            msc/usbd_storage_emfat.c \
            msc/emfat.c \
            msc/emfat_file.c \
            msc/usbd_storage_sd_spi.c \
            msc/usbd_storage_sdio.c \
            drivers/usb_msc_common.c
