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

# MCUboot only boots a signed image: the raw binary is wrapped with imgtool
# (hash only, 0x800 byte header). The slot size matches the linker script.
DEFAULT_OUTPUT     := bin
IMGTOOL            ?= imgtool
IMGTOOL_SLOT_SIZE  ?= 0x800000
IMGTOOL_VERSION    ?= 0.0.0
IMGTOOL_SIGN       := sign --header-size 0x800 --pad-header --slot-size $(IMGTOOL_SLOT_SIZE) \
                      --align 2 --version $(IMGTOOL_VERSION)
BIN_FROM_ELF_CMD    = $(OBJCOPY) -O binary $< $@.raw && \
    if ! command -v $(IMGTOOL) >/dev/null 2>&1; then \
        echo "$(IMGTOOL) not found: MCUboot only boots a signed image, install it with 'pip install imgtool' or set IMGTOOL=<path>" >&2; \
        exit 1; \
    fi && \
    $(IMGTOOL) $(IMGTOOL_SIGN) $@.raw $@

MCU_EXCLUDES += \
            drivers/sdcard_spi.c \
            msc/usbd_storage.c \
            msc/usbd_storage_emfat.c \
            msc/emfat.c \
            msc/emfat_file.c \
            msc/usbd_storage_sd_spi.c \
            msc/usbd_storage_sdio.c \
            drivers/usb_msc_common.c
