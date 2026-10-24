# ─────────────────────────────────────────────────────────────────────────────
# NAVQ95.mk — platform family makefile for the NXP i.MX 95 Cortex-M7 (NavQ95).
#
# Included by the top-level Makefile (via TARGET_MCU_FAMILY := NAVQ95). Sets the
# toolchain, architecture/device flags, startup, linker script and source list.
# Peripheral drivers (clocks, GPIO, SPI/I2C, UART, ...) are added incrementally.
# ─────────────────────────────────────────────────────────────────────────────

# ARM bare-metal toolchain (arm-none-eabi-), as other Cortex-M platforms.
PLATFORM_SDK := arm

# ── Toolchain / architecture ────────────────────────────────────────────────
# i.MX 95 real-time domain: Cortex-M7, double-precision FPU (fpv5-d16), hard-float.
ARCH_FLAGS      = -mthumb -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16

# ── MCU device flags ──────────────────────────────────────────────────────────
DEVICE_FLAGS    += \
            -DIMX95_M7 \
            -DNAVQ95 \
            -DARM_MATH_CM7 \
            -D__FPU_PRESENT=1 \
            -DUNALIGNED_SUPPORT_DISABLE \
            -DMAX_MPU_REGIONS=16

# Virtual sensors. These must be defined on the command line (not in target.h):
# platform.h includes target/common_pre.h before target.h, and common_pre.h's
# default-build block enables the full real baro/mag driver list unless
# USE_VIRTUAL_* / USE_BARO is already defined at that point.
# GYRO/ACC are REAL (ICM-45686 over LPSPI1). MAG is REAL (BMM350 on LPI2C1 @0x14,
# Phase 5): with USE_VIRTUAL_MAG removed and USE_MAG set, common_post.h enables
# the mag driver list (incl. USE_MAG_BMM350) and compass_bmm350.c (already in
# COMMON_SRC) is compiled. The mag bus/address default comes from
# MAG_I2C_INSTANCE / MAG_I2C_ADDRESS in target.h.
#
# BARO is now REAL too (BMP581 on LPI2C1 @0x46, Phase 6). The BMP581 I2C path is
# SPECIAL and must be forced on the COMMAND LINE (common_pre.h runs before
# target.h):
#  - common_pre.h's default baro list only enables the *SPI* BMP5xx variants
#    (USE_BARO_SPI_BMP580/581), NOT the I2C USE_BARO_BMP581, and its auto-default
#    selector (barometer.c) never picks BMP5xx (it prefers DPS310/BMP388/...).
#  - Defining USE_BARO here makes common_pre.h SKIP its whole default baro list
#    (`#if !defined(USE_BARO) ...`), so DPS310/etc. are NOT pulled in and cannot
#    win the default-bus selection.
#  - USE_BARO_BMP581 enables the I2C BMP5xx driver gate; DEFAULT_BARO_BMP581
#    makes pgResetFn_barometerConfig take the I2C bus branch; DEFAULT_BARO_DEVICE
#    selects BMP581 as the default baro hardware. Bus/addr come from
#    BARO_I2C_INSTANCE / BARO_I2C_ADDRESS in target.h.
DEVICE_FLAGS    += \
            -DUSE_BARO \
            -DUSE_BARO_BMP581 \
            -DDEFAULT_BARO_BMP581 \
            -DDEFAULT_BARO_DEVICE=BARO_BMP581

# Real sensor + bus: ICM-45686 6-axis IMU on LPSPI1 (polled register-level SPI).
# NOTE: do NOT pass -DUSE_ACCGYRO_ICM45686 here. common_pre.h's classic-build
# block (`#if !defined(USE_GYRO) && !defined(USE_ACC)`) already does a bare
# `#define USE_ACCGYRO_ICM45686` (and the full default IMU list) because our
# target defines neither USE_GYRO nor USE_ACC before that point. Adding it on the
# command line redefines the bare macro -> "redefined" under -Werror. The
# ICM45686 driver gate therefore comes for free; the driver file is already in
# COMMON_SRC unconditionally. Only the bus macros are set here.
# USE_SPI_GYRO + the GYRO_1_* device config live in target.h.
DEVICE_FLAGS    += \
            -DUSE_SPI \
            -DUSE_SPI_DEVICE_1

# ── I2C (Phase 1): two LPI2C master instances over the generic I2C core ──────
# Betaflight's common drivers/bus_i2c_*.c provide the high-level API and the
# config parameter group; the platform supplies the hardware hooks in
# NAVQ95/bus_i2c_navq95.c (register-level polled LPI2C, pads muxed + clock
# ungated over SCMI). USE_I2C enables the common I2C core; the per-instance
# USE_I2C_DEVICE_1 / USE_I2C_DEVICE_6 spellings live in target.h (Option-A
# numbering: I2CDEV_1 -> LPI2C1, I2CDEV_6 -> LPI2C6). NOT soft-I2C.
DEVICE_FLAGS    += \
            -DUSE_I2C

# ── Include directories ───────────────────────────────────────────────────────
INCLUDE_DIRS    += \
            $(TARGET_PLATFORM_DIR) \
            $(TARGET_PLATFORM_DIR)/include \
            $(TARGET_PLATFORM_DIR)/include/platform \
            $(TARGET_PLATFORM_DIR)/startup \
            $(LIB_MAIN_DIR)/CMSIS/Core/Include

# ── Linker script ─────────────────────────────────────────────────────────────
# XIP-from-FlexSPI1 layout: MCUboot 0x800 header, app vectors at app_base+0x800,
# code XIP from FlexSPI1 (alias 0x02020000, flashed to primary 0x28020000),
# writable data/stack in SRAM 0x20008000. See navq95_flexspi.ld header.
LD_SCRIPT       = $(LINKER_DIR)/navq95_flexspi.ld

# ── Startup ───────────────────────────────────────────────────────────────────
STARTUP_SRC     = NAVQ95/startup/startup_navq95.s

# ── Post-build: raw .bin, then (deferred) MCUboot signing ────────────────────
# Betaflight emits the ELF; we add a raw binary for the FlexSPI XIP image.
# Signing is intentionally deferred (a post-build wrapper that does NOT affect
# image content). When ready, set SIGNING_TOOL/SIGNING_ARGS in the environment
# (mirroring the PX4 nuttx build) and the signed image is produced from the .bin.
#   SIGNING_TOOL ?= <imgtool or sign script>
#   SIGNING_ARGS ?= sign --header-size 0x800 --align 4 --version 1.0.0 --slot-size <N> --key <key>
# For now the deliverable is the unsigned raw .bin: betaflight_NAVQ95.bin

# ── Platform source list ──────────────────────────────────────────────────────
# NAVQ95/serial_uart_navq95.c supplies the i.MX 95 platform half of the generic
# serial core (uartHardware[] table + hooks) for UART7 (MSP) and UART2 (SBUS RX).
# drivers/serial_pinconfig.c provides the serialPinConfig PG (SERIAL_TRAIT_PIN_CONFIG).
# drivers/bus_spi_config.c provides spiPreinit() (called unconditionally from
# fc/init.c under USE_SPI). It is NOT in mk/source.mk — each platform .mk adds it
# (see IMXRT.mk / STM32_COMMON.mk). The common SPI core drivers/bus_spi.c IS added
# by source.mk when USE_SPI is defined.
# NAVQ95/config_navq95.c implements targetConfiguration() (SBUS board defaults).
# NAVQ95/sdio_usdhc_navq95.c is the i.MX 95 uSDHC SD-card driver (USE_SDCARD_SDIO;
# provides the SD_* HAL that drivers/sdcard_sdio_baremetal.c calls; ADMA2 polled).
MCU_COMMON_SRC  = \
            NAVQ95/platform_stubs.c \
            NAVQ95/system.c \
            NAVQ95/io_navq95.c \
            NAVQ95/serial_uart_navq95.c \
            NAVQ95/config_navq95.c \
            NAVQ95/sdio_usdhc_navq95.c \
            NAVQ95/pwm_navq95.c \
            NAVQ95/flexspi_nor_imx95.c \
            NAVQ95/flexspi_nor_selftest.c \
            NAVQ95/config_flash_navq95.c \
            NAVQ95/bus_i2c_navq95.c \
            drivers/bus_spi_config.c \
            drivers/serial_pinconfig.c

# No peripheral SDK sources yet.
DEVICE_STDPERIPH_SRC :=

# ── Serial: use the generic drivers/serial_uart.c core (force-added to
# COMMON_SRC by mk/source.mk) plus the i.MX 95 platform layer in
# NAVQ95/serial_uart_navq95.c, which supplies the uartHardware[] table and the
# platform hooks (serialUART/uartReconfigure/uartIrqHandler/uartPinConfigure/
# uartEnableTxInterrupt/...). This mirrors the PICO port (serial_uart_pico.c +
# uart_hw.c). drivers/serial_uart_pinconfig.c is also force-added and maps the
# UART7_RX/TX_PIN tags into the serialPinConfig PG (SERIAL_TRAIT_PIN_CONFIG).

# ── MCU_EXCLUDES: external-storage drivers not yet supported ──────────────────
# mk/source.mk unconditionally adds the SD card, FAT and USB MSC sources. These
# need a bus/DMA layer that is not implemented yet, so they are excluded here
# (SRC := $(filter-out $(MCU_EXCLUDES), $(SRC)) in source.mk) without editing any
# core file.
#
# The flash-chip stack (flash.c + chip drivers + flashfs.c) is NOT excluded:
# USE_SPI (for the gyro) makes common_post.h auto-derive USE_FLASH_SPI +
# USE_FLASH_CHIP from the classic-build USE_FLASH_M25P16, which makes
# bus_spi_config.c's spiPreinit() call flashPreinit() — so flash.c must build or
# the link fails. With no flash chip on the bus, flashDeviceInit() just returns
# haveFlash=false.
#
# SD card: USE_SDCARD + USE_SDCARD_SDIO build the SDIO path — so sdcard.c,
# sdcard_sdio_baremetal.c (the SD_* HAL glue), sdcard_standard.c, and the
# AsyncFatFS stack (asyncfatfs.c + fat_standard.c) are NOT excluded. Only the
# SPI-mode SD backend (sdcard_spi.c) is excluded (USE_SDCARD_SPI is not defined).
# The platform SD_* HAL lives in NAVQ95/sdio_usdhc_navq95.c.
MCU_EXCLUDES += \
            drivers/sdcard_spi.c

# USB mass-storage (MSC_SRC) is likewise force-added and pulls in the same
# storage backends + USB stack.
MCU_EXCLUDES += \
            msc/usbd_storage.c \
            msc/usbd_storage_emfat.c \
            msc/emfat.c \
            msc/emfat_file.c \
            msc/usbd_storage_sd_spi.c \
            msc/usbd_storage_sdio.c \
            drivers/usb_msc_common.c

# Features are disabled by excluding their source files (above), not by
# predefining -DUSE_xxx=0: common_pre.h uses bare `#define USE_xxx` with no
# value, so a command-line -DUSE_xxx=0 collides under -Werror.
