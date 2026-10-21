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
# USE_VIRTUAL_* is already defined at that point.
# GYRO/ACC are now REAL (ICM-45686 over LPSPI1); only MAG/BARO stay virtual until
# their I2C bus is brought up.
DEVICE_FLAGS    += \
            -DUSE_VIRTUAL_MAG \
            -DUSE_VIRTUAL_BARO

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
MCU_COMMON_SRC  = \
            NAVQ95/platform_stubs.c \
            NAVQ95/system.c \
            NAVQ95/io_navq95.c \
            NAVQ95/serial_uart_navq95.c \
            NAVQ95/config_navq95.c \
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
# haveFlash=false. SD card stays excluded (USE_SDCARD_SPI is never defined, so the
# sdcard_preinit call is dead).
MCU_EXCLUDES += \
            drivers/sdcard.c \
            drivers/sdcard_spi.c \
            drivers/sdcard_sdio_baremetal.c \
            drivers/sdcard_standard.c \
            io/asyncfatfs/asyncfatfs.c \
            io/asyncfatfs/fat_standard.c

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
