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
# USE_VIRTUAL_* is already defined at that point. common_post.h also requires at
# least one ACC/GYRO driver. Replaced by real drivers as the IMU/bus bring-up lands.
DEVICE_FLAGS    += \
            -DUSE_VIRTUAL_ACC \
            -DUSE_VIRTUAL_GYRO \
            -DUSE_VIRTUAL_MAG \
            -DUSE_VIRTUAL_BARO

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
# Skeleton: only the platform stub shim (satisfies the linker contract that
# Betaflight's common code calls into). Real drivers are added incrementally.
MCU_COMMON_SRC  = \
            NAVQ95/platform_stubs.c \
            NAVQ95/system.c \
            NAVQ95/io_navq95.c

# No peripheral SDK sources yet.
DEVICE_STDPERIPH_SRC :=

# ── MCU_EXCLUDES: external-storage drivers not yet supported ──────────────────
# mk/source.mk unconditionally adds the flash-chip, SD card, FAT and USB MSC
# sources. These need a bus/DMA/timer layer that is not implemented yet, so they
# are excluded here (SRC := $(filter-out $(MCU_EXCLUDES), $(SRC)) in source.mk)
# without editing any core file. Remove entries as each subsystem is added.
MCU_EXCLUDES += \
            drivers/flash/flash.c \
            drivers/flash/flash_m25p16.c \
            drivers/flash/flash_mt29f.c \
            drivers/flash/flash_mx66uw1g45g.c \
            drivers/flash/flash_w25m.c \
            drivers/flash/flash_w25n.c \
            drivers/flash/flash_w25q128fv.c \
            io/flashfs.c \
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
