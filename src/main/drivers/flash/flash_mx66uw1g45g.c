/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Macronix MX66UW1G45G — 1Gbit (128 MiB) Octal NOR Flash.
 *
 * Phase 1: memory-mapped read only. The chip is expected to be configured by
 * the bootloader in 8-line OPI/DTR mode and memory-mapped at
 * MX66UW1G45G_MEMORY_MAPPED_BASE (default 0x70000000, OCTOSPI2/XSPI2 base).
 *
 * Because the chip is in OPI mode, it cannot respond to 1-line or 4-line
 * JEDEC RDID probes. Selection is done at build time via the
 * OCTOSPI_FLASH_CHIP=MX66UW1G45G make variable, which emits both
 * USE_FLASH_MX66UW1G45G (driver gating) and OCTOSPI_FLASH_CHIP_MX66UW1G45G
 * (dispatch marker — see flash.c:flashOctoSpiInit).
 *
 * Write/erase are stubbed in this phase; adding them requires 8-line OPI
 * primitives in the OCTOSPI bus driver and platform implementations.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_FLASH_MX66UW1G45G) && defined(USE_OCTOSPI)

#include "common/utils.h"

#include "drivers/flash/flash.h"
#include "drivers/flash/flash_impl.h"
#include "drivers/flash/flash_mx66uw1g45g.h"
#include "drivers/system.h"

// Geometry: 1Gbit / 128 MiB, 4 KiB sectors, 256 B pages.
#define MX66UW1G45G_PAGE_SIZE           256U
#define MX66UW1G45G_SECTOR_SIZE         4096U
#define MX66UW1G45G_PAGES_PER_SECTOR    (MX66UW1G45G_SECTOR_SIZE / MX66UW1G45G_PAGE_SIZE)
#define MX66UW1G45G_SECTORS             32768U   // 32768 * 4 KiB = 128 MiB

#ifndef MX66UW1G45G_MEMORY_MAPPED_BASE
#define MX66UW1G45G_MEMORY_MAPPED_BASE  0x70000000U
#endif

static const flashVTable_t mx66uw1g45g_vTable;

MMFLASH_CODE_NOINLINE bool mx66uw1g45g_identify(flashDevice_t *fdevice, uint32_t jedecID)
{
    if (jedecID != MX66UW1G45G_JEDEC_ID) {
        fdevice->geometry.sectors = 0;
        fdevice->geometry.pagesPerSector = 0;
        fdevice->geometry.sectorSize = 0;
        fdevice->geometry.totalSize = 0;
        return false;
    }

    fdevice->geometry.flashType = FLASH_TYPE_NOR;
    fdevice->geometry.sectors = MX66UW1G45G_SECTORS;
    fdevice->geometry.pagesPerSector = MX66UW1G45G_PAGES_PER_SECTOR;
    fdevice->geometry.pageSize = MX66UW1G45G_PAGE_SIZE;
    fdevice->geometry.sectorSize = MX66UW1G45G_SECTOR_SIZE;
    fdevice->geometry.totalSize = (uint32_t)MX66UW1G45G_SECTOR_SIZE * MX66UW1G45G_SECTORS;

    // Ring-mode log rate cap. MX66UW is a 128 MB Macronix OPI NOR used in
    // memory-mapped configurations on H7 targets; the eraseSector vtable entry
    // here actually calls failureMode() (memory-mapped boot flash, not designed
    // for runtime writes), so this driver isn't a real blackbox target. The cap
    // is set for consistency with other fast-NOR class drivers — matching the
    // m25p16 ≥ 16 MB tier (sustained ≈ 60 KB/s / 40 B-frame ≈ 1.5 kHz). If a
    // future use case wires this driver up to flashfs, the writer should still
    // observe the same actual sustained bandwidth ceiling.
    fdevice->geometry.maxSustainedLogRateHz = 1500;

    fdevice->vTable = &mx66uw1g45g_vTable;

    return true;
}

static void mx66uw1g45g_configure(flashDevice_t *fdevice, uint32_t configurationFlags)
{
    UNUSED(fdevice);
    UNUSED(configurationFlags);
    // Bootloader has configured OPI memory-mapped mode; nothing to do.
}

MMFLASH_CODE static bool mx66uw1g45g_isReady(flashDevice_t *fdevice)
{
    UNUSED(fdevice);
    return true;
}

MMFLASH_CODE static bool mx66uw1g45g_waitForReady(flashDevice_t *fdevice)
{
    UNUSED(fdevice);
    return true;
}

// Phase 1 is memory-mapped read only. Any write or erase attempt is a
// programming error in the caller (e.g. enabling USE_FLASHFS or
// CONFIG_IN_EXTERNAL_FLASH on a board that uses this chip) and is reported via
// failureMode rather than silently no-op'ing — silent stubs would let flashfs
// loop forever waiting for progress.

MMFLASH_CODE static void mx66uw1g45g_eraseSector(flashDevice_t *fdevice, uint32_t address)
{
    UNUSED(fdevice);
    UNUSED(address);
    failureMode(FAILURE_FLASH_WRITE_FAILED);
}

static void mx66uw1g45g_eraseCompletely(flashDevice_t *fdevice)
{
    UNUSED(fdevice);
    failureMode(FAILURE_FLASH_WRITE_FAILED);
}

MMFLASH_CODE static void mx66uw1g45g_pageProgramBegin(flashDevice_t *fdevice, uint32_t address, void (*callback)(uintptr_t arg))
{
    UNUSED(fdevice);
    UNUSED(address);
    UNUSED(callback);
    failureMode(FAILURE_FLASH_WRITE_FAILED);
}

MMFLASH_CODE static uint32_t mx66uw1g45g_pageProgramContinue(flashDevice_t *fdevice, uint8_t const **buffers, const uint32_t *bufferSizes, uint32_t bufferCount)
{
    UNUSED(fdevice);
    UNUSED(buffers);
    UNUSED(bufferSizes);
    UNUSED(bufferCount);
    failureMode(FAILURE_FLASH_WRITE_FAILED);
    return 0;
}

MMFLASH_CODE static void mx66uw1g45g_pageProgramFinish(flashDevice_t *fdevice)
{
    UNUSED(fdevice);
    failureMode(FAILURE_FLASH_WRITE_FAILED);
}

MMFLASH_CODE static void mx66uw1g45g_pageProgram(flashDevice_t *fdevice, uint32_t address, const uint8_t *data, uint32_t length, void (*callback)(uintptr_t arg))
{
    UNUSED(fdevice);
    UNUSED(address);
    UNUSED(data);
    UNUSED(length);
    UNUSED(callback);
    failureMode(FAILURE_FLASH_WRITE_FAILED);
}

MMFLASH_CODE static void mx66uw1g45g_flush(flashDevice_t *fdevice)
{
    UNUSED(fdevice);
}

MMFLASH_CODE static int mx66uw1g45g_readBytes(flashDevice_t *fdevice, uint32_t address, uint8_t *buffer, uint32_t length)
{
    if (address >= fdevice->geometry.totalSize) {
        return 0;
    }
    const uint32_t remaining = fdevice->geometry.totalSize - address;
    if (length > remaining) {
        length = remaining;
    }

    memcpy(buffer, (const void *)(uintptr_t)(MX66UW1G45G_MEMORY_MAPPED_BASE + address), length);

    return (int)length;
}

static const flashGeometry_t *mx66uw1g45g_getGeometry(flashDevice_t *fdevice)
{
    return &fdevice->geometry;
}

MMFLASH_DATA static const flashVTable_t mx66uw1g45g_vTable = {
    .configure = mx66uw1g45g_configure,
    .isReady = mx66uw1g45g_isReady,
    .waitForReady = mx66uw1g45g_waitForReady,
    .eraseSector = mx66uw1g45g_eraseSector,
    .eraseCompletely = mx66uw1g45g_eraseCompletely,
    .pageProgramBegin = mx66uw1g45g_pageProgramBegin,
    .pageProgramContinue = mx66uw1g45g_pageProgramContinue,
    .pageProgramFinish = mx66uw1g45g_pageProgramFinish,
    .pageProgram = mx66uw1g45g_pageProgram,
    .flush = mx66uw1g45g_flush,
    .readBytes = mx66uw1g45g_readBytes,
    .getGeometry = mx66uw1g45g_getGeometry,
};

#endif
