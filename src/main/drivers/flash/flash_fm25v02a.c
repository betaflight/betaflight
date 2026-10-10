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
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_FLASH_FM25V02A

#include "common/maths.h"

#include "drivers/bus_spi.h"
#include "drivers/flash/flash.h"
#include "drivers/flash/flash_fm25v02a.h"
#include "drivers/flash/flash_impl.h"

#define FM25V02A_INSTRUCTION_WRITE_ENABLE       0x06
#define FM25V02A_INSTRUCTION_READ_STATUS        0x05
#define FM25V02A_INSTRUCTION_WRITE_STATUS       0x01
#define FM25V02A_INSTRUCTION_READ               0x03
#define FM25V02A_INSTRUCTION_WRITE              0x02

#define FM25V02A_STATUS_BLOCK_PROTECT_MASK      0x0C
#define FM25V02A_STATUS_WRITE_PROTECT_ENABLE    0x80
#define FM25V02A_STATUS_PROTECTION_MASK         (FM25V02A_STATUS_BLOCK_PROTECT_MASK | FM25V02A_STATUS_WRITE_PROTECT_ENABLE)

// 25 MHz is valid over the complete 2.0 V to 3.6 V operating range.
#define FM25V02A_MAX_SPI_CLK_HZ                 25000000

#define FM25V02A_ERASE_CHUNK_SIZE               128
#define FM25V02A_ERASE_CHUNKS_PER_SECTOR        (FM25V02A_LOGICAL_SECTOR_SIZE / FM25V02A_ERASE_CHUNK_SIZE)

STATIC_ASSERT(FM25V02A_LOGICAL_SECTOR_SIZE % FM25V02A_ERASE_CHUNK_SIZE == 0,
    FM25V02A_erase_chunk_must_divide_logical_sector);

enum {
    FM25V02A_PROGRAM_WRITE_ENABLE,
    FM25V02A_PROGRAM_COMMAND,
    FM25V02A_PROGRAM_DATA1,
    FM25V02A_PROGRAM_DATA2,
    FM25V02A_PROGRAM_END,
};

static const flashVTable_t fm25v02a_vTable;

static void fm25v02a_setCommandAddress(uint8_t *command, uint32_t address)
{
    command[1] = (address >> 8) & 0x7F;
    command[2] = address & 0xFF;
}

static uint8_t fm25v02a_readStatus(flashDevice_t *fdevice)
{
    uint8_t command[2] = { FM25V02A_INSTRUCTION_READ_STATUS, 0 };
    uint8_t response[2] = { 0 };

    spiReadWriteBuf(fdevice->io.handle.dev, command, response, sizeof(command));

    return response[1];
}

static void fm25v02a_writeStatus(flashDevice_t *fdevice, uint8_t status)
{
    uint8_t writeEnable[] = { FM25V02A_INSTRUCTION_WRITE_ENABLE };
    uint8_t writeStatus[] = { FM25V02A_INSTRUCTION_WRITE_STATUS, status };
    busSegment_t segments[] = {
        { .u.buffers = { writeEnable, NULL }, sizeof(writeEnable), true, NULL },
        { .u.buffers = { writeStatus, NULL }, sizeof(writeStatus), true, NULL },
        { .u.link = { NULL, NULL }, 0, true, NULL },
    };

    spiSequence(fdevice->io.handle.dev, segments);
    spiWait(fdevice->io.handle.dev);
}

static bool fm25v02a_hasSupportedDeviceId(const uint8_t *deviceId, size_t deviceIdLength)
{
    static const uint8_t manufacturerId[] = { 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0xC2, 0x22 };

    if (deviceIdLength != FM25V02A_DEVICE_ID_LENGTH
        || memcmp(deviceId, manufacturerId, sizeof(manufacturerId)) != 0) {
        return false;
    }

    // 0x08 is the standard-temperature part and 0x48 the extended-temperature part.
    return deviceId[8] == 0x08 || deviceId[8] == 0x48;
}

bool fm25v02a_identify(flashDevice_t *fdevice, const uint8_t *deviceId, size_t deviceIdLength)
{
    flashGeometry_t *geometry = &fdevice->geometry;
    memset(geometry, 0, sizeof(*geometry));

    if (!deviceId || !fm25v02a_hasSupportedDeviceId(deviceId, deviceIdLength)) {
        return false;
    }

    /*
     * F-RAM has no physical erase sectors or program pages.  The logical
     * geometry keeps the partition and FlashFS APIs useful; the driver does
     * not impose the logical page boundary on writes and emulates erase by
     * writing 0xFF only to the requested logical sector.
     */
    geometry->sectors = FM25V02A_TOTAL_SIZE / FM25V02A_LOGICAL_SECTOR_SIZE;
    geometry->pageSize = FM25V02A_LOGICAL_PAGE_SIZE;
    geometry->sectorSize = FM25V02A_LOGICAL_SECTOR_SIZE;
    geometry->totalSize = FM25V02A_TOTAL_SIZE;
    geometry->pagesPerSector = FM25V02A_LOGICAL_SECTOR_SIZE / FM25V02A_LOGICAL_PAGE_SIZE;
    geometry->flashType = FLASH_TYPE_FRAM;
    geometry->jedecId = (0xC2U << 16) | (0x22U << 8) | deviceId[8];
    geometry->maxReadClkSPIHz = FM25V02A_MAX_SPI_CLK_HZ;

    fdevice->couldBeBusy = false;
    fdevice->isLargeFlash = false;
    fdevice->vTable = &fm25v02a_vTable;

    return true;
}

static void fm25v02a_configure(flashDevice_t *fdevice, uint32_t configurationFlags)
{
    UNUSED(configurationFlags);

    spiSetClkDivisor(fdevice->io.handle.dev, spiCalculateDivider(FM25V02A_MAX_SPI_CLK_HZ));

    /*
     * BP0, BP1 and WPEN are nonvolatile.  Clear all three when possible so the
     * status register remains maintainable.  WPEN only controls whether the
     * WP pin protects the status register; it does not protect the array.
     * Refuse to expose storage only when the array-protecting BP bits remain.
     */
    uint8_t status = fm25v02a_readStatus(fdevice);
    if (status & FM25V02A_STATUS_PROTECTION_MASK) {
        fm25v02a_writeStatus(fdevice, 0);
        status = fm25v02a_readStatus(fdevice);
        if (status & FM25V02A_STATUS_BLOCK_PROTECT_MASK) {
            fdevice->geometry.sectors = 0;
            fdevice->geometry.totalSize = 0;
        }
    }
}

static bool fm25v02a_isReady(flashDevice_t *fdevice)
{
    // F-RAM writes complete at bus speed; only an active SPI transfer can be busy.
    return !spiIsBusy(fdevice->io.handle.dev);
}

static bool fm25v02a_waitForReady(flashDevice_t *fdevice)
{
    spiWait(fdevice->io.handle.dev);
    return true;
}

// Called after the last SPI data segment has completed, potentially from ISR context.
static busStatus_e fm25v02a_programComplete(uintptr_t arg)
{
    flashDevice_t *fdevice = (flashDevice_t *)arg;

    fdevice->currentWriteAddress += fdevice->bytesWritten;
    if (fdevice->callback) {
        fdevice->callback(fdevice->bytesWritten);
    }

    return BUS_READY;
}

static void fm25v02a_pageProgramBegin(flashDevice_t *fdevice, uint32_t address, void (*callback)(uintptr_t arg))
{
    fdevice->callback = callback;
    fdevice->currentWriteAddress = address;
}

static uint32_t fm25v02a_pageProgramContinue(flashDevice_t *fdevice, uint8_t const **buffers,
    const uint32_t *bufferSizes, uint32_t bufferCount)
{
    if (!buffers || !bufferSizes || bufferCount == 0 || bufferCount > 2
        || fdevice->currentWriteAddress >= fdevice->geometry.totalSize) {
        return 0;
    }

    uint32_t remaining = fdevice->geometry.totalSize - fdevice->currentWriteAddress;
    const uint32_t firstSize = MIN(bufferSizes[0], remaining);
    remaining -= firstSize;
    const uint32_t secondSize = bufferCount == 2 ? MIN(bufferSizes[1], remaining) : 0;
    if (firstSize == 0 && secondSize == 0) {
        return 0;
    }

    // The segment list must outlive this non-blocking routine.
    STATIC_DMA_DATA_AUTO uint8_t writeEnable[] = { FM25V02A_INSTRUCTION_WRITE_ENABLE };
    STATIC_DMA_DATA_AUTO uint8_t writeCommand[3] = { FM25V02A_INSTRUCTION_WRITE };
    static busSegment_t segments[] = {
        { .u.buffers = { writeEnable, NULL }, sizeof(writeEnable), true, NULL },
        { .u.buffers = { writeCommand, NULL }, sizeof(writeCommand), false, NULL },
        { .u.link = { NULL, NULL }, 0, true, NULL },
        { .u.link = { NULL, NULL }, 0, true, NULL },
        { .u.link = { NULL, NULL }, 0, true, NULL },
    };

    spiWait(fdevice->io.handle.dev);

    fm25v02a_setCommandAddress(writeCommand, fdevice->currentWriteAddress);

    segments[FM25V02A_PROGRAM_DATA1].u.buffers.txData = (uint8_t *)(firstSize ? buffers[0] : buffers[1]);
    segments[FM25V02A_PROGRAM_DATA1].u.buffers.rxData = NULL;
    segments[FM25V02A_PROGRAM_DATA1].len = firstSize ? firstSize : secondSize;

    fdevice->bytesWritten = firstSize + secondSize;

    if (firstSize && secondSize) {
        segments[FM25V02A_PROGRAM_DATA1].negateCS = false;
        segments[FM25V02A_PROGRAM_DATA1].callback = NULL;
        segments[FM25V02A_PROGRAM_DATA2].u.buffers.txData = (uint8_t *)buffers[1];
        segments[FM25V02A_PROGRAM_DATA2].u.buffers.rxData = NULL;
        segments[FM25V02A_PROGRAM_DATA2].len = secondSize;
        segments[FM25V02A_PROGRAM_DATA2].negateCS = true;
        segments[FM25V02A_PROGRAM_DATA2].callback = fm25v02a_programComplete;
    } else {
        segments[FM25V02A_PROGRAM_DATA1].negateCS = true;
        segments[FM25V02A_PROGRAM_DATA1].callback = fm25v02a_programComplete;
        segments[FM25V02A_PROGRAM_DATA2].u.link.dev = NULL;
        segments[FM25V02A_PROGRAM_DATA2].u.link.segments = NULL;
        segments[FM25V02A_PROGRAM_DATA2].len = 0;
        segments[FM25V02A_PROGRAM_DATA2].negateCS = true;
        segments[FM25V02A_PROGRAM_DATA2].callback = NULL;
    }

    segments[FM25V02A_PROGRAM_END].u.link.dev = NULL;
    segments[FM25V02A_PROGRAM_END].u.link.segments = NULL;
    segments[FM25V02A_PROGRAM_END].len = 0;

    spiSequence(fdevice->io.handle.dev, segments);

    if (!fdevice->callback) {
        spiWait(fdevice->io.handle.dev);
    }

    return fdevice->bytesWritten;
}

static void fm25v02a_pageProgramFinish(flashDevice_t *fdevice)
{
    UNUSED(fdevice);
}

static void fm25v02a_pageProgram(flashDevice_t *fdevice, uint32_t address, const uint8_t *data,
    uint32_t length, void (*callback)(uintptr_t arg))
{
    fm25v02a_pageProgramBegin(fdevice, address, callback);
    fm25v02a_pageProgramContinue(fdevice, &data, &length, 1);
    fm25v02a_pageProgramFinish(fdevice);
}

static void fm25v02a_fillWithErasedValue(flashDevice_t *fdevice, uint32_t address, uint32_t length)
{
    if (address >= fdevice->geometry.totalSize) {
        return;
    }

    length = MIN(length, fdevice->geometry.totalSize - address);

    STATIC_DMA_DATA_AUTO uint8_t erasedData[FM25V02A_ERASE_CHUNK_SIZE];
    STATIC_DMA_DATA_AUTO uint8_t writeEnable[] = { FM25V02A_INSTRUCTION_WRITE_ENABLE };
    STATIC_DMA_DATA_AUTO uint8_t writeCommand[3] = { FM25V02A_INSTRUCTION_WRITE };
    busSegment_t segments[FM25V02A_ERASE_CHUNKS_PER_SECTOR + 3];

    memset(erasedData, 0xFF, sizeof(erasedData));
    memset(segments, 0, sizeof(segments));

    segments[0] = (busSegment_t){ .u.buffers = { writeEnable, NULL }, sizeof(writeEnable), true, NULL };
    segments[1] = (busSegment_t){ .u.buffers = { writeCommand, NULL }, sizeof(writeCommand), false, NULL };

    fm25v02a_setCommandAddress(writeCommand, address);

    const uint32_t chunkCount = (length + FM25V02A_ERASE_CHUNK_SIZE - 1) / FM25V02A_ERASE_CHUNK_SIZE;
    uint32_t bytesRemaining = length;
    for (uint32_t i = 0; i < chunkCount; i++) {
        const uint32_t chunkSize = MIN(bytesRemaining, (uint32_t)sizeof(erasedData));
        segments[i + 2] = (busSegment_t){
            .u.buffers = { erasedData, NULL },
            .len = chunkSize,
            .negateCS = (i + 1 == chunkCount),
            .callback = NULL,
        };
        bytesRemaining -= chunkSize;
    }
    segments[chunkCount + 2] = (busSegment_t){ .u.link = { NULL, NULL }, 0, true, NULL };

    spiWait(fdevice->io.handle.dev);
    spiSequence(fdevice->io.handle.dev, segments);
    spiWait(fdevice->io.handle.dev);
}

static void fm25v02a_eraseSector(flashDevice_t *fdevice, uint32_t address)
{
    address -= address % FM25V02A_LOGICAL_SECTOR_SIZE;
    fm25v02a_fillWithErasedValue(fdevice, address, FM25V02A_LOGICAL_SECTOR_SIZE);
}

static void fm25v02a_eraseCompletely(flashDevice_t *fdevice)
{
    for (uint32_t address = 0; address < fdevice->geometry.totalSize; address += FM25V02A_LOGICAL_SECTOR_SIZE) {
        fm25v02a_fillWithErasedValue(fdevice, address, FM25V02A_LOGICAL_SECTOR_SIZE);
    }
}

static int fm25v02a_readBytes(flashDevice_t *fdevice, uint32_t address, uint8_t *buffer, uint32_t length)
{
    if (!buffer || address >= fdevice->geometry.totalSize) {
        return 0;
    }

    length = MIN(length, fdevice->geometry.totalSize - address);
    if (length == 0) {
        return 0;
    }

    uint8_t readCommand[3] = { FM25V02A_INSTRUCTION_READ };
    busSegment_t segments[] = {
        { .u.buffers = { readCommand, NULL }, sizeof(readCommand), false, NULL },
        { .u.buffers = { NULL, buffer }, (int)length, true, NULL },
        { .u.link = { NULL, NULL }, 0, true, NULL },
    };

    fm25v02a_setCommandAddress(readCommand, address);

    spiWait(fdevice->io.handle.dev);
    spiSequence(fdevice->io.handle.dev, segments);
    spiWait(fdevice->io.handle.dev);

    return length;
}

static const flashGeometry_t *fm25v02a_getGeometry(flashDevice_t *fdevice)
{
    return &fdevice->geometry;
}

static const flashVTable_t fm25v02a_vTable = {
    .configure = fm25v02a_configure,
    .isReady = fm25v02a_isReady,
    .waitForReady = fm25v02a_waitForReady,
    .eraseSector = fm25v02a_eraseSector,
    .eraseCompletely = fm25v02a_eraseCompletely,
    .pageProgramBegin = fm25v02a_pageProgramBegin,
    .pageProgramContinue = fm25v02a_pageProgramContinue,
    .pageProgramFinish = fm25v02a_pageProgramFinish,
    .pageProgram = fm25v02a_pageProgram,
    .readBytes = fm25v02a_readBytes,
    .getGeometry = fm25v02a_getGeometry,
};

#endif // USE_FLASH_FM25V02A
