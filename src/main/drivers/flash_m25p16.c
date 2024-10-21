/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "build/debug.h"

#ifdef USE_FLASH_M25P16

#include "drivers/bus_spi.h"
#include "drivers/bus_quadspi.h"
#include "drivers/flash.h"
#include "drivers/flash_impl.h"
#include "drivers/io.h"
#include "drivers/time.h"

#include "pg/flash.h"

#include "flash_m25p16.h"

#define M25P16_INSTRUCTION_RDID             SPIFLASH_INSTRUCTION_RDID
#define M25P16_INSTRUCTION_READ_BYTES       0x03
#define M25P16_INSTRUCTION_QUAD_READ        0x6B
#define M25P16_INSTRUCTION_READ_STATUS_REG  0x05
#define M25P16_INSTRUCTION_WRITE_STATUS_REG 0x01
#define M25P16_INSTRUCTION_WRITE_ENABLE     0x06
#define M25P16_INSTRUCTION_WRITE_DISABLE    0x04
#define M25P16_INSTRUCTION_PAGE_PROGRAM     0x02
#define M25P16_INSTRUCTION_QPAGE_PROGRAM    0x32
#define M25P16_INSTRUCTION_SECTOR_ERASE     0xD8
#define M25P16_INSTRUCTION_BULK_ERASE       0xC7

#define M25P16_STATUS_FLAG_WRITE_IN_PROGRESS 0x01
#define M25P16_STATUS_FLAG_WRITE_ENABLED     0x02

#define W25Q256_INSTRUCTION_ENTER_4BYTE_ADDRESS_MODE 0xB7

#define M25P16_FAST_READ_DUMMY_CYCLES       8

// SPI transaction segment indicies for m25p16_pageProgramContinue()
enum {READ_STATUS, WRITE_ENABLE, PAGE_PROGRAM, DATA1, DATA2};

static uint32_t maxClkSPIHz;
static uint32_t maxReadClkSPIHz;

// Table of recognised FLASH devices
struct {
    uint32_t        jedecID;
    uint16_t        maxClkSPIMHz;
    uint16_t        maxReadClkSPIMHz;
    flashSector_t   sectors;
    uint16_t        pagesPerSector;
} m25p16FlashConfig[] = {
    // Macronix MX25L3206E
    // Datasheet: https://docs.rs-online.com/5c85/0900766b814ac6f9.pdf
    { 0xC22016, 86, 33, 64, 256 },
    // Macronix MX25L6406E
    // Datasheet: https://www.macronix.com/Lists/Datasheet/Attachments/7370/MX25L6406E,%203V,%2064Mb,%20v1.9.pdf
    { 0xC22017, 86, 33, 128, 256 },
    // Macronix MX25L25635E
    // Datasheet: https://www.macronix.com/Lists/Datasheet/Attachments/7331/MX25L25635E,%203V,%20256Mb,%20v1.3.pdf
    { 0xC22019, 80, 50, 512, 256 },
    // Micron M25P16
    // Datasheet: https://www.micron.com/-/media/client/global/documents/products/data-sheet/nor-flash/serial-nor/m25p/m25p16.pdf
    { 0x202015, 25, 20, 32, 256 },
    // Micron N25Q064
    // Datasheet: https://www.micron.com/-/media/client/global/documents/products/data-sheet/nor-flash/serial-nor/n25q/n25q_64a_3v_65nm.pdf
    { 0x20BA17, 108, 54, 128, 256 },
    // Micron N25Q128
    // Datasheet: https://www.micron.com/-/media/client/global/documents/products/data-sheet/nor-flash/serial-nor/n25q/n25q_128mb_1_8v_65nm.pdf
    { 0x20ba18, 108, 54, 256, 256 },
    // Winbond W25Q80
    // Datasheet: https://www.winbond.com/resource-files/w25q80dv%20dl_revh_10022015.pdf
    { 0xEF4014, 104, 50, 16, 256 },
    // Winbond W25Q16
    // Datasheet: https://www.winbond.com/resource-files/w25q16dv_revi_nov1714_web.pdf
    { 0xEF4015, 104, 50, 32, 256 },
    // Winbond W25X32
    // Datasheet: https://www.winbond.com/resource-files/w25x32a_revb_080709.pdf
    { 0xEF3016, 133, 50, 64, 256 },
    // Winbond W25Q32
    // Datasheet: https://www.winbond.com/resource-files/w25q32jv%20dtr%20revf%2002242017.pdf?__locale=zh_TW
    { 0xEF4016, 133, 50, 64, 256 },
    // Winbond W25Q64
    // Datasheet: https://www.winbond.com/resource-files/w25q64jv%20spi%20%20%20revc%2006032016%20kms.pdf
    { 0xEF4017, 133, 50, 128, 256 }, // W25Q64JV-IQ/JQ
    { 0xEF7017, 133, 50, 128, 256 }, // W25Q64JV-IM/JM*
    // Winbond W25Q128
    // Datasheet: https://www.winbond.com/resource-files/w25q128fv%20rev.l%2008242015.pdf
    { 0xEF4018, 104, 50, 256, 256 },
    // PUYA PY25Q128
    // Datasheet: https://www.puyasemi.com/download_path/%E6%95%B0%E6%8D%AE%E6%89%8B%E5%86%8C/Flash%20%E8%8A%AF%E7%89%87/PY25F128HA_datasheet_V1.1.pdf
    { 0x852018, 133, 80, 256, 256 },
    // Zbit ZB25VQ128
    // Datasheet: http://zbitsemi.com/upload/file/20201010/20201010174048_82182.pdf
    { 0x5E4018, 104, 50, 256, 256 },
    // Winbond W25Q128_DTR
    // Datasheet: https://www.winbond.com/resource-files/w25q128jv%20dtr%20revb%2011042016.pdf
    { 0xEF7018, 66, 50, 256, 256 },
    // Winbond W25Q256
    // Datasheet: https://www.winbond.com/resource-files/w25q256jv%20spi%20revb%2009202016.pdf
    { 0xEF4019, 133, 50, 512, 256 },
    // Cypress S25FL064L
    // Datasheet: https://www.cypress.com/file/316661/download
    { 0x016017, 133, 50, 128, 256 },
    // Cypress S25FL128L
    // Datasheet: https://www.cypress.com/file/316171/download
    { 0x016018, 133, 50, 256, 256 },
    // BergMicro W25Q32
    // Datasheet: https://www.winbond.com/resource-files/w25q32jv%20dtr%20revf%2002242017.pdf?__locale=zh_TW
    { 0xE04016, 133, 50, 1024, 16 },
    // End of list
    { 0x000000, 0, 0, 0, 0 }
};

#define M25P16_PAGESIZE 256

STATIC_ASSERT(M25P16_PAGESIZE < FLASH_MAX_PAGE_SIZE, M25P16_PAGESIZE_too_small);

const flashVTable_t m25p16_vTable;

#ifdef USE_QUADSPI
const flashVTable_t m25p16Qspi_vTable;
// Buffer needed for assembling flash page when writing
static uint8_t m25p16_page_buffer[M25P16_PAGESIZE];
#endif /* USE_QUADSPI */

static uint8_t m25p16_readStatus(flashDevice_t *fdevice)
{
    uint8_t status = 0;
    if (fdevice->io.mode == FLASHIO_SPI) {
        STATIC_DMA_DATA_AUTO uint8_t readStatus[2] = { M25P16_INSTRUCTION_READ_STATUS_REG, 0 };
        STATIC_DMA_DATA_AUTO uint8_t readyStatus[2];

        spiReadWriteBuf(fdevice->io.handle.dev, readStatus, readyStatus, sizeof(readStatus));

        status = readyStatus[1];
    } else {
#ifdef USE_QUADSPI
        if (fdevice->io.mode == FLASHIO_QUADSPI) {
            quadSpiReceive1LINE(fdevice->io.handle.quadSpi, M25P16_INSTRUCTION_READ_STATUS_REG, 0, &status, 1);
        }
#endif
    }

    return status;
}

static bool m25p16_isReady(flashDevice_t *fdevice)
{
    // If we're waiting on DMA completion, then SPI is busy
    if (fdevice->io.mode == FLASHIO_SPI) {
        if (fdevice->io.handle.dev->bus->useDMA && spiIsBusy(fdevice->io.handle.dev)) {
            return false;
        }
    }

    // If couldBeBusy is false, don't bother to poll the flash chip for its status
    if (!fdevice->couldBeBusy) {
        return true;
    }

    // Poll the FLASH device to see if it's busy
    fdevice->couldBeBusy = ((m25p16_readStatus(fdevice) & M25P16_STATUS_FLAG_WRITE_IN_PROGRESS) != 0);

    return !fdevice->couldBeBusy;
}

static bool m25p16_waitForReady(flashDevice_t *fdevice)
{
    while (!m25p16_isReady(fdevice));

    return true;
}

/**
 * Read chip identification and geometry information (into global `geometry`).
 *
 * Returns true if we get valid ident, false if something bad happened like there is no M25P16.
 */
bool m25p16_identify(flashDevice_t *fdevice, uint32_t jedecID)
{
    flashGeometry_t *geometry = &fdevice->geometry;
    uint8_t index;

    for (index = 0; m25p16FlashConfig[index].jedecID; index++) {
        if (m25p16FlashConfig[index].jedecID == jedecID) {
            maxClkSPIHz = m25p16FlashConfig[index].maxClkSPIMHz * 1000000;
            maxReadClkSPIHz = m25p16FlashConfig[index].maxReadClkSPIMHz * 1000000;
            geometry->sectors = m25p16FlashConfig[index].sectors;
            geometry->pagesPerSector = m25p16FlashConfig[index].pagesPerSector;
            break;
        }
    }

    if (m25p16FlashConfig[index].jedecID == 0) {
        // Unsupported chip or not an SPI NOR flash
        geometry->sectors = 0;
        geometry->pagesPerSector = 0;
        geometry->sectorSize = 0;
        geometry->totalSize = 0;
        return false;
    }

    geometry->flashType = FLASH_TYPE_NOR;
    geometry->pageSize = M25P16_PAGESIZE;
    geometry->sectorSize = geometry->pagesPerSector * geometry->pageSize;
    geometry->totalSize = geometry->sectorSize * geometry->sectors;

    fdevice->couldBeBusy = true; // Just for luck we'll assume the chip could be busy even though it isn't specced to be
    fdevice->couldBeBusy = true; // Just for luck we'll assume the chip could be busy even though it isn't specced to be

    if (fdevice->io.mode == FLASHIO_SPI) {
        fdevice->vTable = &m25p16_vTable;
    }
#ifdef USE_QUADSPI
    else if (fdevice->io.mode == FLASHIO_QUADSPI) {
        fdevice->vTable = &m25p16Qspi_vTable;
    }
#endif
    return true;
}

void m25p16_configure(flashDevice_t *fdevice, uint32_t configurationFlags)
{
    if (configurationFlags & FLASH_CF_SYSTEM_IS_MEMORY_MAPPED) {
        return;
    }

    if (fdevice->io.mode == FLASHIO_SPI) {
        // Adjust the SPI bus clock frequency
        spiSetClkDivisor(fdevice->io.handle.dev, spiCalculateDivider(maxReadClkSPIHz));
    }

    flashGeometry_t *geometry = &fdevice->geometry;
    if (geometry->totalSize > 16 * 1024 * 1024) {
        fdevice->isLargeFlash = true;

        // This routine blocks so no need to use static data
        uint8_t modeSet[] = { W25Q256_INSTRUCTION_ENTER_4BYTE_ADDRESS_MODE };

        if (fdevice->io.mode == FLASHIO_SPI) {
            spiReadWriteBuf(fdevice->io.handle.dev, modeSet, NULL, sizeof(modeSet));
        }
#ifdef USE_QUADSPI
        else if (fdevice->io.mode == FLASHIO_QUADSPI) {
            quadSpiTransmit1LINE(fdevice->io.handle.quadSpi, W25Q256_INSTRUCTION_ENTER_4BYTE_ADDRESS_MODE, 0, NULL, 0);
        }
#endif
    }
}


static void m25p16_setCommandAddress(uint8_t *buf, uint32_t address, bool useLongAddress)
{
    if (useLongAddress) {
        *buf++ = (address >> 24) & 0xff;
    }
    *buf++ = (address >> 16) & 0xff;
    *buf++ = (address >> 8) & 0xff;
    *buf = address & 0xff;
}

// Called in ISR context
// A write enable has just been issued
busStatus_e m25p16_callbackWriteEnable(uint32_t arg)
{
    flashDevice_t *fdevice = (flashDevice_t *)arg;

    // As a write has just occurred, the device could be busy
    fdevice->couldBeBusy = true;

    return BUS_READY;
}

// Called in ISR context
// Write operation has just completed
busStatus_e m25p16_callbackWriteComplete(uint32_t arg)
{
    flashDevice_t *fdevice = (flashDevice_t *)arg;

    fdevice->currentWriteAddress += fdevice->callbackArg;

    // Call transfer completion callback
    if (fdevice->callback) {
        fdevice->callback(fdevice->callbackArg);
    }

    return BUS_READY;
}

// Called in ISR context
// Check if the status was busy and if so repeat the poll
busStatus_e m25p16_callbackReady(uint32_t arg)
{
    flashDevice_t *fdevice = (flashDevice_t *)arg;
    extDevice_t *dev = fdevice->io.handle.dev;

    uint8_t readyPoll = dev->bus->curSegment->u.buffers.rxData[1];

    if (readyPoll & M25P16_STATUS_FLAG_WRITE_IN_PROGRESS) {
        return BUS_BUSY;
    }

    // Bus is now known not to be busy
    fdevice->couldBeBusy = false;

    return BUS_READY;
}


/**
 * Erase a sector full of bytes to all 1's at the given byte offset in the flash chip.
 */
static void m25p16_eraseSector(flashDevice_t *fdevice, uint32_t address)
{
    STATIC_DMA_DATA_AUTO uint8_t sectorErase[5] = { M25P16_INSTRUCTION_SECTOR_ERASE };
    STATIC_DMA_DATA_AUTO uint8_t readStatus[2] = { M25P16_INSTRUCTION_READ_STATUS_REG, 0 };
    STATIC_DMA_DATA_AUTO uint8_t readyStatus[2];
    STATIC_DMA_DATA_AUTO uint8_t writeEnable[] = { M25P16_INSTRUCTION_WRITE_ENABLE };

    busSegment_t segments[] = {
            {.u.buffers = {readStatus, readyStatus}, sizeof(readStatus), true, m25p16_callbackReady},
            {.u.buffers = {writeEnable, NULL}, sizeof(writeEnable), true, m25p16_callbackWriteEnable},
            {.u.buffers = {sectorErase, NULL}, fdevice->isLargeFlash ? 5 : 4, true, NULL},
            {.u.link = {NULL, NULL}, 0, true, NULL},
    };

    // Ensure any prior DMA has completed before continuing
    spiWait(fdevice->io.handle.dev);

    m25p16_setCommandAddress(&sectorErase[1], address, fdevice->isLargeFlash);

    spiSequence(fdevice->io.handle.dev, segments);

    // Block pending completion of SPI access, but the erase will be ongoing
    spiWait(fdevice->io.handle.dev);
}

#ifdef USE_QUADSPI
static void m25p16_eraseSectorQspi(flashDevice_t *fdevice, uint32_t address)
{
    m25p16_waitForReady(fdevice);

    quadSpiTransmit1LINE(fdevice->io.handle.quadSpi, M25P16_INSTRUCTION_WRITE_ENABLE, 0, NULL, 0);
    quadSpiInstructionWithAddress1LINE(fdevice->io.handle.quadSpi, M25P16_INSTRUCTION_SECTOR_ERASE, 0, address, fdevice->isLargeFlash ? 32 : 24);

    fdevice->couldBeBusy = true;
}
#endif

static void m25p16_eraseCompletely(flashDevice_t *fdevice)
{
    STATIC_DMA_DATA_AUTO uint8_t readStatus[2] = { M25P16_INSTRUCTION_READ_STATUS_REG, 0 };
    STATIC_DMA_DATA_AUTO uint8_t readyStatus[2];
    STATIC_DMA_DATA_AUTO uint8_t writeEnable[] = { M25P16_INSTRUCTION_WRITE_ENABLE };
    STATIC_DMA_DATA_AUTO uint8_t bulkErase[] = { M25P16_INSTRUCTION_BULK_ERASE };

    busSegment_t segments[] = {
            {.u.buffers = {readStatus, readyStatus}, sizeof(readStatus), true, m25p16_callbackReady},
            {.u.buffers = {writeEnable, NULL}, sizeof(writeEnable), true, m25p16_callbackWriteEnable},
            {.u.buffers = {bulkErase, NULL}, sizeof(bulkErase), true, NULL},
            {.u.link = {NULL, NULL}, 0, true, NULL},
    };

    spiSequence(fdevice->io.handle.dev, segments);

    // Block pending completion of SPI access, but the erase will be ongoing
    spiWait(fdevice->io.handle.dev);
}

#ifdef USE_QUADSPI
static void m25p16_eraseCompletelyQspi(flashDevice_t *fdevice)
{
    m25p16_waitForReady(fdevice);

    quadSpiTransmit1LINE(fdevice->io.handle.quadSpi, M25P16_INSTRUCTION_WRITE_ENABLE, 0, NULL, 0);
    quadSpiTransmit1LINE(fdevice->io.handle.quadSpi, M25P16_INSTRUCTION_BULK_ERASE, 0, NULL, 0);

    fdevice->couldBeBusy = true;
}
#endif

static void m25p16_pageProgramBegin(flashDevice_t *fdevice, uint32_t address, void (*callback)(uint32_t length))
{
    fdevice->callback = callback;
    fdevice->currentWriteAddress = address;
}


static uint32_t m25p16_pageProgramContinue(flashDevice_t *fdevice, uint8_t const **buffers, const uint32_t *bufferSizes, uint32_t bufferCount)
{
    // The segment list cannot be in automatic storage as this routine is non-blocking
    STATIC_DMA_DATA_AUTO uint8_t readStatus[2] = { M25P16_INSTRUCTION_READ_STATUS_REG, 0 };
    STATIC_DMA_DATA_AUTO uint8_t readyStatus[2];
    STATIC_DMA_DATA_AUTO uint8_t writeEnable[] = { M25P16_INSTRUCTION_WRITE_ENABLE };
    STATIC_DMA_DATA_AUTO uint8_t pageProgram[5] = { M25P16_INSTRUCTION_PAGE_PROGRAM };

    static busSegment_t segments[] = {
            {.u.buffers = {readStatus, readyStatus}, sizeof(readStatus), true, m25p16_callbackReady},
            {.u.buffers = {writeEnable, NULL}, sizeof(writeEnable), true, m25p16_callbackWriteEnable},
            {.u.buffers = {pageProgram, NULL}, 0, false, NULL},
            {.u.link = {NULL, NULL}, 0, true, NULL},
            {.u.link = {NULL, NULL}, 0, true, NULL},
            {.u.link = {NULL, NULL}, 0, true, NULL},
    };

    // Ensure any prior DMA has completed before continuing
    spiWait(fdevice->io.handle.dev);

    // Patch the pageProgram segment
    segments[PAGE_PROGRAM].len = fdevice->isLargeFlash ? 5 : 4;
    m25p16_setCommandAddress(&pageProgram[1], fdevice->currentWriteAddress, fdevice->isLargeFlash);

    // Patch the data segments
    segments[DATA1].u.buffers.txData = (uint8_t *)buffers[0];
    segments[DATA1].len = bufferSizes[0];
    fdevice->callbackArg = bufferSizes[0];

    /* As the DATA2 segment may be used as the terminating segment, the rxData and txData may be overwritten
     * with a link to the following transaction (u.link.dev and u.link.segments respectively) so ensure that
     * rxData is reinitialised otherwise it will remain pointing at a chained u.link.segments structure which
     * would result in it being corrupted.
     */
    segments[DATA2].u.buffers.rxData = (uint8_t *)NULL;

    if (bufferCount == 1) {
        segments[DATA1].negateCS = true;
        segments[DATA1].callback = m25p16_callbackWriteComplete;
        // Mark segment following data as being of zero length
        segments[DATA2].u.buffers.txData = (uint8_t *)NULL;
        segments[DATA2].len = 0;
    } else if (bufferCount == 2) {
        segments[DATA1].negateCS = false;
        segments[DATA1].callback = NULL;
        segments[DATA2].u.buffers.txData = (uint8_t *)buffers[1];
        segments[DATA2].len = bufferSizes[1];
        fdevice->callbackArg += bufferSizes[1];
        segments[DATA2].negateCS = true;
        segments[DATA2].callback = m25p16_callbackWriteComplete;
    } else {
        return 0;
    }

    spiSequence(fdevice->io.handle.dev, fdevice->couldBeBusy ? &segments[READ_STATUS] : &segments[WRITE_ENABLE]);

    if (fdevice->callback == NULL) {
        // No callback was provided so block
        spiWait(fdevice->io.handle.dev);
    }

    return fdevice->callbackArg;
}

static void m25p16_pageProgramFinish(flashDevice_t *fdevice)
{
    UNUSED(fdevice);
}

/**
 * Write bytes to a flash page. Address must not cross a page boundary.
 *
 * Bits can only be set to zero, not from zero back to one again. In order to set bits to 1, use the erase command.
 *
 * Length must be smaller than the page size.
 *
 * This will wait for the flash to become ready before writing begins.
 *
 * Datasheet indicates typical programming time is 0.8ms for 256 bytes, 0.2ms for 64 bytes, 0.05ms for 16 bytes.
 * (Although the maximum possible write time is noted as 5ms).
 *
 * If you want to write multiple buffers (whose sum of sizes is still not more than the page size) then you can
 * break this operation up into one beginProgram call, one or more continueProgram calls, and one finishProgram call.
 */
static void m25p16_pageProgram(flashDevice_t *fdevice, uint32_t address, const uint8_t *data, uint32_t length, void (*callback)(uint32_t length))
{
    m25p16_pageProgramBegin(fdevice, address, callback);

    m25p16_pageProgramContinue(fdevice, &data, &length, 1);

    m25p16_pageProgramFinish(fdevice);
}

#ifdef USE_QUADSPI
// Page programming QSPI mode

static uint32_t m25p16_pageProgramContinueQspi(flashDevice_t *fdevice, uint8_t const **buffers, const uint32_t *bufferSizes, uint32_t bufferCount)
{
    if (bufferCount == 0) {
        return 0;
    }

    uint32_t dataSize;
    const uint8_t * pData;

    if (bufferCount == 1) {
        dataSize = bufferSizes[0];
        if (dataSize > M25P16_PAGESIZE) {
            return 0;
        }
        pData = buffers[0];
    } else {
        // Need to copy all buffers into page buffer
        dataSize = 0;
        uint8_t * pBuffer = m25p16_page_buffer;
        for (uint32_t i = 0; i < bufferCount; i++) {
            dataSize += bufferSizes[i];
            if (dataSize > sizeof(m25p16_page_buffer)) {
                return 0;
            }
            memcpy(pBuffer, buffers[i], bufferSizes[i]);
            pBuffer += bufferSizes[i];
        }
        pData = m25p16_page_buffer;
    }

    m25p16_waitForReady(fdevice);

    quadSpiTransmit1LINE(fdevice->io.handle.quadSpi, M25P16_INSTRUCTION_WRITE_ENABLE, 0, NULL, 0);

    quadSpiTransmitWithAddress4LINES(fdevice->io.handle.quadSpi, M25P16_INSTRUCTION_QPAGE_PROGRAM, 0,
                                     fdevice->currentWriteAddress, fdevice->isLargeFlash ? 32 : 24, pData, dataSize);

    fdevice->currentWriteAddress += dataSize;

    if (fdevice->callback) {
        fdevice->callback(bufferSizes[0]);
    }

    fdevice->couldBeBusy = true;

    return bufferSizes[0];
}

static void m25p16_pageProgramQspi(flashDevice_t *fdevice, uint32_t address, const uint8_t *data, uint32_t length, void (*callback)(uint32_t length))
{
    m25p16_pageProgramBegin(fdevice, address, callback);

    m25p16_pageProgramContinueQspi(fdevice, &data, &length, 1);

    m25p16_pageProgramFinish(fdevice);
}
#endif /* USE_QUADSPI */


/**
 * Read `length` bytes into the provided `buffer` from the flash starting from the given `address` (which need not lie
 * on a page boundary).
 *
 * The number of bytes actually read is returned, which can be zero if an error or timeout occurred.
 */
static int m25p16_readBytes(flashDevice_t *fdevice, uint32_t address, uint8_t *buffer, uint32_t length)
{
    STATIC_DMA_DATA_AUTO uint8_t readStatus[2] = { M25P16_INSTRUCTION_READ_STATUS_REG, 0 };
    STATIC_DMA_DATA_AUTO uint8_t readyStatus[2];
    STATIC_DMA_DATA_AUTO uint8_t readBytes[5] = { M25P16_INSTRUCTION_READ_BYTES };

    // Ensure any prior DMA has completed before continuing
    spiWait(fdevice->io.handle.dev);

    busSegment_t segments[] = {
            {.u.buffers = {readStatus, readyStatus}, sizeof(readStatus), true, m25p16_callbackReady},
            {.u.buffers = {readBytes, NULL}, fdevice->isLargeFlash ? 5 : 4, false, NULL},
            {.u.buffers = {NULL, buffer}, length, true, NULL},
            {.u.link = {NULL, NULL}, 0, true, NULL},
    };

    // Patch the readBytes command
    m25p16_setCommandAddress(&readBytes[1], address, fdevice->isLargeFlash);

    spiSetClkDivisor(fdevice->io.handle.dev, spiCalculateDivider(maxReadClkSPIHz));

    spiSequence(fdevice->io.handle.dev, fdevice->couldBeBusy ? &segments[0] : &segments[1]);

    // Block until code is re-factored to exploit non-blocking
    spiWait(fdevice->io.handle.dev);

    spiSetClkDivisor(fdevice->io.handle.dev, spiCalculateDivider(maxClkSPIHz));

    return length;
}

#ifdef USE_QUADSPI
// Reading data QSPI mode

static int m25p16_readBytesQspi(flashDevice_t *fdevice, uint32_t address, uint8_t *buffer, uint32_t length)
{
    m25p16_waitForReady(fdevice);

    quadSpiReceiveWithAddress4LINES(fdevice->io.handle.quadSpi, M25P16_INSTRUCTION_QUAD_READ, M25P16_FAST_READ_DUMMY_CYCLES,
                                    address, fdevice->isLargeFlash ? 32 : 24, buffer, length);

    return length;
}
#endif /* USE_QUADSPI */

/**
 * Fetch information about the detected flash chip layout.
 *
 * Can be called before calling m25p16_init() (the result would have totalSize = 0).
 */
static const flashGeometry_t* m25p16_getGeometry(flashDevice_t *fdevice)
{
    return &fdevice->geometry;
}

const flashVTable_t m25p16_vTable = {
    .configure = m25p16_configure,
    .isReady = m25p16_isReady,
    .waitForReady = m25p16_waitForReady,
    .eraseSector = m25p16_eraseSector,
    .eraseCompletely = m25p16_eraseCompletely,
    .pageProgramBegin = m25p16_pageProgramBegin,
    .pageProgramContinue = m25p16_pageProgramContinue,
    .pageProgramFinish = m25p16_pageProgramFinish,
    .pageProgram = m25p16_pageProgram,
    .readBytes = m25p16_readBytes,
    .getGeometry = m25p16_getGeometry,
};

#ifdef USE_QUADSPI
const flashVTable_t m25p16Qspi_vTable = {
    .isReady = m25p16_isReady,
    .waitForReady = m25p16_waitForReady,
    .eraseSector = m25p16_eraseSectorQspi,
    .eraseCompletely = m25p16_eraseCompletelyQspi,
    .pageProgramBegin = m25p16_pageProgramBegin,
    .pageProgramContinue = m25p16_pageProgramContinueQspi,
    .pageProgramFinish = m25p16_pageProgramFinish,
    .pageProgram = m25p16_pageProgramQspi,
    .readBytes = m25p16_readBytesQspi,
    .getGeometry = m25p16_getGeometry,
};
#endif /* USE_QUADSPI */

#endif
