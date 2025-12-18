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
 *
 * Author: Dominic Clifton <me@dominiclifton.name>
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_FLASH_MT29F

#include "drivers/flash/flash.h"
#include "drivers/flash/flash_impl.h"
#include "drivers/flash/flash_mt29f.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_quadspi.h"
#include "drivers/io.h"
#include "drivers/time.h"

// Instructions

#define MT29F_INSTRUCTION_RDID                      0x9F
#define MT29F_INSTRUCTION_DEVICE_RESET              0xFF

#define MT29F_INSTRUCTION_GET_FEATURE               0x0F
#define MT29F_INSTRUCTION_SET_FEATURE               0x1F
#define MT29F_INSTRUCTION_WRITE_ENABLE              0x06
#define MT29F_INSTRUCTION_BLOCK_ERASE               0xD8
#define MT29F_INSTRUCTION_PROGRAM_LOAD_X1           0x02
#define MT29F_INSTRUCTION_PROGRAM_LOAD_RANDOM_X1    0x84
#define MT29F_INSTRUCTION_PROGRAM_EXECUTE           0x10

#define MT29F_INSTRUCTION_PAGE_READ                 0x13
#define MT29F_INSTRUCTION_READ_FROM_CACHE_X1        0x03
#define MT29F_INSTRUCTION_READ_FROM_CACHE_X4        0x6B

// Config/status register addresses
#define MT29F_BLOCK_LOCK                            0xA0
#define MT29F_CONFIGURATION_REG                     0xB0
#define MT29F_STATUS_REG                            0xC0

// Bits in config/status register 1 (MT29F_BLOCK_LOCK)
#define MT29F_BLOCK_LOCK_CLEAR                (0)
#define MT29F_BLOCK_LOCK_RESERVED_0           (1 << 0)
#define MT29F_BLOCK_LOCK_WP_HOLD_DISABLE      (1 << 1)
#define MT29F_BLOCK_LOCK_TB_ENABLE            (1 << 2)
#define MT29F_BLOCK_LOCK_PB0_ENABLE           (1 << 3)
#define MT29F_BLOCK_LOCK_PB1_ENABLE           (1 << 4)
#define MT29F_BLOCK_LOCK_PB2_ENABLE           (1 << 5)
#define MT29F_BLOCK_LOCK_PB3_ENABLE           (1 << 6)
#define MT29F_BLOCK_LOCK_BRWD_ENABLE          (1 << 7)

// Bits in config/status register 2 (MT29F_CONFIGURATION_REG)
#define MT29F_CONFIG_RESERVED_0         (1 << 0)
#define MT29F_CONFIG_CFG0               (1 << 1)
#define MT29F_CONFIG_RESERVED_1         (1 << 2)
#define MT29F_CONFIG_RESERVED_2         (1 << 3)
#define MT29F_CONFIG_ECC_ENABLE         (1 << 4)
// LOT = Lock Tight
#define MT29F_CONFIG_LOT_ENABLE         (1 << 5)
#define MT29F_CONFIG_CFG1               (1 << 6)
#define MT29F_CONFIG_CFG2               (1 << 7)

// Bits in config/status register 3 (MT29F_STATREG)
// OIP = Operation in progress
#define MT29F_STATUS_FLAG_OIP           (1 << 0)
// WEL = Write enable latch
#define MT29F_STATUS_FLAG_WEL           (1 << 1)
#define MT29F_STATUS_ERASE_FAIL         (1 << 2)
#define MT29F_STATUS_PROGRAM_FAIL       (1 << 3)
#define MT29F_STATUS_FLAG_ECC_POS       4
#define MT29F_STATUS_FLAG_ECC_MASK      ((1 << 6)|(1 << 5)|(1 << 4))
#define MT29F_STATUS_FLAG_ECC(status)   (((status) & MT29F_STATUS_FLAG_ECC_MASK) >> MT29F_STATUS_FLAG_ECC_POS)
// CRBUSY = Cache read busy
#define MT29F_STATUS_FLAG_CRBUSY        (1 << 7)
#define MT29F_STATUS_PROGRAM_FAIL       (1 << 3)

// Some useful defs and macros
#define MT29F_PAGE_SIZE                 fdevice->geometry.pageSize
#define MT29F_LINEAR_TO_COLUMN(laddr)   ((laddr) % MT29F_PAGE_SIZE)
#define MT29F_LINEAR_TO_PAGE(laddr)     ((laddr) / MT29F_PAGE_SIZE)
#define MT29F_LINEAR_TO_BLOCK(laddr)    (MT29F_LINEAR_TO_PAGE(laddr) / fdevice->geometry.pagesPerSector)
#define MT29F_BLOCK_TO_PAGE(block)      ((block) * fdevice->geometry.pagesPerSector)
#define MT29F_BLOCK_TO_LINEAR(block)    (MT29F_BLOCK_TO_PAGE(block) * MT29F_PAGE_SIZE)

// IMPORTANT: Timeout values are currently required to be set to the highest value required by any of the supported flash chips by this driver

// The timeout values (2ms minimum to avoid 1 tick advance in consecutive calls to millis).
#define MT29F_TIMEOUT_PAGE_READ_MS        2   // tREmax = 60us (ECC enabled)
#define MT29F_TIMEOUT_PAGE_PROGRAM_MS     2   // tPROGmax = 600us
#define MT29F_TIMEOUT_BLOCK_ERASE_MS      10  // tERSmax = 10ms
#define MT29F_TIMEOUT_RESET_MS            2   // tRSTmax = 570us (ECC enabled)

// Sizes (in bits)  Note: These differ for devices with > 1Gb of flash
#define MT29F_STATUS_REGISTER_SIZE        8
#define MT29F_PAGE_ADDRESS_SIZE           24
#define MT29F_COLUMN_ADDRESS_SIZE         16 // 12 bytes + 3 + plane select

// Table of recognised FLASH devices
struct {
    // bytes 1 and 2 from the READ_ID response.
    uint16_t        jedecID;

    // See 'Array organization' in the datasheet.

    // aka 'blocks per lun'
    flashSector_t   sectors;
    // aka 'pages per block'
    uint16_t        pagesPerSector;
    uint16_t        pageSize;
} mt29fFlashConfig[] = {
    // Winbond MT29F1G01ABAFDWB-IT:F
    // Datasheet: https://www.micron.com/content/dam/micron/global/secure/products/data-sheet/nand-flash/70-series/m78a-1gb-3v-nand-spi.pdf
    { 0x2C14, 1024, 64, 2048 },
    { 0, 0, 0, 0 },
};

static bool mt29f_waitForReady(flashDevice_t *fdevice);

static void mt29f_setTimeout(flashDevice_t *fdevice, uint32_t timeoutMillis)
{
    uint32_t now = millis();
    fdevice->timeoutAt = now + timeoutMillis;
}

/**
 * Send the given command byte to the device.
 */
static void mt29f_performOneByteCommand(flashDeviceIO_t *io, uint8_t command)
{
    if (io->mode == FLASHIO_SPI) {
        extDevice_t *dev = io->handle.dev;

        busSegment_t segments[] = {
                {.u.buffers = {&command, NULL}, sizeof(command), true, NULL},
                {.u.link = {NULL, NULL}, 0, true, NULL},
        };

        spiSequence(dev, &segments[0]);

        // Block pending completion of SPI access
        spiWait(dev);
    }
#ifdef USE_QUADSPI
    else if (io->mode == FLASHIO_QUADSPI) {
        extDevice_t *dev = io->handle.dev;
        quadSpiTransmit1LINE(dev, command, 0, NULL, 0);
    }
#endif
}

static void mt29f_performCommandWithPageAddress(flashDeviceIO_t *io, uint8_t command, uint32_t pageAddress)
{
    if (io->mode == FLASHIO_SPI) {
        extDevice_t *dev = io->handle.dev;

        uint8_t cmd[] = { command, 0, (pageAddress >> 8) & 0xff, (pageAddress >> 0) & 0xff};

        busSegment_t segments[] = {
                {.u.buffers = {cmd, NULL}, sizeof(cmd), true, NULL},
                {.u.link = {NULL, NULL}, 0, true, NULL},
        };

        spiSequence(dev, &segments[0]);

        // Block pending completion of SPI access
        spiWait(dev);
    }
#ifdef USE_QUADSPI
    else if (io->mode == FLASHIO_QUADSPI) {
        extDevice_t *dev = io->handle.dev;

        quadSpiInstructionWithAddress1LINE(dev, command, 0, pageAddress & 0xffffff, MT29F_PAGE_ADDRESS_SIZE);
    }
#endif
}

static uint8_t mt29f_readRegister(flashDeviceIO_t *io, uint8_t reg)
{
    if (io->mode == FLASHIO_SPI) {
        extDevice_t *dev = io->handle.dev;

        uint8_t cmd[3] = { MT29F_INSTRUCTION_GET_FEATURE, reg, 0 };
        uint8_t in[3];

        busSegment_t segments[] = {
                {.u.buffers = {cmd, in}, sizeof(cmd), true, NULL},
                {.u.link = {NULL, NULL}, 0, true, NULL},
        };

        // Ensure any prior DMA has completed before continuing
        spiWait(dev);

        spiSequence(dev, &segments[0]);

        // Block pending completion of SPI access
        spiWait(dev);

        return in[2];
    }
#ifdef USE_QUADSPI
    else if (io->mode == FLASHIO_QUADSPI) {

        extDevice_t *dev = io->handle.dev;

        uint8_t in[MT29F_STATUS_REGISTER_SIZE / 8];
        quadSpiReceiveWithAddress1LINE(dev, MT29F_INSTRUCTION_GET_FEATURE, 0, reg, MT29F_STATUS_REGISTER_SIZE, in, sizeof(in));

        return in[0];
    }
#endif
    return 0;
}

static void mt29f_writeRegister(flashDeviceIO_t *io, uint8_t reg, uint8_t data)
{
    if (io->mode == FLASHIO_SPI) {
        extDevice_t *dev = io->handle.dev;
        uint8_t cmd[3] = { MT29F_INSTRUCTION_SET_FEATURE, reg, data };

        busSegment_t segments[] = {
                {.u.buffers = {cmd, NULL}, sizeof(cmd), true, NULL},
                {.u.link = {NULL, NULL}, 0, true, NULL},
        };

        // Ensure any prior DMA has completed before continuing
        spiWait(dev);

        spiSequence(dev, &segments[0]);

        // Block pending completion of SPI access
        spiWait(dev);
   }
#ifdef USE_QUADSPI
   else if (io->mode == FLASHIO_QUADSPI) {
       extDevice_t *dev = io->handle.dev;

       quadSpiTransmitWithAddress1LINE(dev, MT29F_INSTRUCTION_SET_FEATURE, 0, reg, MT29F_STATUS_REGISTER_SIZE, &data, 1);
   }
#endif
}

static void mt29f_deviceReset(flashDevice_t *fdevice)
{
    flashDeviceIO_t *io = &fdevice->io;

    mt29f_performOneByteCommand(io, MT29F_INSTRUCTION_DEVICE_RESET);

    mt29f_setTimeout(fdevice, MT29F_TIMEOUT_RESET_MS);
    mt29f_waitForReady(fdevice);

    // No protection, WP-E off, WP-E prevents use of IO2
    mt29f_writeRegister(io, MT29F_BLOCK_LOCK, MT29F_BLOCK_LOCK_CLEAR);

    // Buffered read mode (BUF = 1), ECC enabled (ECC = 1)
    mt29f_writeRegister(io, MT29F_CONFIGURATION_REG, MT29F_CONFIG_ECC_ENABLE);
}

static bool mt29f_isReady(flashDevice_t *fdevice)
{
    // If we're waiting on DMA completion, then SPI is busy
    if (fdevice->io.mode == FLASHIO_SPI) {
        if (fdevice->io.handle.dev->bus->useDMA && spiIsBusy(fdevice->io.handle.dev)) {
            return false;
        }
    }

    // Irrespective of the current state of fdevice->couldBeBusy read the status or device blocks

    // Poll the FLASH device to see if it's busy
    fdevice->couldBeBusy = ((mt29f_readRegister(&fdevice->io, MT29F_STATUS_REG) & (MT29F_STATUS_FLAG_OIP | MT29F_STATUS_FLAG_CRBUSY)) != 0);

    return !fdevice->couldBeBusy;
}

static bool mt29f_waitForReady(flashDevice_t *fdevice)
{
    while (!mt29f_isReady(fdevice)) {
        uint32_t now = millis();
        if (cmp32(now, fdevice->timeoutAt) >= 0) {
            return false;
        }
    }
    fdevice->timeoutAt = 0;

    return true;
}

/**
 * The flash requires this write enable command to be sent before commands that would cause
 * a write like program and erase.
 */
static void mt29f_writeEnable(flashDevice_t *fdevice)
{
    mt29f_performOneByteCommand(&fdevice->io, MT29F_INSTRUCTION_WRITE_ENABLE);

    // Assume that we're about to do some writing, so the device is just about to become busy
    fdevice->couldBeBusy = true;
}

const flashVTable_t mt29f_vTable;

bool mt29f_identify(flashDevice_t *fdevice, uint32_t jedecID)
{
    flashGeometry_t *geometry = &fdevice->geometry;
    uint8_t index;

    // only two bytes of JEDEC ID are used, so shift right 8 bits
    uint16_t jedecID_u16 = jedecID >> 8;

    for (index = 0; mt29fFlashConfig[index].jedecID; index++) {
        if (mt29fFlashConfig[index].jedecID == jedecID_u16) {
            geometry->sectors = mt29fFlashConfig[index].sectors;
            geometry->pagesPerSector = mt29fFlashConfig[index].pagesPerSector;
            geometry->pageSize = mt29fFlashConfig[index].pageSize;
            break;
        }
    }

    if (mt29fFlashConfig[index].jedecID == 0) {
        // Unsupported chip
        fdevice->geometry.sectors = 0;
        fdevice->geometry.pagesPerSector = 0;
        fdevice->geometry.sectorSize = 0;
        fdevice->geometry.totalSize = 0;
        return false;
    }

    fdevice->geometry.flashType = FLASH_TYPE_NAND;
    fdevice->geometry.sectorSize = fdevice->geometry.pagesPerSector * fdevice->geometry.pageSize;
    fdevice->geometry.totalSize = fdevice->geometry.sectorSize * fdevice->geometry.sectors;

    fdevice->couldBeBusy = true; // Just for luck we'll assume the chip could be busy even though it isn't specced to be
    fdevice->vTable = &mt29f_vTable;

    return true;
}

static void mt29f_deviceInit(flashDevice_t *flashdev);

static void mt29f_configure(flashDevice_t *fdevice, uint32_t configurationFlags)
{
    if (configurationFlags & FLASH_CF_SYSTEM_IS_MEMORY_MAPPED) {
        return;
    }

    mt29f_deviceReset(fdevice);

    if (fdevice->io.mode == FLASHIO_SPI) {    // Need to set clock speed for 8kHz logging support with SPI
        spiSetClkDivisor(fdevice->io.handle.dev, spiCalculateDivider(100000000));
    }

    mt29f_deviceInit(fdevice);
}

/**
 * Erase a sector full of bytes to all 1's at the given byte offset in the flash chip.
 */
static void mt29f_eraseSector(flashDevice_t *fdevice, uint32_t address)
{

    mt29f_waitForReady(fdevice);

    mt29f_writeEnable(fdevice);

    mt29f_performCommandWithPageAddress(&fdevice->io, MT29F_INSTRUCTION_BLOCK_ERASE, MT29F_LINEAR_TO_PAGE(address));

    mt29f_setTimeout(fdevice, MT29F_TIMEOUT_BLOCK_ERASE_MS);
}

//
// MT29F does not support full chip erase.
// Call eraseSector repeatedly.

static void mt29f_eraseCompletely(flashDevice_t *fdevice)
{
    for (uint32_t block = 0; block < fdevice->geometry.sectors; block++) {
        mt29f_eraseSector(fdevice, MT29F_BLOCK_TO_LINEAR(block));
    }
}

#ifdef USE_QUADSPI
static void mt29f_programDataLoad(flashDevice_t *fdevice, uint16_t columnAddress, const uint8_t *data, int length)
{

    mt29f_waitForReady(fdevice);

    if (fdevice->io.mode == FLASHIO_SPI) {
        extDevice_t *dev = fdevice->io.handle.dev;
        uint8_t cmd[] = { MT29F_INSTRUCTION_PROGRAM_LOAD_X1, columnAddress >> 8, columnAddress & 0xff };

         busSegment_t segments[] = {
                 {.u.buffers = {cmd, NULL}, sizeof(cmd), false, NULL},
                 {.u.buffers = {(uint8_t *)data, NULL}, length, true, NULL},
                 {.u.link = {NULL, NULL}, 0, true, NULL},
         };

         spiSequence(dev, &segments[0]);

         // Block pending completion of SPI access
         spiWait(dev);
   }
#ifdef USE_QUADSPI
   else if (fdevice->io.mode == FLASHIO_QUADSPI) {
       extDevice_t *dev = fdevice->io.handle.dev;

       quadSpiTransmitWithAddress1LINE(dev, MT29F_INSTRUCTION_PROGRAM_LOAD_X1, 0, columnAddress, MT29F_COLUMN_ADDRESS_SIZE, data, length);
    }
#endif

    mt29f_setTimeout(fdevice, MT29F_TIMEOUT_PAGE_PROGRAM_MS);
}

static void mt29f_randomProgramDataLoad(flashDevice_t *fdevice, uint16_t columnAddress, const uint8_t *data, int length)
{
    uint8_t cmd[] = { MT29F_INSTRUCTION_PROGRAM_LOAD_RANDOM_X1, columnAddress >> 8, columnAddress & 0xff };

    mt29f_waitForReady(fdevice);

    if (fdevice->io.mode == FLASHIO_SPI) {
        extDevice_t *dev = fdevice->io.handle.dev;

        busSegment_t segments[] = {
                {.u.buffers = {cmd, NULL}, sizeof(cmd), false, NULL},
                {.u.buffers = {(uint8_t *)data, NULL}, length, true, NULL},
                {.u.link = {NULL, NULL}, 0, true, NULL},
        };

        spiSequence(dev, &segments[0]);

        // Block pending completion of SPI access
        spiWait(dev);
    }
#ifdef USE_QUADSPI
    else if (fdevice->io.mode == FLASHIO_QUADSPI) {
        extDevice_t *dev = fdevice->io.handle.dev;

        quadSpiTransmitWithAddress1LINE(dev, MT29F_INSTRUCTION_PROGRAM_LOAD_RANDOM_X1, 0, columnAddress, MT29F_COLUMN_ADDRESS_SIZE, data, length);
     }
#endif

    mt29f_setTimeout(fdevice, MT29F_TIMEOUT_PAGE_PROGRAM_MS);

}
#endif

static void mt29f_programExecute(flashDevice_t *fdevice, uint32_t pageAddress)
{
    mt29f_waitForReady(fdevice);

    mt29f_performCommandWithPageAddress(&fdevice->io, MT29F_INSTRUCTION_PROGRAM_EXECUTE, pageAddress);

    mt29f_setTimeout(fdevice, MT29F_TIMEOUT_PAGE_PROGRAM_MS);
}

//
// Writes are done in three steps:
// (1) Load internal data buffer with data to write
//     - We use "Random Load Program Data", as "Load Program Data" resets unused data bytes in the buffer to 0xff.
//     - Each "Random Load Program Data" instruction must be accompanied by at least a single data.
//     - Each "Random Load Program Data" instruction terminates at the rising of CS.
// (2) Enable write
// (3) Issue "Execute Program"
//

/*
flashfs page program behavior
- Single program never crosses page boundary.
- Except for this characteristic, it program arbitral size.
- Write address is, naturally, not a page boundary.

To cope with this behavior.

If buffer is dirty and programLoadAddress != address, then the last page is a partial write;
issue PAGE_PROGRAM_EXECUTE to flash buffer contents, clear dirty and record the address as programLoadAddress and programStartAddress.

Mark buffer as dirty.
If programLoadAddress is on page boundary, then issue PROGRAM_LOAD_DATA, else issue RANDOM_PROGRAM_LOAD_DATA.
Update programLoadAddress.
Optionally observe the programLoadAddress, and if it's on page boundary, issue PAGE_PROGRAM_EXECUTE.

Observe programLoadAddress. If it's on page boundary, issue PAGE_PROGRAM_EXECUTE and clear dirty, else just return.
If pageProgramContinue observes the page boundary, then do nothing(?).
*/

static uint32_t programStartAddress;
static uint32_t programLoadAddress;
static bool bufferDirty = false;

// Called in ISR context
// Check if the status was busy and if so repeat the poll
static busStatus_e mt29f_callbackReady(uintptr_t arg)
{
    flashDevice_t *fdevice = (flashDevice_t *)arg;
    extDevice_t *dev = fdevice->io.handle.dev;

    uint8_t readyPoll = dev->bus->curSegment->u.buffers.rxData[2];

    if (readyPoll & MT29F_STATUS_FLAG_CRBUSY) {
        return BUS_BUSY;
    }

    // Bus is now known not to be busy
    fdevice->couldBeBusy = false;

    return BUS_READY;
}

#ifdef USE_QUADSPI
static bool isProgramming = false;

static void mt29f_pageProgramBegin(flashDevice_t *fdevice, uint32_t address, void (*callback)(uintptr_t arg))
{
    fdevice->callback = callback;

    if (bufferDirty) {
        if (address != programLoadAddress) {
            mt29f_waitForReady(fdevice);

            isProgramming = false;

            mt29f_writeEnable(fdevice);

            mt29f_programExecute(fdevice, MT29F_LINEAR_TO_PAGE(programStartAddress));

            bufferDirty = false;
            isProgramming = true;
        }
    } else {
        programStartAddress = programLoadAddress = address;
    }
}

static uint32_t mt29f_pageProgramContinue(flashDevice_t *fdevice, uint8_t const **buffers, const uint32_t *bufferSizes, uint32_t bufferCount)
{
    if (bufferCount < 1) {
        fdevice->callback(0);
        return 0;
    }

    mt29f_waitForReady(fdevice);

    mt29f_writeEnable(fdevice);

    isProgramming = false;

    if (!bufferDirty) {
        mt29f_programDataLoad(fdevice, MT29F_LINEAR_TO_COLUMN(programLoadAddress), buffers[0], bufferSizes[0]);
    } else {
        mt29f_randomProgramDataLoad(fdevice, MT29F_LINEAR_TO_COLUMN(programLoadAddress), buffers[0], bufferSizes[0]);
    }

    // XXX Test if write enable is reset after each data loading.

    bufferDirty = true;
    programLoadAddress += bufferSizes[0];

    if (fdevice->callback) {
        fdevice->callback(bufferSizes[0]);
    }

    return bufferSizes[0];
}

static uint32_t currentPage = UINT32_MAX;

static void mt29f_pageProgramFinish(flashDevice_t *fdevice)
{
    if (bufferDirty && MT29F_LINEAR_TO_COLUMN(programLoadAddress) == 0) {

        currentPage = MT29F_LINEAR_TO_PAGE(programStartAddress); // reset page to the page being written

        mt29f_programExecute(fdevice, MT29F_LINEAR_TO_PAGE(programStartAddress));

        bufferDirty = false;
        isProgramming = true;

        programStartAddress = programLoadAddress;
    }
}
#else
static void mt29f_pageProgramBegin(flashDevice_t *fdevice, uint32_t address, void (*callback)(uintptr_t arg))
{
    fdevice->callback = callback;
    fdevice->currentWriteAddress = address;

}

static uint32_t currentPage = UINT32_MAX;

// Called in ISR context
// A write enable has just been issued
static busStatus_e mt29f_callbackWriteEnable(uintptr_t arg)
{
    flashDevice_t *fdevice = (flashDevice_t *)arg;

    // As a write has just occurred, the device could be busy
    fdevice->couldBeBusy = true;

    return BUS_READY;
}

// Called in ISR context
// Write operation has just completed
static busStatus_e mt29f_callbackWriteComplete(uintptr_t arg)
{
    flashDevice_t *fdevice = (flashDevice_t *)arg;

    fdevice->currentWriteAddress += fdevice->bytesWritten;
    // Call transfer completion callback
    if (fdevice->callback) {
        fdevice->callback(fdevice->bytesWritten);
    }

    return BUS_READY;
}

static uint32_t mt29f_pageProgramContinue(flashDevice_t *fdevice, uint8_t const **buffers, const uint32_t *bufferSizes, uint32_t bufferCount)
{
    if (bufferCount < 1) {
        fdevice->callback(0);
        return 0;
    }

    // The segment list cannot be in automatic storage as this routine is non-blocking
    STATIC_DMA_DATA_AUTO uint8_t readStatus[] = { MT29F_INSTRUCTION_GET_FEATURE, MT29F_STATUS_REG, 0 };
    STATIC_DMA_DATA_AUTO uint8_t readyStatus[3];
    STATIC_DMA_DATA_AUTO uint8_t writeEnable[] = { MT29F_INSTRUCTION_WRITE_ENABLE };
    STATIC_DMA_DATA_AUTO uint8_t progExecCmd[] = { MT29F_INSTRUCTION_PROGRAM_EXECUTE, 0, 0, 0};
    STATIC_DMA_DATA_AUTO uint8_t progExecDataLoad[] = { MT29F_INSTRUCTION_PROGRAM_LOAD_X1, 0, 0};
    STATIC_DMA_DATA_AUTO uint8_t progRandomProgDataLoad[] = { MT29F_INSTRUCTION_PROGRAM_LOAD_RANDOM_X1, 0, 0};

    static busSegment_t segmentsDataLoad[] = {
        {.u.buffers = {readStatus, readyStatus}, sizeof(readStatus), true, mt29f_callbackReady},
        {.u.buffers = {writeEnable, NULL}, sizeof(writeEnable), true, mt29f_callbackWriteEnable},
        {.u.buffers = {progExecDataLoad, NULL}, sizeof(progExecDataLoad), false, NULL},
        {.u.link = {NULL, NULL}, 0, true, NULL},
    };

    static busSegment_t segmentsRandomDataLoad[] = {
        {.u.buffers = {readStatus, readyStatus}, sizeof(readStatus), true, mt29f_callbackReady},
        {.u.buffers = {writeEnable, NULL}, sizeof(writeEnable), true, mt29f_callbackWriteEnable},
        {.u.buffers = {progRandomProgDataLoad, NULL}, sizeof(progRandomProgDataLoad), false, NULL},
        {.u.link = {NULL, NULL}, 0, true, NULL},
    };

    static busSegment_t segmentsBuffer[] = {
        {.u.buffers = {NULL, NULL}, 0, true, NULL},
        {.u.link = {NULL, NULL}, 0, true, NULL},
    };

    static busSegment_t segmentsFlash[] = {
        {.u.buffers = {readStatus, readyStatus}, sizeof(readStatus), true, mt29f_callbackReady},
        {.u.buffers = {writeEnable, NULL}, sizeof(writeEnable), true, mt29f_callbackWriteEnable},
        {.u.buffers = {progExecCmd, NULL}, sizeof(progExecCmd), true, mt29f_callbackWriteComplete},
        {.u.link = {NULL, NULL}, 0, true, NULL},
    };

    busSegment_t *programSegment;

    // Ensure any prior DMA has completed before continuing
    spiWait(fdevice->io.handle.dev);

    uint32_t columnAddress;

    if (bufferDirty) {
        columnAddress = MT29F_LINEAR_TO_COLUMN(programLoadAddress);
        // Set the address and buffer details for the random data load
        progRandomProgDataLoad[1] = (columnAddress >> 8) & 0xff;
        progRandomProgDataLoad[2] = columnAddress & 0xff;
        programSegment = segmentsRandomDataLoad;
    } else {
        programStartAddress = programLoadAddress = fdevice->currentWriteAddress;
        columnAddress = MT29F_LINEAR_TO_COLUMN(programLoadAddress);
        // Set the address and buffer details for the data load
        progExecDataLoad[1] = (columnAddress >> 8) & 0xff;
        progExecDataLoad[2] = columnAddress & 0xff;
        programSegment = segmentsDataLoad;
    }

    // Add the data buffer
    segmentsBuffer[0].u.buffers.txData = (uint8_t *)buffers[0];
    segmentsBuffer[0].len = bufferSizes[0];
    segmentsBuffer[0].callback = NULL;

    spiLinkSegments(fdevice->io.handle.dev, programSegment, segmentsBuffer);

    bufferDirty = true;
    programLoadAddress += bufferSizes[0];

    if (MT29F_LINEAR_TO_COLUMN(programLoadAddress) == 0) {
        // Flash the loaded data
        currentPage = MT29F_LINEAR_TO_PAGE(programStartAddress);

        progExecCmd[2] = (currentPage >> 8) & 0xff;
        progExecCmd[3] = currentPage & 0xff;

        spiLinkSegments(fdevice->io.handle.dev, segmentsBuffer, segmentsFlash);

        bufferDirty = false;

        programStartAddress = programLoadAddress;
    } else {
        // Callback on completion of data load
        segmentsBuffer[0].callback = mt29f_callbackWriteComplete;
    }

    if (!fdevice->couldBeBusy) {
        // Skip the ready check
        programSegment++;
    }

    fdevice->bytesWritten = bufferSizes[0];

    spiSequence(fdevice->io.handle.dev, programSegment);

    if (fdevice->callback == NULL) {
        // No callback was provided so block
        // Block pending completion of SPI access
        spiWait(fdevice->io.handle.dev);
    }

    return fdevice->bytesWritten;
}

static void mt29f_pageProgramFinish(flashDevice_t *fdevice)
{
    UNUSED(fdevice);
}
#endif // USE_QUADSPI

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

static void mt29f_pageProgram(flashDevice_t *fdevice, uint32_t address, const uint8_t *data, uint32_t length, void (*callback)(uintptr_t arg))
{
    mt29f_pageProgramBegin(fdevice, address, callback);
    mt29f_pageProgramContinue(fdevice, &data, &length, 1);
    mt29f_pageProgramFinish(fdevice);
}

static void mt29f_flush(flashDevice_t *fdevice)
{
    if (bufferDirty) {
        currentPage = MT29F_LINEAR_TO_PAGE(programStartAddress); // reset page to the page being written

        mt29f_programExecute(fdevice, MT29F_LINEAR_TO_PAGE(programStartAddress));

        bufferDirty = false;
    }
}

static void mt29f_addError(uint32_t address, uint8_t code)
{
    UNUSED(address);
    UNUSED(code);
}

/**
 * Read `length` bytes into the provided `buffer` from the flash starting from the given `address` (which need not lie
 * on a page boundary).
 *
 * Waits up to MT29F_TIMEOUT_PAGE_READ_MS milliseconds for the flash to become ready before reading.
 *
 * The number of bytes actually read is returned, which can be zero if an error or timeout occurred.
 */

// Continuous read mode (BUF = 0):
// (1) "Page Data Read" command is executed for the page pointed by address
// (2) "Read Data" command is executed for bytes not requested and data are discarded
// (3) "Read Data" command is executed and data are stored directly into caller's buffer
//
// Buffered read mode (BUF = 1), non-read ahead
// (1) If currentBufferPage != requested page, then issue PAGE_DATA_READ on requested page.
// (2) Compute transferLength as smaller of remaining length and requested length.
// (3) Issue READ_DATA on column address.
// (4) Return transferLength.

static int mt29f_readBytes(flashDevice_t *fdevice, uint32_t address, uint8_t *buffer, uint32_t length)
{
    uint32_t targetPage = MT29F_LINEAR_TO_PAGE(address);

    // As data is buffered before being written a flush must be performed before attempting a read
    bool was_dirty = bufferDirty;
    mt29f_flush(fdevice);

    bool page_change = (currentPage != targetPage);

    if (was_dirty || page_change) {
        if (!mt29f_waitForReady(fdevice)) {
            return 0;
        }
    }

    if (page_change) {
        currentPage = UINT32_MAX;

        mt29f_performCommandWithPageAddress(&fdevice->io, MT29F_INSTRUCTION_PAGE_READ, targetPage);

        mt29f_setTimeout(fdevice, MT29F_TIMEOUT_PAGE_READ_MS);
        if (!mt29f_waitForReady(fdevice)) {
            return 0;
        }

        currentPage = targetPage;
    }

    uint32_t column = MT29F_LINEAR_TO_COLUMN(address);
    uint16_t transferLength;

    if (length > MT29F_PAGE_SIZE - column) {
        transferLength = MT29F_PAGE_SIZE - column;
    } else {
        transferLength = length;
    }

    if (fdevice->io.mode == FLASHIO_SPI) {
        extDevice_t *dev = fdevice->io.handle.dev;

        uint8_t readStatus[] = { MT29F_INSTRUCTION_GET_FEATURE, MT29F_STATUS_REG, 0 };
        uint8_t readyStatus[3];

        uint8_t cmd[4];
        cmd[0] = MT29F_INSTRUCTION_READ_FROM_CACHE_X1;
        cmd[1] = (column >> 8) & 0xff;
        cmd[2] = (column >> 0) & 0xff;
        cmd[3] = 0;

        busSegment_t segments[] = {
                {.u.buffers = {readStatus, readyStatus}, sizeof(readStatus), true, mt29f_callbackReady},
                {.u.buffers = {cmd, NULL}, sizeof(cmd), false, NULL},
                {.u.buffers = {NULL, buffer}, length, true, NULL},
                {.u.link = {NULL, NULL}, 0, true, NULL},
        };

        spiSequence(dev, &segments[0]);

        // Block pending completion of SPI access
        spiWait(dev);
    }
#ifdef USE_QUADSPI
    else if (fdevice->io.mode == FLASHIO_QUADSPI) {
        extDevice_t *dev = fdevice->io.handle.dev;

        quadSpiReceiveWithAddress4LINES(dev, MT29F_INSTRUCTION_READ_FROM_CACHE_X4, 8, column, MT29F_COLUMN_ADDRESS_SIZE, buffer, length);
    }
#endif

    // XXX Don't need this?
    mt29f_setTimeout(fdevice, MT29F_TIMEOUT_PAGE_READ_MS);
    if (!mt29f_waitForReady(fdevice)) {
        return 0;
    }

    // Check ECC

    uint8_t statReg = mt29f_readRegister(&fdevice->io, MT29F_STATUS_REG);
    uint8_t eccCode = MT29F_STATUS_FLAG_ECC(statReg);

    switch (eccCode) {
    case 0: // Successful read, no ECC correction
        break;
    case 1: // Successful read with ECC correction
    case 2: // Uncorrectable ECC in a single page
    case 3: // Uncorrectable ECC in multiple pages
        mt29f_addError(address, eccCode);
        mt29f_deviceReset(fdevice);
        break;
    }

    return transferLength;
}

LOCAL_UNUSED_FUNCTION static int mt29f_readExtensionBytes(flashDevice_t *fdevice, uint32_t address, uint8_t *buffer, int length)
{

    if (!mt29f_waitForReady(fdevice)) {
        return 0;
    }

    mt29f_performCommandWithPageAddress(&fdevice->io, MT29F_INSTRUCTION_PAGE_READ, MT29F_LINEAR_TO_PAGE(address));

    uint32_t column = 2048;

    if (fdevice->io.mode == FLASHIO_SPI) {
        extDevice_t *dev = fdevice->io.handle.dev;

        uint8_t cmd[4];
        cmd[0] = MT29F_INSTRUCTION_READ_FROM_CACHE_X1;
        cmd[1] = (column >> 8) & 0xff;
        cmd[2] = (column >> 0) & 0xff;
        cmd[3] = 0;

        busSegment_t segments[] = {
                {.u.buffers = {cmd, NULL}, sizeof(cmd), false, NULL},
                {.u.buffers = {NULL, buffer}, length, true, NULL},
                {.u.link = {NULL, NULL}, 0, true, NULL},
        };

        // Ensure any prior DMA has completed before continuing
        spiWait(dev);

        spiSequence(dev, &segments[0]);

        // Block pending completion of SPI access
        spiWait(dev);

    }
#ifdef USE_QUADSPI
    else if (fdevice->io.mode == FLASHIO_QUADSPI) {
        extDevice_t *dev = fdevice->io.handle.dev;

        quadSpiReceiveWithAddress1LINE(dev, MT29F_INSTRUCTION_READ_FROM_CACHE_X1, 8, column, MT29F_COLUMN_ADDRESS_SIZE, buffer, length);
    }
#endif

    mt29f_setTimeout(fdevice, MT29F_TIMEOUT_PAGE_READ_MS);

    return length;
}

/**
 * Fetch information about the detected flash chip layout.
 *
 * Can be called before calling mt29f_init() (the result would have totalSize = 0).
 */
static const flashGeometry_t* mt29f_getGeometry(flashDevice_t *fdevice)
{
    return &fdevice->geometry;
}

const flashVTable_t mt29f_vTable = {
    .configure = mt29f_configure,
    .isReady = mt29f_isReady,
    .waitForReady = mt29f_waitForReady,
    .eraseSector = mt29f_eraseSector,
    .eraseCompletely = mt29f_eraseCompletely,
    .pageProgramBegin = mt29f_pageProgramBegin,
    .pageProgramContinue = mt29f_pageProgramContinue,
    .pageProgramFinish = mt29f_pageProgramFinish,
    .pageProgram = mt29f_pageProgram,
    .flush = mt29f_flush,
    .readBytes = mt29f_readBytes,
    .getGeometry = mt29f_getGeometry,
};

static void mt29f_deviceInit(flashDevice_t *flashdev)
{
    UNUSED(flashdev);
}
#endif
