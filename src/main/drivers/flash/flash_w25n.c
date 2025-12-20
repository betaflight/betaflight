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
 * Author: jflyper
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_FLASH_W25N

#include "drivers/flash/flash.h"
#include "drivers/flash/flash_impl.h"
#include "drivers/flash/flash_w25n.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_quadspi.h"
#include "drivers/io.h"
#include "drivers/time.h"

// BB replacement area
#define W25N_BB_MARKER_BLOCKS           1
#define W25N_BB_REPLACEMENT_BLOCKS      20
#define W25N_BB_MANAGEMENT_BLOCKS       (W25N_BB_REPLACEMENT_BLOCKS + W25N_BB_MARKER_BLOCKS)
// blocks are zero-based index; when negative, it is from end of flash
#define W25N_BB_MANAGEMENT_START_BLOCK  (-W25N_BB_MANAGEMENT_BLOCKS)

// Instructions

#define W25N_INSTRUCTION_RDID             0x9F
#define W25N_INSTRUCTION_DEVICE_RESET     0xFF
#define W25N_INSTRUCTION_READ_STATUS_REG  0x05
#define W25N_INSTRUCTION_READ_STATUS_ALTERNATE_REG  0x0F
#define W25N_INSTRUCTION_WRITE_STATUS_REG 0x01
#define W25N_INSTRUCTION_WRITE_STATUS_ALTERNATE_REG 0x1F
#define W25N_INSTRUCTION_WRITE_ENABLE     0x06
#define W25N_INSTRUCTION_DIE_SELECT       0xC2
#define W25N_INSTRUCTION_BLOCK_ERASE      0xD8
#define W25N_INSTRUCTION_READ_BBM_LUT     0xA5
#define W25N_INSTRUCTION_BB_MANAGEMENT    0xA1
#define W25N_INSTRUCTION_PROGRAM_DATA_LOAD        0x02
#define W25N_INSTRUCTION_RANDOM_PROGRAM_DATA_LOAD 0x84
#define W25N_INSTRUCTION_PROGRAM_EXECUTE  0x10
#define W25N_INSTRUCTION_PAGE_DATA_READ   0x13
#define W25N_INSTRUCTION_READ_DATA        0x03
#define W25N_INSTRUCTION_FAST_READ        0x1B
#define W25N_INSTRUCTION_FAST_READ_QUAD_OUTPUT 0x6B

// Config/status register addresses
#define W25N_PROT_REG 0xA0
#define W25N_CONF_REG 0xB0
#define W25N_STAT_REG 0xC0

// Bits in config/status register 1 (W25N_PROT_REG)
#define W25N_PROT_CLEAR                (0)
#define W25N_PROT_SRP1_ENABLE          (1 << 0)
#define W25N_PROT_WP_E_ENABLE          (1 << 1)
#define W25N_PROT_TB_ENABLE            (1 << 2)
#define W25N_PROT_PB0_ENABLE           (1 << 3)
#define W25N_PROT_PB1_ENABLE           (1 << 4)
#define W25N_PROT_PB2_ENABLE           (1 << 5)
#define W25N_PROT_PB3_ENABLE           (1 << 6)
#define W25N_PROT_SRP2_ENABLE          (1 << 7)

// Bits in config/status register 2 (W25N_CONF_REG)
#define W25N_CONFIG_ECC_ENABLE         (1 << 4)
#define W25N_CONFIG_BUFFER_READ_MODE   (1 << 3)

// Bits in config/status register 3 (W25N_STATREG)
#define W25N_STATUS_BBM_LUT_FULL       (1 << 6)
#define W25N_STATUS_FLAG_ECC_POS       4
#define W25N_STATUS_FLAG_ECC_MASK      ((1 << 5)|(1 << 4))
#define W25N_STATUS_FLAG_ECC(status)   (((status) & W25N_STATUS_FLAG_ECC_MASK) >> 4)
#define W25N_STATUS_PROGRAM_FAIL       (1 << 3)
#define W25N_STATUS_ERASE_FAIL         (1 << 2)
#define W25N_STATUS_FLAG_WRITE_ENABLED (1 << 1)
#define W25N_STATUS_FLAG_BUSY          (1 << 0)

#define W25N_BBLUT_TABLE_ENTRY_COUNT     20
#define W25N_BBLUT_TABLE_ENTRY_SIZE      4  // in bytes

// Bits in LBA for BB LUT
#define W25N_BBLUT_STATUS_ENABLED (1 << 15)
#define W25N_BBLUT_STATUS_INVALID (1 << 14)
#define W25N_BBLUT_STATUS_MASK    (W25N_BBLUT_STATUS_ENABLED | W25N_BBLUT_STATUS_INVALID)

// Some useful defs and macros
#define W25N_PAGE_SIZE fdevice->geometry.pageSize
#define W25N_LINEAR_TO_COLUMN(laddr) ((laddr) % W25N_PAGE_SIZE)
#define W25N_LINEAR_TO_PAGE(laddr) ((laddr) / W25N_PAGE_SIZE)
#define W25N_LINEAR_TO_BLOCK(laddr) (W25N_LINEAR_TO_PAGE(laddr) / fdevice->geometry.pagesPerSector)
#define W25N_BLOCK_TO_PAGE(block) ((block) * fdevice->geometry.pagesPerSector)
#define W25N_BLOCK_TO_LINEAR(block) (W25N_BLOCK_TO_PAGE(block) * W25N_PAGE_SIZE)

// IMPORTANT: Timeout values are currently required to be set to the highest value required by any of the supported flash chips by this driver

// The timeout values (2ms minimum to avoid 1 tick advance in consecutive calls to millis).
#define W25N_TIMEOUT_PAGE_READ_MS        2   // tREmax = 60us (ECC enabled)
#define W25N_TIMEOUT_PAGE_PROGRAM_MS     2   // tPPmax = 700us
#define W25N_TIMEOUT_BLOCK_ERASE_MS      15  // tBEmax = 10ms
#define W25N_TIMEOUT_RESET_MS            2   // tRSTmax = 500us

// Sizes (in bits)
#define W25N_STATUS_REGISTER_SIZE        8
#define W25N_STATUS_PAGE_ADDRESS_SIZE    16
#define W25N_STATUS_COLUMN_ADDRESS_SIZE  16

typedef struct bblut_s {
    uint16_t pba;
    uint16_t lba;
} bblut_t;

// Table of recognised FLASH devices
struct {
    uint32_t        jedecID;
    flashSector_t   sectors;
    uint16_t        pagesPerSector;
    uint16_t        pageSize;
} w25nFlashConfig[] = {
    // Winbond W25N01GV
    // Datasheet: https://www.winbond.com/resource-files/W25N01GV%20Rev%20R%20070323.pdf
    { 0xEFAA21, 2048, 64, 1024 },
    // Winbond W25N02KV
    // Datasheet: https://www.winbond.com/resource-files/W25N02KVxxIRU_Datasheet_RevM.pdf
    { 0xEFAA22, 2048, 64, 2048 },
    { 0, 0, 0, 0 },
};

static bool w25n_waitForReady(flashDevice_t *fdevice);

static void w25n_setTimeout(flashDevice_t *fdevice, uint32_t timeoutMillis)
{
    uint32_t now = millis();
    fdevice->timeoutAt = now + timeoutMillis;
}

/**
 * Send the given command byte to the device.
 */
static void w25n_performOneByteCommand(flashDeviceIO_t *io, uint8_t command)
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

static void w25n_performCommandWithPageAddress(flashDeviceIO_t *io, uint8_t command, uint32_t pageAddress)
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

        quadSpiInstructionWithAddress1LINE(dev, command, 0, pageAddress & 0xffff, W25N_STATUS_PAGE_ADDRESS_SIZE + 8);
    }
#endif
}

static uint8_t w25n_readRegister(flashDeviceIO_t *io, uint8_t reg)
{
    if (io->mode == FLASHIO_SPI) {
        extDevice_t *dev = io->handle.dev;

        uint8_t cmd[3] = { W25N_INSTRUCTION_READ_STATUS_REG, reg, 0 };
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

        uint8_t in[W25N_STATUS_REGISTER_SIZE / 8];
        quadSpiReceiveWithAddress1LINE(dev, W25N_INSTRUCTION_READ_STATUS_REG, 0, reg, W25N_STATUS_REGISTER_SIZE, in, sizeof(in));

        return in[0];
    }
#endif
    return 0;
}

static void w25n_writeRegister(flashDeviceIO_t *io, uint8_t reg, uint8_t data)
{
    if (io->mode == FLASHIO_SPI) {
        extDevice_t *dev = io->handle.dev;
        uint8_t cmd[3] = { W25N_INSTRUCTION_WRITE_STATUS_REG, reg, data };

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

       quadSpiTransmitWithAddress1LINE(dev, W25N_INSTRUCTION_WRITE_STATUS_REG, 0, reg, W25N_STATUS_REGISTER_SIZE, &data, 1);
   }
#endif
}

static void w25n_deviceReset(flashDevice_t *fdevice)
{
    flashDeviceIO_t *io = &fdevice->io;

    w25n_performOneByteCommand(io, W25N_INSTRUCTION_DEVICE_RESET);

    w25n_setTimeout(fdevice, W25N_TIMEOUT_RESET_MS);
    w25n_waitForReady(fdevice);

    // Protection for upper 1/32 (BP[3:0] = 0101, TB=0), WP-E on; to protect bad block replacement area
    // DON'T DO THIS. This will prevent writes through the bblut as well.
    // w25n_writeRegister(dev, W25N_PROT_REG, W25N_PROT_PB0_ENABLE|W25N_PROT_PB2_ENABLE|W25N_PROT_WP_E_ENABLE);

    // No protection, WP-E off, WP-E prevents use of IO2
    w25n_writeRegister(io, W25N_PROT_REG, W25N_PROT_CLEAR);

    // Buffered read mode (BUF = 1), ECC enabled (ECC = 1)
    w25n_writeRegister(io, W25N_CONF_REG, W25N_CONFIG_ECC_ENABLE|W25N_CONFIG_BUFFER_READ_MODE);
}

static bool w25n_isReady(flashDevice_t *fdevice)
{
    // If we're waiting on DMA completion, then SPI is busy
    if (fdevice->io.mode == FLASHIO_SPI) {
        if (fdevice->io.handle.dev->bus->useDMA && spiIsBusy(fdevice->io.handle.dev)) {
            return false;
        }
    }

    // Irrespective of the current state of fdevice->couldBeBusy read the status or device blocks

    // Poll the FLASH device to see if it's busy
    fdevice->couldBeBusy = ((w25n_readRegister(&fdevice->io, W25N_STAT_REG) & W25N_STATUS_FLAG_BUSY) != 0);

    return !fdevice->couldBeBusy;
}

static bool w25n_waitForReady(flashDevice_t *fdevice)
{
    while (!w25n_isReady(fdevice)) {
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
static void w25n_writeEnable(flashDevice_t *fdevice)
{
    w25n_performOneByteCommand(&fdevice->io, W25N_INSTRUCTION_WRITE_ENABLE);

    // Assume that we're about to do some writing, so the device is just about to become busy
    fdevice->couldBeBusy = true;
}

const flashVTable_t w25n_vTable;

bool w25n_identify(flashDevice_t *fdevice, uint32_t jedecID)
{
    flashGeometry_t *geometry = &fdevice->geometry;
    uint8_t index;

    for (index = 0; w25nFlashConfig[index].jedecID; index++) {
        if (w25nFlashConfig[index].jedecID == jedecID) {
            geometry->sectors = w25nFlashConfig[index].sectors;
            geometry->pagesPerSector = w25nFlashConfig[index].pagesPerSector;
            geometry->pageSize = w25nFlashConfig[index].pageSize;
            break;
        }
    }

    if (w25nFlashConfig[index].jedecID == 0) {
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

    const uint32_t managementStartBlock = W25N_BB_MANAGEMENT_START_BLOCK >= 0 ? W25N_BB_MANAGEMENT_START_BLOCK : fdevice->geometry.sectors + W25N_BB_MANAGEMENT_START_BLOCK;
    flashPartitionSet(FLASH_PARTITION_TYPE_BADBLOCK_MANAGEMENT,
            managementStartBlock,
            managementStartBlock + W25N_BB_MANAGEMENT_BLOCKS - 1);

    fdevice->couldBeBusy = true; // Just for luck we'll assume the chip could be busy even though it isn't specced to be
    fdevice->vTable = &w25n_vTable;

    return true;
}

static void w25n_deviceInit(flashDevice_t *flashdev);

static void w25n_configure(flashDevice_t *fdevice, uint32_t configurationFlags)
{
    if (configurationFlags & FLASH_CF_SYSTEM_IS_MEMORY_MAPPED) {
        return;
    }

    w25n_deviceReset(fdevice);

    // Upper 4MB (32 blocks * 128KB/block) will be used for bad block replacement area.

    // Blocks in this area are only written through bad block LUT,
    // and factory written bad block marker in unused blocks are retained.

    // When a replacement block is required,
    // (1) "Read BB LUT" command is used to obtain the last block mapped,
    // (2) blocks after the last block is scanned for a good block,
    // (3) the first good block is used for replacement, and the BB LUT is updated.

    // There are only 20 BB LUT entries, and there are 32 replacement blocks.
    // There will be a least chance of running out of replacement blocks.
    // If it ever run out, the device becomes unusable.

    if (fdevice->io.mode == FLASHIO_SPI) {    // Need to set clock speed for 8kHz logging support with SPI
        spiSetClkDivisor(fdevice->io.handle.dev, spiCalculateDivider(100000000));
    }

    w25n_deviceInit(fdevice);
}

/**
 * Erase a sector full of bytes to all 1's at the given byte offset in the flash chip.
 */
static void w25n_eraseSector(flashDevice_t *fdevice, uint32_t address)
{

    w25n_waitForReady(fdevice);

    w25n_writeEnable(fdevice);

    w25n_performCommandWithPageAddress(&fdevice->io, W25N_INSTRUCTION_BLOCK_ERASE, W25N_LINEAR_TO_PAGE(address));

    w25n_setTimeout(fdevice, W25N_TIMEOUT_BLOCK_ERASE_MS);
}

//
// W25N01G does not support full chip erase.
// Call eraseSector repeatedly.

static void w25n_eraseCompletely(flashDevice_t *fdevice)
{
    for (uint32_t block = 0; block < fdevice->geometry.sectors; block++) {
        w25n_eraseSector(fdevice, W25N_BLOCK_TO_LINEAR(block));
    }
}

#ifdef USE_QUADSPI
static void w25n_programDataLoad(flashDevice_t *fdevice, uint16_t columnAddress, const uint8_t *data, int length)
{

    w25n_waitForReady(fdevice);

    if (fdevice->io.mode == FLASHIO_SPI) {
        extDevice_t *dev = fdevice->io.handle.dev;
        uint8_t cmd[] = { W25N_INSTRUCTION_PROGRAM_DATA_LOAD, columnAddress >> 8, columnAddress & 0xff };

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

       quadSpiTransmitWithAddress1LINE(dev, W25N_INSTRUCTION_PROGRAM_DATA_LOAD, 0, columnAddress, W25N_STATUS_COLUMN_ADDRESS_SIZE, data, length);
    }
#endif

    w25n_setTimeout(fdevice, W25N_TIMEOUT_PAGE_PROGRAM_MS);
}

static void w25n_randomProgramDataLoad(flashDevice_t *fdevice, uint16_t columnAddress, const uint8_t *data, int length)
{
    uint8_t cmd[] = { W25N_INSTRUCTION_RANDOM_PROGRAM_DATA_LOAD, columnAddress >> 8, columnAddress & 0xff };

    w25n_waitForReady(fdevice);

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

        quadSpiTransmitWithAddress1LINE(dev, W25N_INSTRUCTION_RANDOM_PROGRAM_DATA_LOAD, 0, columnAddress, W25N_STATUS_COLUMN_ADDRESS_SIZE, data, length);
     }
#endif

    w25n_setTimeout(fdevice, W25N_TIMEOUT_PAGE_PROGRAM_MS);

}
#endif

static void w25n_programExecute(flashDevice_t *fdevice, uint32_t pageAddress)
{
    w25n_waitForReady(fdevice);

    w25n_performCommandWithPageAddress(&fdevice->io, W25N_INSTRUCTION_PROGRAM_EXECUTE, pageAddress);

    w25n_setTimeout(fdevice, W25N_TIMEOUT_PAGE_PROGRAM_MS);
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
static busStatus_e w25n_callbackReady(uintptr_t arg)
{
    flashDevice_t *fdevice = (flashDevice_t *)arg;
    extDevice_t *dev = fdevice->io.handle.dev;

    uint8_t readyPoll = dev->bus->curSegment->u.buffers.rxData[2];

    if (readyPoll & W25N_STATUS_FLAG_BUSY) {
        return BUS_BUSY;
    }

    // Bus is now known not to be busy
    fdevice->couldBeBusy = false;

    return BUS_READY;
}

#ifdef USE_QUADSPI
static bool isProgramming = false;

static void w25n_pageProgramBegin(flashDevice_t *fdevice, uint32_t address, void (*callback)(uintptr_t arg))
{
    fdevice->callback = callback;

    if (bufferDirty) {
        if (address != programLoadAddress) {
            w25n_waitForReady(fdevice);

            isProgramming = false;

            w25n_writeEnable(fdevice);

            w25n_programExecute(fdevice, W25N_LINEAR_TO_PAGE(programStartAddress));

            bufferDirty = false;
            isProgramming = true;
        }
    } else {
        programStartAddress = programLoadAddress = address;
    }
}

static uint32_t w25n_pageProgramContinue(flashDevice_t *fdevice, uint8_t const **buffers, const uint32_t *bufferSizes, uint32_t bufferCount)
{
    if (bufferCount < 1) {
        fdevice->callback(0);
        return 0;
    }

    w25n_waitForReady(fdevice);

    w25n_writeEnable(fdevice);

    isProgramming = false;

    if (!bufferDirty) {
        w25n_programDataLoad(fdevice, W25N_LINEAR_TO_COLUMN(programLoadAddress), buffers[0], bufferSizes[0]);
    } else {
        w25n_randomProgramDataLoad(fdevice, W25N_LINEAR_TO_COLUMN(programLoadAddress), buffers[0], bufferSizes[0]);
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

static void w25n_pageProgramFinish(flashDevice_t *fdevice)
{
    if (bufferDirty && W25N_LINEAR_TO_COLUMN(programLoadAddress) == 0) {

        currentPage = W25N_LINEAR_TO_PAGE(programStartAddress); // reset page to the page being written

        w25n_programExecute(fdevice, W25N_LINEAR_TO_PAGE(programStartAddress));

        bufferDirty = false;
        isProgramming = true;

        programStartAddress = programLoadAddress;
    }
}
#else
static void w25n_pageProgramBegin(flashDevice_t *fdevice, uint32_t address, void (*callback)(uintptr_t arg))
{
    fdevice->callback = callback;
    fdevice->currentWriteAddress = address;

}

static uint32_t currentPage = UINT32_MAX;

// Called in ISR context
// A write enable has just been issued
static busStatus_e w25n_callbackWriteEnable(uintptr_t arg)
{
    flashDevice_t *fdevice = (flashDevice_t *)arg;

    // As a write has just occurred, the device could be busy
    fdevice->couldBeBusy = true;

    return BUS_READY;
}

// Called in ISR context
// Write operation has just completed
static busStatus_e w25n_callbackWriteComplete(uintptr_t arg)
{
    flashDevice_t *fdevice = (flashDevice_t *)arg;

    fdevice->currentWriteAddress += fdevice->bytesWritten;
    // Call transfer completion callback
    if (fdevice->callback) {
        fdevice->callback(fdevice->bytesWritten);
    }

    return BUS_READY;
}

static uint32_t w25n_pageProgramContinue(flashDevice_t *fdevice, uint8_t const **buffers, const uint32_t *bufferSizes, uint32_t bufferCount)
{
    if (bufferCount < 1) {
        fdevice->callback(0);
        return 0;
    }

    // The segment list cannot be in automatic storage as this routine is non-blocking
    STATIC_DMA_DATA_AUTO uint8_t readStatus[] = { W25N_INSTRUCTION_READ_STATUS_REG, W25N_STAT_REG, 0 };
    STATIC_DMA_DATA_AUTO uint8_t readyStatus[3];
    STATIC_DMA_DATA_AUTO uint8_t writeEnable[] = { W25N_INSTRUCTION_WRITE_ENABLE };
    STATIC_DMA_DATA_AUTO uint8_t progExecCmd[] = { W25N_INSTRUCTION_PROGRAM_EXECUTE, 0, 0, 0};
    STATIC_DMA_DATA_AUTO uint8_t progExecDataLoad[] = { W25N_INSTRUCTION_PROGRAM_DATA_LOAD, 0, 0};
    STATIC_DMA_DATA_AUTO uint8_t progRandomProgDataLoad[] = { W25N_INSTRUCTION_RANDOM_PROGRAM_DATA_LOAD, 0, 0};

    static busSegment_t segmentsDataLoad[] = {
        {.u.buffers = {readStatus, readyStatus}, sizeof(readStatus), true, w25n_callbackReady},
        {.u.buffers = {writeEnable, NULL}, sizeof(writeEnable), true, w25n_callbackWriteEnable},
        {.u.buffers = {progExecDataLoad, NULL}, sizeof(progExecDataLoad), false, NULL},
        {.u.link = {NULL, NULL}, 0, true, NULL},
    };

    static busSegment_t segmentsRandomDataLoad[] = {
        {.u.buffers = {readStatus, readyStatus}, sizeof(readStatus), true, w25n_callbackReady},
        {.u.buffers = {writeEnable, NULL}, sizeof(writeEnable), true, w25n_callbackWriteEnable},
        {.u.buffers = {progRandomProgDataLoad, NULL}, sizeof(progRandomProgDataLoad), false, NULL},
        {.u.link = {NULL, NULL}, 0, true, NULL},
    };

    static busSegment_t segmentsBuffer[] = {
        {.u.buffers = {NULL, NULL}, 0, true, NULL},
        {.u.link = {NULL, NULL}, 0, true, NULL},
    };

    static busSegment_t segmentsFlash[] = {
        {.u.buffers = {readStatus, readyStatus}, sizeof(readStatus), true, w25n_callbackReady},
        {.u.buffers = {writeEnable, NULL}, sizeof(writeEnable), true, w25n_callbackWriteEnable},
        {.u.buffers = {progExecCmd, NULL}, sizeof(progExecCmd), true, w25n_callbackWriteComplete},
        {.u.link = {NULL, NULL}, 0, true, NULL},
    };

    busSegment_t *programSegment;

    // Ensure any prior DMA has completed before continuing
    spiWait(fdevice->io.handle.dev);

    uint32_t columnAddress;

    if (bufferDirty) {
        columnAddress = W25N_LINEAR_TO_COLUMN(programLoadAddress);
        // Set the address and buffer details for the random data load
        progRandomProgDataLoad[1] = (columnAddress >> 8) & 0xff;
        progRandomProgDataLoad[2] = columnAddress & 0xff;
        programSegment = segmentsRandomDataLoad;
    } else {
        programStartAddress = programLoadAddress = fdevice->currentWriteAddress;
        columnAddress = W25N_LINEAR_TO_COLUMN(programLoadAddress);
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

    if (W25N_LINEAR_TO_COLUMN(programLoadAddress) == 0) {
        // Flash the loaded data
        currentPage = W25N_LINEAR_TO_PAGE(programStartAddress);

        progExecCmd[2] = (currentPage >> 8) & 0xff;
        progExecCmd[3] = currentPage & 0xff;

        spiLinkSegments(fdevice->io.handle.dev, segmentsBuffer, segmentsFlash);

        bufferDirty = false;

        programStartAddress = programLoadAddress;
    } else {
        // Callback on completion of data load
        segmentsBuffer[0].callback = w25n_callbackWriteComplete;
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

static void w25n_pageProgramFinish(flashDevice_t *fdevice)
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

static void w25n_pageProgram(flashDevice_t *fdevice, uint32_t address, const uint8_t *data, uint32_t length, void (*callback)(uintptr_t arg))
{
    w25n_pageProgramBegin(fdevice, address, callback);
    w25n_pageProgramContinue(fdevice, &data, &length, 1);
    w25n_pageProgramFinish(fdevice);
}

static void w25n_flush(flashDevice_t *fdevice)
{
    if (bufferDirty) {
        currentPage = W25N_LINEAR_TO_PAGE(programStartAddress); // reset page to the page being written

        w25n_programExecute(fdevice, W25N_LINEAR_TO_PAGE(programStartAddress));

        bufferDirty = false;
    }
}

static void w25n_addError(uint32_t address, uint8_t code)
{
    UNUSED(address);
    UNUSED(code);
}

/**
 * Read `length` bytes into the provided `buffer` from the flash starting from the given `address` (which need not lie
 * on a page boundary).
 *
 * Waits up to W25N_TIMEOUT_PAGE_READ_MS milliseconds for the flash to become ready before reading.
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

static int w25n_readBytes(flashDevice_t *fdevice, uint32_t address, uint8_t *buffer, uint32_t length)
{
    uint32_t targetPage = W25N_LINEAR_TO_PAGE(address);

    // As data is buffered before being written a flush must be performed before attempting a read
    bool was_dirty = bufferDirty;
    w25n_flush(fdevice);

    bool page_change = (currentPage != targetPage);

    if (was_dirty || page_change) {
        // if the buffer was dirty, we re-read the freshly written data, including the results of any write failures

        if (!w25n_waitForReady(fdevice)) {
            return 0;
        }

        currentPage = UINT32_MAX;

        w25n_performCommandWithPageAddress(&fdevice->io, W25N_INSTRUCTION_PAGE_DATA_READ, targetPage);

        w25n_setTimeout(fdevice, W25N_TIMEOUT_PAGE_READ_MS);
        if (!w25n_waitForReady(fdevice)) {
            return 0;
        }

        currentPage = targetPage;
    }

    uint32_t column = W25N_LINEAR_TO_COLUMN(address);
    uint16_t transferLength;

    if (length > W25N_PAGE_SIZE - column) {
        transferLength = W25N_PAGE_SIZE - column;
    } else {
        transferLength = length;
    }

    if (fdevice->io.mode == FLASHIO_SPI) {
        extDevice_t *dev = fdevice->io.handle.dev;

        uint8_t readStatus[] = { W25N_INSTRUCTION_READ_STATUS_REG, W25N_STAT_REG, 0 };
        uint8_t readyStatus[3];

        uint8_t cmd[4];
        cmd[0] = W25N_INSTRUCTION_READ_DATA;
        cmd[1] = (column >> 8) & 0xff;
        cmd[2] = (column >> 0) & 0xff;
        cmd[3] = 0;

        busSegment_t segments[] = {
                {.u.buffers = {readStatus, readyStatus}, sizeof(readStatus), true, w25n_callbackReady},
                {.u.buffers = {cmd, NULL}, sizeof(cmd), false, NULL},
                {.u.buffers = {NULL, buffer}, transferLength, true, NULL},
                {.u.link = {NULL, NULL}, 0, true, NULL},
        };

        spiSequence(dev, &segments[0]);

        // Block pending completion of SPI access
        spiWait(dev);
    }
#ifdef USE_QUADSPI
    else if (fdevice->io.mode == FLASHIO_QUADSPI) {
        extDevice_t *dev = fdevice->io.handle.dev;

        //quadSpiReceiveWithAddress1LINE(dev, W25N_INSTRUCTION_READ_DATA, 8, column, W25N_STATUS_COLUMN_ADDRESS_SIZE, buffer, transferLength);
        quadSpiReceiveWithAddress4LINES(dev, W25N_INSTRUCTION_FAST_READ_QUAD_OUTPUT, 8, column, W25N_STATUS_COLUMN_ADDRESS_SIZE, buffer, transferLength);
    }
#endif

    // XXX Don't need this?
    w25n_setTimeout(fdevice, W25N_TIMEOUT_PAGE_READ_MS);
    if (!w25n_waitForReady(fdevice)) {
        return 0;
    }

    // Check ECC

    uint8_t statReg = w25n_readRegister(&fdevice->io, W25N_STAT_REG);
    uint8_t eccCode = W25N_STATUS_FLAG_ECC(statReg);

    switch (eccCode) {
    case 0: // Successful read, no ECC correction
        break;
    case 1: // Successful read with ECC correction
        w25n_addError(address, eccCode);
        break;
    case 2: // Uncorrectable ECC in a single page
    case 3: // Uncorrectable ECC in multiple pages
        w25n_addError(address, eccCode);
        w25n_deviceReset(fdevice);
        break;
    }

    return transferLength;
}

LOCAL_UNUSED_FUNCTION static int w25n_readExtensionBytes(flashDevice_t *fdevice, uint32_t address, uint8_t *buffer, int length)
{

    if (!w25n_waitForReady(fdevice)) {
        return 0;
    }

    w25n_performCommandWithPageAddress(&fdevice->io, W25N_INSTRUCTION_PAGE_DATA_READ, W25N_LINEAR_TO_PAGE(address));

    uint32_t column = W25N_PAGE_SIZE;

    if (fdevice->io.mode == FLASHIO_SPI) {
        extDevice_t *dev = fdevice->io.handle.dev;

        uint8_t cmd[4];
        cmd[0] = W25N_INSTRUCTION_READ_DATA;
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

        quadSpiReceiveWithAddress1LINE(dev, W25N_INSTRUCTION_READ_DATA, 8, column, W25N_STATUS_COLUMN_ADDRESS_SIZE, buffer, length);
    }
#endif

    w25n_setTimeout(fdevice, W25N_TIMEOUT_PAGE_READ_MS);

    return length;
}

/**
 * Fetch information about the detected flash chip layout.
 *
 * Can be called before calling w25n_init() (the result would have totalSize = 0).
 */
static const flashGeometry_t* w25n_getGeometry(flashDevice_t *fdevice)
{
    return &fdevice->geometry;
}

const flashVTable_t w25n_vTable = {
    .configure = w25n_configure,
    .isReady = w25n_isReady,
    .waitForReady = w25n_waitForReady,
    .eraseSector = w25n_eraseSector,
    .eraseCompletely = w25n_eraseCompletely,
    .pageProgramBegin = w25n_pageProgramBegin,
    .pageProgramContinue = w25n_pageProgramContinue,
    .pageProgramFinish = w25n_pageProgramFinish,
    .pageProgram = w25n_pageProgram,
    .flush = w25n_flush,
    .readBytes = w25n_readBytes,
    .getGeometry = w25n_getGeometry,
};

typedef volatile struct cb_context_s {
    flashDevice_t *fdevice;
    bblut_t *bblut;
    int lutsize;
    int lutindex;
} cb_context_t;

// Called in ISR context
// Read of BBLUT entry has just completed
static busStatus_e w25n_readBBLUTCallback(uintptr_t arg)
{
    cb_context_t *cb_context = (cb_context_t *)arg;
    flashDevice_t *fdevice = cb_context->fdevice;
    uint8_t *rxData = fdevice->io.handle.dev->bus->curSegment->u.buffers.rxData;

    cb_context->bblut->pba = (rxData[0] << 16)|rxData[1];
    cb_context->bblut->lba = (rxData[2] << 16)|rxData[3];

    if (++cb_context->lutindex < cb_context->lutsize) {
        cb_context->bblut++;
        return BUS_BUSY; // Repeat the operation
    }

    return BUS_READY; // All done
}

LOCAL_UNUSED_FUNCTION static void w25n_readBBLUT(flashDevice_t *fdevice, bblut_t *bblut, int lutsize)
{
    UNUSED(bblut);
    UNUSED(lutsize);

    uint8_t in[4];

    if (fdevice->io.mode == FLASHIO_SPI) {
        extDevice_t *dev = fdevice->io.handle.dev;

        uint8_t cmd[4];

        cmd[0] = W25N_INSTRUCTION_READ_BBM_LUT;
        cmd[1] = 0;

        busSegment_t segments[] = {
                {.u.buffers = {cmd, NULL}, sizeof(cmd), false, NULL},
                {.u.buffers = {NULL, in}, sizeof(in), true, w25n_readBBLUTCallback},
                {.u.link = {NULL, NULL}, 0, true, NULL},
        };

        spiSequence(dev, &segments[0]);

        // Block pending completion of SPI access
        spiWait(dev);
    }
#ifdef USE_QUADSPI
    else if (fdevice->io.mode == FLASHIO_QUADSPI) {
        extDevice_t *dev = fdevice->io.handle.dev;

        // Note: Using HAL QuadSPI there doesn't appear to be a way to send 2 bytes, then blocks of 4 bytes, while keeping the CS line LOW
        // thus, we have to read the entire BBLUT in one go and process the result.

        uint8_t bblutBuffer[W25N_BBLUT_TABLE_ENTRY_COUNT * W25N_BBLUT_TABLE_ENTRY_SIZE];
        quadSpiReceive1LINE(dev, W25N_INSTRUCTION_READ_BBM_LUT, 8, bblutBuffer, sizeof(bblutBuffer));

        for (int i = 0, offset = 0 ; i < lutsize ; i++, offset += 4) {
            if (i < W25N_BBLUT_TABLE_ENTRY_COUNT) {
                bblut[i].pba = (in[offset + 0] << 16)|in[offset + 1];
                bblut[i].lba = (in[offset + 2] << 16)|in[offset + 3];
            }
        }
    }
#endif
}

LOCAL_UNUSED_FUNCTION static void w25n_writeBBLUT(flashDevice_t *fdevice, uint16_t lba, uint16_t pba)
{
    w25n_waitForReady(fdevice);

    if (fdevice->io.mode == FLASHIO_SPI) {
        extDevice_t *dev = fdevice->io.handle.dev;

        uint8_t cmd[5] = { W25N_INSTRUCTION_BB_MANAGEMENT, lba >> 8, lba, pba >> 8, pba };

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
    else if (fdevice->io.mode == FLASHIO_QUADSPI) {
        extDevice_t *dev = fdevice->io.handle.dev;

        uint8_t data[4] = { lba >> 8, lba, pba >> 8, pba };
        quadSpiInstructionWithData1LINE(dev, W25N_INSTRUCTION_BB_MANAGEMENT, 0, data, sizeof(data));
    }
#endif

    w25n_setTimeout(fdevice, W25N_TIMEOUT_PAGE_PROGRAM_MS);
}

static void w25n_deviceInit(flashDevice_t *flashdev)
{
    UNUSED(flashdev);
}
#endif
