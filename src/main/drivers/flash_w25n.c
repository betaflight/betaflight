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

#include "platform.h"

#include "build/debug.h"

#ifdef USE_FLASH_W25N

#include "common/maths.h"
#include "drivers/bus_spi.h"
#include "drivers/flash.h"
#include "drivers/flash_impl.h"
#include "drivers/io.h"
#include "drivers/time.h"

#include "pg/flash.h"

#include "flash_w25n.h"

#define W25N_INSTRUCTION_RDID                       SPIFLASH_INSTRUCTION_RDID
#define W25N_INSTRUCTION_PAGE_DATA_READ             0x13
#define W25N_INSTRUCTION_READ_DATA                  0x03
#define W25N_INSTRUCTION_READ_STATUS_REG            0x05
#define W25N_INSTRUCTION_WRITE_STATUS_REG           0x01
#define W25N_INSTRUCTION_WRITE_ENABLE               0x06
#define W25N_INSTRUCTION_WRITE_DISABLE              0x04
#define W25N_INSTRUCTION_LOAD_PROGRAM_DATA          0x02
#define W25N_INSTRUCTION_RANDOM_LOAD_PROGRAM_DATA   0x84
#define W25N_INSTRUCTION_PROGRAM_EXECUTE            0x10
#define W25N_INSTRUCTION_BLOCK_ERASE                0xD8

#define W25N_STATUS_Register_3                      0xC0

#define W25N_STATUS_FLAG_WRITE_ENABLE_LATCH         0x01
#define W25N_STATUS_FLAG_BUSY                       0x02

// Format is manufacturer, memory type, then capacity

// The timeout we expect between being able to issue page program instructions
#define DEFAULT_TIMEOUT_MILLIS                      6

#define READ_PAGE_DATA_TIMEOUT_MILLIS               60
#define PAGE_PROGRAM_TIMEOUT_MILLIS                 700
#define PROGRAM_EXECUTE_TIMEOUT_MILLIS              10
#define BLOCK_ERASE_TIMEOUT_MILLIS                  10000

#define W25N_PAGESIZE                               2048

STATIC_ASSERT(W25N_PAGESIZE <= FLASH_MAX_PAGE_SIZE, W25N_PAGESIZE_too_small);

const flashVTable_t w25n_vTable;

static void w25n_disable(busDevice_t *bus)
{
    IOHi(bus->busdev_u.spi.csnPin);
    __NOP();
}

static void w25n_enable(busDevice_t *bus)
{
    __NOP();
    IOLo(bus->busdev_u.spi.csnPin);
}

static void w25n_transfer(busDevice_t *bus, const uint8_t *txData, uint8_t *rxData, int len)
{
    w25n_enable(bus);
    spiTransfer(bus->busdev_u.spi.instance, txData, rxData, len);
    w25n_disable(bus);
}

/**
 * Send the given command byte to the device.
 */
static void w25n_performOneByteCommand(busDevice_t *bus, uint8_t command)
{
    w25n_enable(bus);

    spiTransferByte(bus->busdev_u.spi.instance, command);

    w25n_disable(bus);
}

/**
 * The flash requires this write enable command to be sent before commands that would cause
 * a write like program and erase.
 */
static void w25n_writeEnable(flashDevice_t *fdevice)
{
    w25n_performOneByteCommand(fdevice->busdev, W25N_INSTRUCTION_WRITE_ENABLE);

    // Assume that we're about to do some writing, so the device is just about to become busy
    fdevice->couldBeBusy = true;
}

static uint8_t w25n_readStatus(busDevice_t *bus)
{
    const uint8_t command[3] = { W25N_INSTRUCTION_READ_STATUS_REG, W25N_STATUS_Register_3, 0 };
    uint8_t in[3];

    w25n_transfer(bus, command, in, sizeof(command));

    return in[2];
}

static bool w25n_isReady(flashDevice_t *fdevice)
{
    // If couldBeBusy is false, don't bother to poll the flash chip for its status
    fdevice->couldBeBusy = fdevice->couldBeBusy && ((w25n_readStatus(fdevice->busdev) & W25N_STATUS_FLAG_WRITE_ENABLE_LATCH) != 0);

    return !fdevice->couldBeBusy;
}

static bool w25n_waitForReady(flashDevice_t *fdevice, uint32_t timeoutMillis)
{
    uint32_t time = millis();
    while (!w25n_isReady(fdevice)) {
        if (millis() - time > timeoutMillis) {
            return false;
        }
    }

    return true;
}

/**
 * Read chip identification and geometry information (into global `geometry`).
 *
 * Returns true if we get valid ident, false if something bad happened like there is no W25N.
 */

bool w25n_detect(flashDevice_t *fdevice, uint32_t chipID)
{
    switch (chipID) {
    case JEDEC_ID_WINBOND_W25N01GV:
        fdevice->geometry.sectors = 1024;
        fdevice->geometry.pagesPerSector = 64;
        break;
    default:
        // Unsupported chip or not an SPI NAND flash
        fdevice->geometry.sectors = 0;
        fdevice->geometry.pagesPerSector = 0;
        fdevice->geometry.sectorSize = 0;
        fdevice->geometry.totalSize = 0;
        return false;
    }

    fdevice->geometry.flashType = FLASH_TYPE_NAND;
    fdevice->geometry.pageSize = W25N_PAGESIZE;
    fdevice->geometry.sectorSize = fdevice->geometry.pagesPerSector * fdevice->geometry.pageSize;
    fdevice->geometry.totalSize = fdevice->geometry.sectorSize * fdevice->geometry.sectors;

    fdevice->couldBeBusy = true; // Just for luck we'll assume the chip could be busy even though it isn't specced to be
    fdevice->vTable = &w25n_vTable;
    spiSetDivisor(fdevice->busdev->busdev_u.spi.instance, SPI_CLOCK_ULTRAFAST);
    return true;
}

static void w25n_setPageAddress(uint8_t *buf, uint32_t address)
{
    *buf++ = (address >> 20) & 0xff;
    *buf = (address >> 12) & 0xff;
}
static void w25n_setColumnAddress(uint8_t *buf, uint32_t address)
{
    *buf++ = (address >> 8) & 0x07;
    *buf = address & 0xff;
}

/**
 * Erase a sector full of bytes to all 1's at the given byte offset in the flash chip.
 */
static void w25n_eraseSector(flashDevice_t *fdevice, uint32_t address)
{
    uint8_t out[4] = { W25N_INSTRUCTION_BLOCK_ERASE };

    w25n_setPageAddress(&out[2], address);

    w25n_waitForReady(fdevice, BLOCK_ERASE_TIMEOUT_MILLIS);

    w25n_writeEnable(fdevice);

    w25n_transfer(fdevice->busdev, out, NULL, sizeof(out));
}

static void w25n_eraseCompletely(flashDevice_t *fdevice)
{
    uint8_t out[4] = { W25N_INSTRUCTION_BLOCK_ERASE };

    for (int page = 0 ; page < fdevice->geometry.sectors ; page++) {

        out[2] = (page >> 8) & 0x07;
        out[3] = page & 0xff;

        w25n_waitForReady(fdevice, BLOCK_ERASE_TIMEOUT_MILLIS);

        w25n_writeEnable(fdevice);

        w25n_transfer(fdevice->busdev, out, NULL, sizeof(out));
    }
}

static void w25n_pageProgramBegin(flashDevice_t *fdevice, uint32_t address)
{
    UNUSED(fdevice);

    fdevice->currentWriteAddress = address;
}

static void w25n_pageProgramContinue(flashDevice_t *fdevice, const uint8_t *data, int length)
{
    static int lastpage = -1;   // actual flash page

    int actualpage = (fdevice->currentWriteAddress >> 12) & 0xffff;

    uint8_t programCmd[3];

    if (actualpage != lastpage) {
        uint8_t executeCmd[4] = { W25N_INSTRUCTION_PROGRAM_EXECUTE };

        w25n_setPageAddress(&executeCmd[2], fdevice->currentWriteAddress);

        w25n_waitForReady(fdevice, DEFAULT_TIMEOUT_MILLIS);

        w25n_enable(fdevice->busdev);

        spiTransfer(fdevice->busdev->busdev_u.spi.instance, executeCmd, NULL, sizeof(executeCmd));

        w25n_disable(fdevice->busdev);

        lastpage = actualpage;

        programCmd[0] = W25N_INSTRUCTION_LOAD_PROGRAM_DATA;
    } else {
        programCmd[0] = W25N_INSTRUCTION_RANDOM_LOAD_PROGRAM_DATA;
    }

    w25n_setColumnAddress(&programCmd[1], fdevice->currentWriteAddress);

    w25n_waitForReady(fdevice, PAGE_PROGRAM_TIMEOUT_MILLIS);

    w25n_writeEnable(fdevice);

    w25n_enable(fdevice->busdev);

    spiTransfer(fdevice->busdev->busdev_u.spi.instance, programCmd, NULL, sizeof(programCmd));

    spiTransfer(fdevice->busdev->busdev_u.spi.instance, data, NULL, length);

    w25n_disable(fdevice->busdev);

    fdevice->currentWriteAddress += length;
}

static void w25n_pageProgramFinish(flashDevice_t *fdevice)
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
static void w25n_pageProgram(flashDevice_t *fdevice, uint32_t address, const uint8_t *data, int length)
{
    w25n_pageProgramBegin(fdevice, address);

    w25n_pageProgramContinue(fdevice, data, length);

    w25n_pageProgramFinish(fdevice);
}

/**
 * Read `length` bytes into the provided `buffer` from the flash starting from the given `address` (which need not lie
 * on a page boundary).
 *
 * Waits up to DEFAULT_TIMEOUT_MILLIS milliseconds for the flash to become ready before reading.
 *
 * The number of bytes actually read is returned, which can be zero if an error or timeout occurred.
 */
static int w25n_readBytes(flashDevice_t *fdevice, uint32_t address, uint8_t *buffer, int length)
{
    static int lastpage = -1;   // actual flash page
    int rlen;                   // remaining length
    int tlen;                   // transfer length for a round
    int rbytes;                 // read bytes

    // Divide a read that spans multiple flash blocks.

    for (rlen = length; rlen; rlen -= tlen) {
        int actualpage = (address >> 12) & 0xffff;
        int columnAddress = address % W25N_PAGESIZE;
        tlen = MIN(columnAddress + rlen, W25N_PAGESIZE) - columnAddress;

        if (actualpage != lastpage) {
            uint8_t pageCmd[4] = { W25N_INSTRUCTION_PAGE_DATA_READ };

            w25n_setPageAddress(&pageCmd[2], address);

            if (!w25n_waitForReady(fdevice, PAGE_PROGRAM_TIMEOUT_MILLIS)) {
                return 0;
            }

            w25n_enable(fdevice->busdev);

            spiTransfer(fdevice->busdev->busdev_u.spi.instance, pageCmd, NULL, sizeof(pageCmd));

            w25n_disable(fdevice->busdev);

        }

        uint8_t readCmd[3] = { W25N_INSTRUCTION_READ_DATA };

        w25n_setColumnAddress(&readCmd[1], address);

        if (!w25n_waitForReady(fdevice, DEFAULT_TIMEOUT_MILLIS)) {
            return 0;
        }

        w25n_enable(fdevice->busdev);

        spiTransfer(fdevice->busdev->busdev_u.spi.instance, readCmd, NULL, sizeof(readCmd));
        rbytes = spiTransfer(fdevice->busdev->busdev_u.spi.instance, NULL, buffer, tlen);

        w25n_disable(fdevice->busdev);

        if (!rbytes) {
            return 0;
        }

        lastpage = actualpage;
        address += tlen;
        buffer += tlen;
    }

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
    .isReady = w25n_isReady,
    .waitForReady = w25n_waitForReady,
    .eraseSector = w25n_eraseSector,
    .eraseCompletely = w25n_eraseCompletely,
    .pageProgramBegin = w25n_pageProgramBegin,
    .pageProgramContinue = w25n_pageProgramContinue,
    .pageProgramFinish = w25n_pageProgramFinish,
    .pageProgram = w25n_pageProgram,
    .readBytes = w25n_readBytes,
    .getGeometry = w25n_getGeometry,
};
#endif
