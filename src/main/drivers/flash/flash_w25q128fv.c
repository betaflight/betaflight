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
 * Author: Dominic Clifton - Initial implementation and testing.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_FLASH_W25Q128FV) && (defined(USE_QUADSPI) || defined(USE_OCTOSPI))

#define USE_FLASH_WRITES_USING_4LINES
#define USE_FLASH_READS_USING_4LINES

#include "build/debug.h"
#include "common/utils.h"

#include "drivers/time.h"
#include "drivers/flash/flash.h"
#include "drivers/flash/flash_impl.h"
#include "drivers/flash/flash_w25q128fv.h"
#include "drivers/bus_quadspi.h"
#include "drivers/bus_octospi.h"
#include "drivers/bus.h"

// JEDEC ID
#define JEDEC_ID_WINBOND_W25Q128FV_SPI          0xEF4018
#define JEDEC_ID_WINBOND_W25Q128FV_QUADSPI      0xEF6018
#define JEDEC_ID_WINBOND_W25Q128JV_QUADSPI      0xEF7018
#define JEDEC_ID_WINBOND_W25Q16JV_SPI           0xEF4015
#define JEDEC_ID_WINBOND_W25Q16JV_DTR_SPI       0xEF7015

// Device size parameters
#define W25Q128FV_PAGE_SIZE         2048
#define W25Q128FV_PAGES_PER_BLOCK   64
#define W25Q128FV_BLOCKS_PER_DIE 1024
#define W25Q128FV_BLOCK_SIZE (W25Q128FV_PAGES_PER_BLOCK * W25Q128FV_PAGE_SIZE)

// Sizes
#define W25Q128FV_STATUS_REGISTER_BITS        8
#define W25Q128FV_ADDRESS_BITS                24

// Instructions
#define W25Q128FV_INSTRUCTION_RDID             0x9F

#define W25Q128FV_INSTRUCTION_ENABLE_RESET     0x66
#define W25Q128FV_INSTRUCTION_RESET_DEVICE     0x99

#define W25Q128FV_INSTRUCTION_READ_STATUS1_REG  0x05
#define W25Q128FV_INSTRUCTION_READ_STATUS2_REG  0x35
#define W25Q128FV_INSTRUCTION_READ_STATUS3_REG  0x15

#define W25Q128FV_INSTRUCTION_WRITE_STATUS1_REG 0x01
#define W25Q128FV_INSTRUCTION_WRITE_STATUS2_REG 0x31
#define W25Q128FV_INSTRUCTION_WRITE_STATUS3_REG 0x11

#define W25Q128FV_INSTRUCTION_WRITE_ENABLE       0x06
#define W25Q128FV_INSTRUCTION_VOLATILE_WRITE_ENABLE 0x50
#define W25Q128FV_INSTRUCTION_BLOCK_ERASE_64KB   0xD8
#define W25Q128FV_INSTRUCTION_CHIP_ERASE         0xC7

#define W25Q128FV_INSTRUCTION_ENTER_QPI_MODE     0x38

#define W25Q128FV_INSTRUCTION_READ_BYTES         0x03
#define W25Q128FV_INSTRUCTION_FAST_READ          0x0B
#define W25Q128FV_INSTRUCTION_FAST_READ_QUAD_OUTPUT 0x6B

#define W25Q128FV_INSTRUCTION_PAGE_PROGRAM       0x02
#define W25Q128FV_INSTRUCTION_QUAD_PAGE_PROGRAM  0x32

#define W25Q128FV_SR1_BIT_WRITE_IN_PROGRESS     (1 << 0)
#define W25Q128FV_SR1_BIT_WRITE_ENABLED         (1 << 1)

#define W25Q128FV_SR2_BIT_QUAD_ENABLE           (1 << 1)

//#define W25Q128FV_INSTRUCTION_WRITE_DISABLE    0x04
//#define W25Q128FV_INSTRUCTION_PAGE_PROGRAM     0x02

// Values from W25Q128FV Datasheet Rev L.
#define W25Q128FV_TIMEOUT_PAGE_READ_MS          1           // No minimum specified in datasheet
#define W25Q128FV_TIMEOUT_RESET_MS              1           // tRST = 30us
#define W25Q128FV_TIMEOUT_BLOCK_ERASE_64KB_MS   2000        // tBE2max = 2000ms, tBE2typ = 150ms
#define W25Q128FV_TIMEOUT_CHIP_ERASE_MS        (200 * 1000) // tCEmax 200s, tCEtyp = 40s

#define W25Q128FV_TIMEOUT_PAGE_PROGRAM_MS       3           // tPPmax = 3ms, tPPtyp = 0.7ms
#define W25Q128FV_TIMEOUT_WRITE_ENABLE_MS       1

static bool w25q128fv_waitForReady(flashDevice_t *fdevice);
static void w25q128fv_waitForTimeout(flashDevice_t *fdevice);

MMFLASH_CODE static void w25q128fv_setTimeout(flashDevice_t *fdevice, timeMs_t timeoutMillis)
{
    timeMs_t nowMs = microsISR() / 1000;
    fdevice->timeoutAt = nowMs + timeoutMillis;
}

MMFLASH_CODE static void w25q128fv_performOneByteCommand(flashDeviceIO_t *io, uint8_t command)
{
#if defined(USE_QUADSPI)
    quadSpiTransmit1LINE(io->handle.dev, command, 0, NULL, 0);
#elif defined(USE_OCTOSPI)
    OCTOSPI_TypeDef *octoSpi = io->handle.octoSpi;
    octoSpiTransmit1LINE(octoSpi, command, 0, NULL, 0);
#endif

}

MMFLASH_CODE static void w25q128fv_performCommandWithAddress(flashDeviceIO_t *io, uint8_t command, uint32_t address)
{
#if defined(USE_QUADSPI)
    quadSpiInstructionWithAddress1LINE(io->handle.dev, command, 0, address & 0xffffff, W25Q128FV_ADDRESS_BITS);
#elif defined(USE_OCTOSPI)
    OCTOSPI_TypeDef *octoSpi = io->handle.octoSpi;

    octoSpiInstructionWithAddress1LINE(octoSpi, command, 0, address & 0xffffff, W25Q128FV_ADDRESS_BITS);
#endif
}

MMFLASH_CODE static void w25q128fv_writeEnable(flashDevice_t *fdevice)
{
    w25q128fv_performOneByteCommand(&fdevice->io, W25Q128FV_INSTRUCTION_WRITE_ENABLE);
}

MMFLASH_CODE static uint8_t w25q128fv_readRegister(flashDeviceIO_t *io, uint8_t command)
{
    uint8_t in[W25Q128FV_STATUS_REGISTER_BITS / 8] = { 0 };
#if defined(USE_QUADSPI)
    quadSpiReceive1LINE(io->handle.dev, command, 0, in, W25Q128FV_STATUS_REGISTER_BITS / 8);
#elif defined(USE_OCTOSPI)
    OCTOSPI_TypeDef *octoSpi = io->handle.octoSpi;

    octoSpiReceive1LINE(octoSpi, command, 0, in, W25Q128FV_STATUS_REGISTER_BITS / 8);
#endif

    return in[0];
}

static void w25q128fv_writeRegister(flashDeviceIO_t *io, uint8_t command, uint8_t data)
{
#if defined(USE_QUADSPI)
    quadSpiTransmit1LINE(io->handle.dev, command, 0, &data, W25Q128FV_STATUS_REGISTER_BITS / 8);
#elif defined(USE_OCTOSPI)
    OCTOSPI_TypeDef *octoSpi = io->handle.octoSpi;

    octoSpiTransmit1LINE(octoSpi, command, 0, &data, W25Q128FV_STATUS_REGISTER_BITS / 8);
#endif
}

static void w25q128fv_deviceReset(flashDevice_t *fdevice)
{
    flashDeviceIO_t *io = &fdevice->io;

    w25q128fv_waitForReady(fdevice);
    w25q128fv_performOneByteCommand(io, W25Q128FV_INSTRUCTION_ENABLE_RESET);
    w25q128fv_performOneByteCommand(io, W25Q128FV_INSTRUCTION_RESET_DEVICE);

    w25q128fv_setTimeout(fdevice, W25Q128FV_TIMEOUT_RESET_MS);
    w25q128fv_waitForTimeout(fdevice);

    w25q128fv_waitForReady(fdevice);

#ifdef DISABLE_NONVOLATILE_QE_MODE // Use this if you encounter a chip with it's QE bit enabled when it shouldn't be.
    w25q128fv_performOneByteCommand(io, W25Q128FV_INSTRUCTION_WRITE_ENABLE);
    w25q128fv_writeRegister(io, W25Q128FV_INSTRUCTION_WRITE_STATUS2_REG, 0x00);
#endif

#if defined(USE_FLASH_WRITES_USING_4LINES) || defined(USE_FLASH_READS_USING_4LINES)
    uint8_t registerValue = w25q128fv_readRegister(io, W25Q128FV_INSTRUCTION_READ_STATUS2_REG);

    //
    // WARNING: DO NOT ENABLE QE bit if IO2/IO3 are connected to GND or VCC.
    //
    // See datasheet https://www.winbond.com/resource-files/w25q128fv%20rev.m%2005132016%20kms.pdf
    // W25Q128FV - Revision M - 7.1.10 Quad Enable
    //
    // There is no such warning for the W25Q128JV in the same documentation section
    // See datasheet https://www.winbond.com/resource-files/w25q128jv%20revg%2004082019%20plus.pdf
    // W25Q128JV - Revision G - 7.1.4 Quad Enable
    //

    if ((registerValue & W25Q128FV_SR2_BIT_QUAD_ENABLE) == 0) {
        // Enable QUADSPI mode.
        registerValue = w25q128fv_readRegister(io, W25Q128FV_INSTRUCTION_READ_STATUS2_REG);

        uint8_t newValue = registerValue;
        newValue |= W25Q128FV_SR2_BIT_QUAD_ENABLE;

        //w25q128fv_performOneByteCommand(io, W25Q128FV_INSTRUCTION_WRITE_ENABLE);
        w25q128fv_performOneByteCommand(io, W25Q128FV_INSTRUCTION_VOLATILE_WRITE_ENABLE);
        w25q128fv_writeRegister(io, W25Q128FV_INSTRUCTION_WRITE_STATUS2_REG, newValue);
    }
#endif
}

static MMFLASH_CODE bool w25q128fv_isReady(flashDevice_t *fdevice)
{
    uint8_t status = w25q128fv_readRegister(&fdevice->io, W25Q128FV_INSTRUCTION_READ_STATUS1_REG);

    bool busy = (status & W25Q128FV_SR1_BIT_WRITE_IN_PROGRESS);

    return !busy;
}

#if !defined(USE_QUADSPI)
MMFLASH_CODE static bool w25q128fv_isWritable(flashDevice_t *fdevice)
{
    uint8_t status = w25q128fv_readRegister(&fdevice->io, W25Q128FV_INSTRUCTION_READ_STATUS1_REG);

    bool writable = (status & W25Q128FV_SR1_BIT_WRITE_ENABLED);

    return writable;
}
#endif

static MMFLASH_CODE bool w25q128fv_hasTimedOut(flashDevice_t *fdevice)
{
    uint32_t nowMs = microsISR() / 1000;
    if (cmp32(nowMs, fdevice->timeoutAt) >= 0) {
        return true;
    }
    return false;
}

MMFLASH_CODE void w25q128fv_waitForTimeout(flashDevice_t *fdevice)
{
   while (!w25q128fv_hasTimedOut(fdevice)) { }

   fdevice->timeoutAt = 0;
}

MMFLASH_CODE bool w25q128fv_waitForReady(flashDevice_t *fdevice)
{
    bool ready = true;
    while (!w25q128fv_isReady(fdevice)) {
        if (w25q128fv_hasTimedOut(fdevice)) {
            ready = false;
            break;
        }
    }
    fdevice->timeoutAt = 0;

    return ready;
}

const flashVTable_t w25q128fv_vTable;

static void w25q128fv_deviceInit(flashDevice_t *flashdev);

MMFLASH_CODE_NOINLINE bool w25q128fv_identify(flashDevice_t *fdevice, uint32_t jedecID)
{
    switch (jedecID) {
    case JEDEC_ID_WINBOND_W25Q128FV_SPI:
    case JEDEC_ID_WINBOND_W25Q128FV_QUADSPI:
    case JEDEC_ID_WINBOND_W25Q128JV_QUADSPI:
        fdevice->geometry.sectors           = 256;
        fdevice->geometry.pagesPerSector    = 256;
        fdevice->geometry.pageSize          = 256;
        // = 16777216 128MBit 16MB
        break;

    case JEDEC_ID_WINBOND_W25Q16JV_DTR_SPI:
    case JEDEC_ID_WINBOND_W25Q16JV_SPI:
        fdevice->geometry.sectors           = 32;
        fdevice->geometry.pagesPerSector    = 256;
        fdevice->geometry.pageSize          = 256;
        // = 2097152 16MBit 2MB
        break;

    default:
        // Unsupported chip
        fdevice->geometry.sectors = 0;
        fdevice->geometry.pagesPerSector = 0;
        fdevice->geometry.sectorSize = 0;
        fdevice->geometry.totalSize = 0;
        return false;
    }

    fdevice->geometry.flashType = FLASH_TYPE_NOR;
    fdevice->geometry.sectorSize = fdevice->geometry.pagesPerSector * fdevice->geometry.pageSize;
    fdevice->geometry.totalSize = fdevice->geometry.sectorSize * fdevice->geometry.sectors;

    fdevice->vTable = &w25q128fv_vTable;

    return true;
}

static void w25q128fv_configure(flashDevice_t *fdevice, uint32_t configurationFlags)
{
    if (configurationFlags & FLASH_CF_SYSTEM_IS_MEMORY_MAPPED) {
        return;
    }

    w25q128fv_deviceReset(fdevice);

    w25q128fv_deviceInit(fdevice);
}

MMFLASH_CODE static void w25q128fv_eraseSector(flashDevice_t *fdevice, uint32_t address)
{
    w25q128fv_waitForReady(fdevice);

    w25q128fv_writeEnable(fdevice);

    w25q128fv_performCommandWithAddress(&fdevice->io, W25Q128FV_INSTRUCTION_BLOCK_ERASE_64KB, address);

    w25q128fv_setTimeout(fdevice, W25Q128FV_TIMEOUT_BLOCK_ERASE_64KB_MS);
}

static void w25q128fv_eraseCompletely(flashDevice_t *fdevice)
{
    w25q128fv_waitForReady(fdevice);

    w25q128fv_writeEnable(fdevice);

    w25q128fv_performOneByteCommand(&fdevice->io, W25Q128FV_INSTRUCTION_CHIP_ERASE);

    w25q128fv_setTimeout(fdevice, W25Q128FV_TIMEOUT_CHIP_ERASE_MS);
}

#if defined(USE_OCTOSPI)
MMFLASH_CODE static void w25q128fv_loadProgramData(flashDevice_t *fdevice, const uint8_t *data, int length)
{
    w25q128fv_waitForReady(fdevice);

    OCTOSPI_TypeDef *octoSpi = fdevice->io.handle.octoSpi;
#ifdef USE_FLASH_WRITES_USING_4LINES
    octoSpiTransmitWithAddress4LINES(octoSpi, W25Q128FV_INSTRUCTION_QUAD_PAGE_PROGRAM, 0, fdevice->currentWriteAddress, W25Q128FV_ADDRESS_BITS, data, length);
#else
    octoSpiTransmitWithAddress1LINE(octoSpi, W25Q128FV_INSTRUCTION_PAGE_PROGRAM, 0, fdevice->currentWriteAddress, W25Q128FV_ADDRESS_BITS, data, length);
#endif

    w25q128fv_setTimeout(fdevice, W25Q128FV_TIMEOUT_PAGE_PROGRAM_MS);

    fdevice->currentWriteAddress += length;
}
#endif

MMFLASH_CODE static void w25q128fv_pageProgramBegin(flashDevice_t *fdevice, uint32_t address, void (*callback)(uintptr_t arg))
{
    fdevice->callback = callback;
    fdevice->currentWriteAddress = address;
}

#if defined(USE_QUADSPI)
// Called in ISR context
static busStatus_e w25q128fv_callbackReady(uintptr_t arg)
{
    flashDevice_t *fdevice = (flashDevice_t *)arg;
    extDevice_t *dev = fdevice->io.handle.dev;

    uint8_t readyPoll = dev->bus->curSegment->u.buffers.rxData[1];

    if (readyPoll & W25Q128FV_SR1_BIT_WRITE_IN_PROGRESS) {
        return BUS_BUSY;
    }

    // Bus is now known not to be busy
    fdevice->couldBeBusy = false;

    return BUS_READY;
}

// Called in ISR context
static busStatus_e w25q128fv_callbackWriteEnable(uintptr_t arg)
{
    flashDevice_t *fdevice = (flashDevice_t *)arg;
    fdevice->couldBeBusy = true;
    return BUS_READY;
}

// Called in ISR context when data segment completes
static busStatus_e w25q128fv_callbackWriteComplete(uintptr_t arg)
{
    flashDevice_t *fdevice = (flashDevice_t *)arg;
    fdevice->currentWriteAddress += fdevice->bytesWritten;
    if (fdevice->callback) {
        fdevice->callback(fdevice->bytesWritten);
    }
    return BUS_READY;
}
#endif

// SPI transaction segment indicies for w25q128fv_pageProgramContinue()
enum {READ_STATUS, WRITE_ENABLE, PAGE_PROGRAM, DATA1, DATA2};

MMFLASH_CODE static uint32_t w25q128fv_pageProgramContinue(flashDevice_t *fdevice, uint8_t const **buffers, const uint32_t *bufferSizes, uint32_t bufferCount)
{
#if defined(USE_QUADSPI)
    if (bufferCount == 0) {
        if (fdevice->callback) fdevice->callback(0);
        return 0;
    }

    // The segment list cannot be in automatic storage as this routine is non-blocking
    STATIC_DMA_DATA_AUTO uint8_t readStatus[2] = { W25Q128FV_INSTRUCTION_READ_STATUS1_REG};
    STATIC_DMA_DATA_AUTO uint8_t readyStatus[2];
    STATIC_DMA_DATA_AUTO uint8_t writeEnable[] = { W25Q128FV_INSTRUCTION_WRITE_ENABLE };
    STATIC_DMA_DATA_AUTO uint8_t pageProgram[1 + 3] = { W25Q128FV_INSTRUCTION_PAGE_PROGRAM };

    static busSegment_t segments[] = {
        {.u.buffers = {readStatus, readyStatus}, .len = sizeof(readStatus), .negateCS = true, w25q128fv_callbackReady},
        {.u.buffers = {writeEnable, NULL}, .len = sizeof(writeEnable), .negateCS = true, w25q128fv_callbackWriteEnable},
        {.u.buffers = {pageProgram, NULL}, .len = sizeof(pageProgram), .negateCS = false, NULL},
        {.u.link = {NULL, NULL}, .len = 0, .negateCS = true, NULL},
        {.u.link = {NULL, NULL}, .len = 0, .negateCS = true, NULL},
        {.u.link = {NULL, NULL}, .len = 0, .negateCS = true, NULL},
    };

    // Ensure any prior DMA has completed before continuing
    quadSpiWait(fdevice->io.handle.dev);

    // Patch command/address
    pageProgram[1] = (fdevice->currentWriteAddress >> 16) & 0xFF;
    pageProgram[2] = (fdevice->currentWriteAddress >> 8) & 0xFF;
    pageProgram[3] = (fdevice->currentWriteAddress >> 0) & 0xFF;

    // Patch the data segments
    segments[DATA1].u.buffers.txData = (uint8_t *)buffers[0];
    segments[DATA1].len = bufferSizes[0];
    fdevice->bytesWritten = bufferSizes[0];

    /* As the DATA2 segment may be used as the terminating segment, the rxData and txData may be overwritten
     * with a link to the following transaction (u.link.dev and u.link.segments respectively) so ensure that
     * rxData is reinitialised otherwise it will remain pointing at a chained u.link.segments structure which
     * would result in it being corrupted.
     */
    segments[DATA2].u.buffers.rxData = (uint8_t *)NULL;

    if (bufferCount == 1) {
        segments[DATA1].negateCS = true;
        segments[DATA1].callback = w25q128fv_callbackWriteComplete;
        // Mark segment following data as being of zero length
        segments[DATA2].u.buffers.txData = (uint8_t *)NULL;
        segments[DATA2].len = 0;
    } else if (bufferCount == 2) {
        segments[DATA1].negateCS = false;
        segments[DATA1].callback = NULL;
        segments[DATA2].u.buffers.txData = (uint8_t *)buffers[1];
        segments[DATA2].len = bufferSizes[1];
        fdevice->bytesWritten += bufferSizes[1];
        segments[DATA2].negateCS = true;
        segments[DATA2].callback = w25q128fv_callbackWriteComplete;
    } else {
        return 0;
    }

    quadSpiSequence(fdevice->io.handle.dev, &segments[0]);

    if (fdevice->callback == NULL) {
        quadSpiWait(fdevice->io.handle.dev);
    }
#else
    fdevice->bytesWritten = 0;
    for (uint32_t i = 0; i < bufferCount; i++) {
        w25q128fv_waitForReady(fdevice);
        w25q128fv_writeEnable(fdevice);
        w25q128fv_setTimeout(fdevice, W25Q128FV_TIMEOUT_WRITE_ENABLE_MS);
        bool writable = false;
        do {
            writable = w25q128fv_isWritable(fdevice);
        } while (!writable && w25q128fv_hasTimedOut(fdevice));
        if (!writable) {
            return 0;
        }
        w25q128fv_loadProgramData(fdevice, buffers[i], bufferSizes[i]);
        fdevice->bytesWritten += bufferSizes[i];
    }
    if (fdevice->callback) {
        fdevice->callback(fdevice->bytesWritten);
    }
#endif

    return fdevice->bytesWritten;
}

MMFLASH_CODE static void w25q128fv_pageProgramFinish(flashDevice_t *fdevice)
{
    UNUSED(fdevice);
}

MMFLASH_CODE static void w25q128fv_pageProgram(flashDevice_t *fdevice, uint32_t address, const uint8_t *data, uint32_t length, void (*callback)(uintptr_t arg))
{
    w25q128fv_pageProgramBegin(fdevice, address, callback);
    w25q128fv_pageProgramContinue(fdevice, &data, &length, 1);
    w25q128fv_pageProgramFinish(fdevice);
}

static MMFLASH_CODE void w25q128fv_flush(flashDevice_t *fdevice)
{
    UNUSED(fdevice);
}

MMFLASH_CODE static int w25q128fv_readBytes(flashDevice_t *fdevice, uint32_t address, uint8_t *buffer, uint32_t length)
{
    if (!w25q128fv_waitForReady(fdevice)) {
        return 0;
    }

#if defined(USE_QUADSPI)
    extDevice_t *dev = fdevice->io.handle.dev;
#ifdef USE_FLASH_READS_USING_4LINES
    bool status = quadSpiReceiveWithAddress4LINES(dev, W25Q128FV_INSTRUCTION_FAST_READ_QUAD_OUTPUT, 8, address, W25Q128FV_ADDRESS_BITS, buffer, length);
#else
    bool status = quadSpiReceiveWithAddress1LINE(dev, W25Q128FV_INSTRUCTION_READ_BYTES, 0, address, W25Q128FV_ADDRESS_BITS, buffer, length);
#endif
#elif defined(USE_OCTOSPI)
    OCTOSPI_TypeDef *octoSpi = fdevice->io.handle.octoSpi;
#ifdef USE_FLASH_READS_USING_4LINES
    bool status = octoSpiReceiveWithAddress4LINES(octoSpi, W25Q128FV_INSTRUCTION_FAST_READ_QUAD_OUTPUT, 8, address, W25Q128FV_ADDRESS_BITS, buffer, length);
#else
    bool status = octoSpiReceiveWithAddress1LINE(octoSpi, W25Q128FV_INSTRUCTION_FAST_READ, 8, address, W25Q128FV_ADDRESS_BITS, buffer, length);
#endif
#endif

    w25q128fv_setTimeout(fdevice, W25Q128FV_TIMEOUT_PAGE_READ_MS);

    if (!status) {
        return 0;
    }

    return length;
}

static const flashGeometry_t* w25q128fv_getGeometry(flashDevice_t *fdevice)
{
    return &fdevice->geometry;
}

MMFLASH_DATA const flashVTable_t w25q128fv_vTable = {
    .configure = w25q128fv_configure,
    .isReady = w25q128fv_isReady,
    .waitForReady = w25q128fv_waitForReady,
    .eraseSector = w25q128fv_eraseSector,
    .eraseCompletely = w25q128fv_eraseCompletely,
    .pageProgramBegin = w25q128fv_pageProgramBegin,
    .pageProgramContinue = w25q128fv_pageProgramContinue,
    .pageProgramFinish = w25q128fv_pageProgramFinish,
    .pageProgram = w25q128fv_pageProgram,
    .flush = w25q128fv_flush,
    .readBytes = w25q128fv_readBytes,
    .getGeometry = w25q128fv_getGeometry,
};

static void w25q128fv_deviceInit(flashDevice_t *flashdev)
{
    UNUSED(flashdev);
}

#endif
