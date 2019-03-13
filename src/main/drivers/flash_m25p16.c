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

#ifdef USE_FLASH_M25P16

#include "drivers/bus_spi.h"
#include "drivers/flash.h"
#include "drivers/flash_impl.h"
#include "drivers/io.h"
#include "drivers/time.h"

#include "pg/flash.h"

#include "flash_m25p16.h"

#define M25P16_INSTRUCTION_RDID             SPIFLASH_INSTRUCTION_RDID
#define M25P16_INSTRUCTION_READ_BYTES       0x03
#define M25P16_INSTRUCTION_READ_STATUS_REG  0x05
#define M25P16_INSTRUCTION_WRITE_STATUS_REG 0x01
#define M25P16_INSTRUCTION_WRITE_ENABLE     0x06
#define M25P16_INSTRUCTION_WRITE_DISABLE    0x04
#define M25P16_INSTRUCTION_PAGE_PROGRAM     0x02
#define M25P16_INSTRUCTION_SECTOR_ERASE     0xD8
#define M25P16_INSTRUCTION_BULK_ERASE       0xC7

#define M25P16_STATUS_FLAG_WRITE_IN_PROGRESS 0x01
#define M25P16_STATUS_FLAG_WRITE_ENABLED     0x02

#define W25Q256_INSTRUCTION_ENTER_4BYTE_ADDRESS_MODE 0xB7

// Format is manufacturer, memory type, then capacity
// See also flash_m25p16.h for additional JEDEC IDs.
#define JEDEC_ID_MACRONIX_MX25L3206E   0xC22016
#define JEDEC_ID_MACRONIX_MX25L6406E   0xC22017
#define JEDEC_ID_MACRONIX_MX25L25635E  0xC22019
#define JEDEC_ID_MICRON_M25P16         0x202015
#define JEDEC_ID_MICRON_N25Q064        0x20BA17
#define JEDEC_ID_MICRON_N25Q128        0x20ba18
#define JEDEC_ID_WINBOND_W25Q16        0xEF4015
#define JEDEC_ID_WINBOND_W25Q32        0xEF4016
#define JEDEC_ID_WINBOND_W25Q64        0xEF4017
#define JEDEC_ID_WINBOND_W25Q128       0xEF4018
#define JEDEC_ID_WINBOND_W25Q128_DTR   0xEF7018
#define JEDEC_ID_CYPRESS_S25FL128L     0x016018
#define JEDEC_ID_BERGMICRO_W25Q32      0xE04016

// The timeout we expect between being able to issue page program instructions
#define DEFAULT_TIMEOUT_MILLIS       6

// These take sooooo long:
#define SECTOR_ERASE_TIMEOUT_MILLIS  5000
#define BULK_ERASE_TIMEOUT_MILLIS    21000

#define M25P16_PAGESIZE 256

STATIC_ASSERT(M25P16_PAGESIZE < FLASH_MAX_PAGE_SIZE, M25P16_PAGESIZE_too_small);

const flashVTable_t m25p16_vTable;

#ifndef USE_SPI_TRANSACTION
static void m25p16_disable(busDevice_t *bus)
{
    IOHi(bus->busdev_u.spi.csnPin);
    __NOP();
}

static void m25p16_enable(busDevice_t *bus)
{
    __NOP();
    IOLo(bus->busdev_u.spi.csnPin);
}
#endif

static void m25p16_transfer(busDevice_t *bus, const uint8_t *txData, uint8_t *rxData, int len)
{
#ifdef USE_SPI_TRANSACTION
    spiBusTransactionTransfer(bus, txData, rxData, len);
#else
    m25p16_enable(bus);
    spiTransfer(bus->busdev_u.spi.instance, txData, rxData, len);
    m25p16_disable(bus);
#endif
}

/**
 * Send the given command byte to the device.
 */
static void m25p16_performOneByteCommand(busDevice_t *bus, uint8_t command)
{
#ifdef USE_SPI_TRANSACTION
    m25p16_transfer(bus, &command, NULL, 1);
#else
    m25p16_enable(bus);
    spiTransferByte(bus->busdev_u.spi.instance, command);
    m25p16_disable(bus);
#endif
}

/**
 * The flash requires this write enable command to be sent before commands that would cause
 * a write like program and erase.
 */
static void m25p16_writeEnable(flashDevice_t *fdevice)
{
    m25p16_performOneByteCommand(fdevice->busdev, M25P16_INSTRUCTION_WRITE_ENABLE);

    // Assume that we're about to do some writing, so the device is just about to become busy
    fdevice->couldBeBusy = true;
}

static uint8_t m25p16_readStatus(busDevice_t *bus)
{
    const uint8_t command[2] = { M25P16_INSTRUCTION_READ_STATUS_REG, 0 };
    uint8_t in[2];

    m25p16_transfer(bus, command, in, sizeof(command));

    return in[1];
}

static bool m25p16_isReady(flashDevice_t *fdevice)
{
    // If couldBeBusy is false, don't bother to poll the flash chip for its status
    fdevice->couldBeBusy = fdevice->couldBeBusy && ((m25p16_readStatus(fdevice->busdev) & M25P16_STATUS_FLAG_WRITE_IN_PROGRESS) != 0);

    return !fdevice->couldBeBusy;
}

static bool m25p16_waitForReady(flashDevice_t *fdevice, uint32_t timeoutMillis)
{
    uint32_t time = millis();
    while (!m25p16_isReady(fdevice)) {
        if (millis() - time > timeoutMillis) {
            return false;
        }
    }

    return true;
}

/**
 * Read chip identification and geometry information (into global `geometry`).
 *
 * Returns true if we get valid ident, false if something bad happened like there is no M25P16.
 */

bool m25p16_detect(flashDevice_t *fdevice, uint32_t chipID)
{
    switch (chipID) {
    case JEDEC_ID_WINBOND_W25Q16:
    case JEDEC_ID_MICRON_M25P16:
        fdevice->geometry.sectors = 32;
        fdevice->geometry.pagesPerSector = 256;
        break;
    case JEDEC_ID_BERGMICRO_W25Q32:
        fdevice->geometry.sectors = 1024;
        fdevice->geometry.pagesPerSector = 16;
        break;
    case JEDEC_ID_WINBOND_W25Q32:
    case JEDEC_ID_MACRONIX_MX25L3206E:
        fdevice->geometry.sectors = 64;
        fdevice->geometry.pagesPerSector = 256;
        break;
    case JEDEC_ID_MICRON_N25Q064:
    case JEDEC_ID_WINBOND_W25Q64:
    case JEDEC_ID_MACRONIX_MX25L6406E:
        fdevice->geometry.sectors = 128;
        fdevice->geometry.pagesPerSector = 256;
        break;
    case JEDEC_ID_MICRON_N25Q128:
    case JEDEC_ID_WINBOND_W25Q128:
    case JEDEC_ID_WINBOND_W25Q128_DTR:
    case JEDEC_ID_CYPRESS_S25FL128L:
        fdevice->geometry.sectors = 256;
        fdevice->geometry.pagesPerSector = 256;
        break;
    case JEDEC_ID_WINBOND_W25Q256:
    case JEDEC_ID_MACRONIX_MX25L25635E:
        fdevice->geometry.sectors = 512;
        fdevice->geometry.pagesPerSector = 256;
        break;
    default:
        // Unsupported chip or not an SPI NOR flash
        fdevice->geometry.sectors = 0;
        fdevice->geometry.pagesPerSector = 0;
        fdevice->geometry.sectorSize = 0;
        fdevice->geometry.totalSize = 0;
        return false;
    }

    fdevice->geometry.flashType = FLASH_TYPE_NOR;
    fdevice->geometry.pageSize = M25P16_PAGESIZE;
    fdevice->geometry.sectorSize = fdevice->geometry.pagesPerSector * fdevice->geometry.pageSize;
    fdevice->geometry.totalSize = fdevice->geometry.sectorSize * fdevice->geometry.sectors;

    if (fdevice->geometry.totalSize > 16 * 1024 * 1024) {
        fdevice->isLargeFlash = true;
        m25p16_performOneByteCommand(fdevice->busdev, W25Q256_INSTRUCTION_ENTER_4BYTE_ADDRESS_MODE);
    }

    fdevice->couldBeBusy = true; // Just for luck we'll assume the chip could be busy even though it isn't specced to be
    fdevice->vTable = &m25p16_vTable;
    return true;
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

/**
 * Erase a sector full of bytes to all 1's at the given byte offset in the flash chip.
 */
static void m25p16_eraseSector(flashDevice_t *fdevice, uint32_t address)
{
    uint8_t out[5] = { M25P16_INSTRUCTION_SECTOR_ERASE };

    m25p16_setCommandAddress(&out[1], address, fdevice->isLargeFlash);

    m25p16_waitForReady(fdevice, SECTOR_ERASE_TIMEOUT_MILLIS);

    m25p16_writeEnable(fdevice);

    m25p16_transfer(fdevice->busdev, out, NULL, sizeof(out));
}

static void m25p16_eraseCompletely(flashDevice_t *fdevice)
{
    m25p16_waitForReady(fdevice, BULK_ERASE_TIMEOUT_MILLIS);

    m25p16_writeEnable(fdevice);

    m25p16_performOneByteCommand(fdevice->busdev, M25P16_INSTRUCTION_BULK_ERASE);
}

static void m25p16_pageProgramBegin(flashDevice_t *fdevice, uint32_t address)
{
    UNUSED(fdevice);

    fdevice->currentWriteAddress = address;
}

static void m25p16_pageProgramContinue(flashDevice_t *fdevice, const uint8_t *data, int length)
{
    uint8_t command[5] = { M25P16_INSTRUCTION_PAGE_PROGRAM };

    m25p16_setCommandAddress(&command[1], fdevice->currentWriteAddress, fdevice->isLargeFlash);

    m25p16_waitForReady(fdevice, DEFAULT_TIMEOUT_MILLIS);

    m25p16_writeEnable(fdevice);

#ifdef USE_SPI_TRANSACTION
    spiBusTransactionBegin(fdevice->busdev);
#else
    m25p16_enable(fdevice->busdev);
#endif

    spiTransfer(fdevice->busdev->busdev_u.spi.instance, command, NULL, fdevice->isLargeFlash ? 5 : 4);
    spiTransfer(fdevice->busdev->busdev_u.spi.instance, data, NULL, length);

#ifdef USE_SPI_TRANSACTION
    spiBusTransactionEnd(fdevice->busdev);
#else
    m25p16_disable(fdevice->busdev);
#endif

    fdevice->currentWriteAddress += length;
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
static void m25p16_pageProgram(flashDevice_t *fdevice, uint32_t address, const uint8_t *data, int length)
{
    m25p16_pageProgramBegin(fdevice, address);

    m25p16_pageProgramContinue(fdevice, data, length);

    m25p16_pageProgramFinish(fdevice);
}

/**
 * Read `length` bytes into the provided `buffer` from the flash starting from the given `address` (which need not lie
 * on a page boundary).
 *
 * Waits up to DEFAULT_TIMEOUT_MILLIS milliseconds for the flash to become ready before reading.
 *
 * The number of bytes actually read is returned, which can be zero if an error or timeout occurred.
 */
static int m25p16_readBytes(flashDevice_t *fdevice, uint32_t address, uint8_t *buffer, int length)
{
    uint8_t command[5] = { M25P16_INSTRUCTION_READ_BYTES };

    m25p16_setCommandAddress(&command[1], address, fdevice->isLargeFlash);

    if (!m25p16_waitForReady(fdevice, DEFAULT_TIMEOUT_MILLIS)) {
        return 0;
    }

#ifdef USE_SPI_TRANSACTION
    spiBusTransactionBegin(fdevice->busdev);
#else
    m25p16_enable(fdevice->busdev);
#endif

    spiTransfer(fdevice->busdev->busdev_u.spi.instance, command, NULL, fdevice->isLargeFlash ? 5 : 4);
    spiTransfer(fdevice->busdev->busdev_u.spi.instance, NULL, buffer, length);

#ifdef USE_SPI_TRANSACTION
    spiBusTransactionEnd(fdevice->busdev);
#else
    m25p16_disable(fdevice->busdev);
#endif

    return length;
}

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
#endif
