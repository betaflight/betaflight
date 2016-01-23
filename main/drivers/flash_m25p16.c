/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_FLASH_M25P16

#include "drivers/flash_m25p16.h"
#include "drivers/bus_spi.h"
#include "drivers/system.h"

#define M25P16_INSTRUCTION_RDID             0x9F
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

// Format is manufacturer, memory type, then capacity
#define JEDEC_ID_MICRON_M25P16         0x202015
#define JEDEC_ID_MICRON_N25Q064        0x20BA17
#define JEDEC_ID_WINBOND_W25Q64        0xEF4017
#define JEDEC_ID_MICRON_N25Q128        0x20ba18
#define JEDEC_ID_WINBOND_W25Q128       0xEF4018

#define DISABLE_M25P16       GPIO_SetBits(M25P16_CS_GPIO,   M25P16_CS_PIN)
#define ENABLE_M25P16        GPIO_ResetBits(M25P16_CS_GPIO, M25P16_CS_PIN)

// The timeout we expect between being able to issue page program instructions
#define DEFAULT_TIMEOUT_MILLIS       6

// These take sooooo long:
#define SECTOR_ERASE_TIMEOUT_MILLIS  5000
#define BULK_ERASE_TIMEOUT_MILLIS    21000

static flashGeometry_t geometry = {.pageSize = M25P16_PAGESIZE};

/*
 * Whether we've performed an action that could have made the device busy for writes.
 *
 * This allows us to avoid polling for writable status when it is definitely ready already.
 */
static bool couldBeBusy = false;

/**
 * Send the given command byte to the device.
 */
static void m25p16_performOneByteCommand(uint8_t command)
{
    ENABLE_M25P16;

    spiTransferByte(M25P16_SPI_INSTANCE, command);

    DISABLE_M25P16;
}

/**
 * The flash requires this write enable command to be sent before commands that would cause
 * a write like program and erase.
 */
static void m25p16_writeEnable()
{
    m25p16_performOneByteCommand(M25P16_INSTRUCTION_WRITE_ENABLE);

    // Assume that we're about to do some writing, so the device is just about to become busy
    couldBeBusy = true;
}

static uint8_t m25p16_readStatus()
{
    uint8_t command[2] = {M25P16_INSTRUCTION_READ_STATUS_REG, 0};
    uint8_t in[2];

    ENABLE_M25P16;

    spiTransfer(M25P16_SPI_INSTANCE, in, command, sizeof(command));

    DISABLE_M25P16;

    return in[1];
}

bool m25p16_isReady()
{
    // If couldBeBusy is false, don't bother to poll the flash chip for its status
    couldBeBusy = couldBeBusy && ((m25p16_readStatus() & M25P16_STATUS_FLAG_WRITE_IN_PROGRESS) != 0);

    return !couldBeBusy;
}

bool m25p16_waitForReady(uint32_t timeoutMillis)
{
    uint32_t time = millis();
    while (!m25p16_isReady()) {
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
static bool m25p16_readIdentification()
{
    uint8_t out[] = { M25P16_INSTRUCTION_RDID, 0, 0, 0};
    uint8_t in[4];
    uint32_t chipID;

    delay(50); // short delay required after initialisation of SPI device instance.

    /* Just in case transfer fails and writes nothing, so we don't try to verify the ID against random garbage
     * from the stack:
     */
    in[1] = 0;

    ENABLE_M25P16;

    spiTransfer(M25P16_SPI_INSTANCE, in, out, sizeof(out));

    // Clearing the CS bit terminates the command early so we don't have to read the chip UID:
    DISABLE_M25P16;

    // Manufacturer, memory type, and capacity
    chipID = (in[1] << 16) | (in[2] << 8) | (in[3]);

    // All supported chips use the same pagesize of 256 bytes

    switch (chipID) {
        case JEDEC_ID_MICRON_M25P16:
            geometry.sectors = 32;
            geometry.pagesPerSector = 256;
        break;
        case JEDEC_ID_MICRON_N25Q064:
        case JEDEC_ID_WINBOND_W25Q64:
            geometry.sectors = 128;
            geometry.pagesPerSector = 256;
        break;
        case JEDEC_ID_MICRON_N25Q128:
        case JEDEC_ID_WINBOND_W25Q128:
            geometry.sectors = 256;
            geometry.pagesPerSector = 256;
        break;
        default:
            // Unsupported chip or not an SPI NOR flash
            geometry.sectors = 0;
            geometry.pagesPerSector = 0;

            geometry.sectorSize = 0;
            geometry.totalSize = 0;
            return false;
    }

    geometry.sectorSize = geometry.pagesPerSector * geometry.pageSize;
    geometry.totalSize = geometry.sectorSize * geometry.sectors;

    couldBeBusy = true; // Just for luck we'll assume the chip could be busy even though it isn't specced to be

    return true;
}

/**
 * Initialize the driver, must be called before any other routines.
 *
 * Attempts to detect a connected m25p16. If found, true is returned and device capacity can be fetched with
 * m25p16_getGeometry().
 */
bool m25p16_init()
{
    //Maximum speed for standard READ command is 20mHz, other commands tolerate 25mHz
    spiSetDivisor(M25P16_SPI_INSTANCE, SPI_18MHZ_CLOCK_DIVIDER);

    return m25p16_readIdentification();
}

/**
 * Erase a sector full of bytes to all 1's at the given byte offset in the flash chip.
 */
void m25p16_eraseSector(uint32_t address)
{
    uint8_t out[] = { M25P16_INSTRUCTION_SECTOR_ERASE, (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF};

    m25p16_waitForReady(SECTOR_ERASE_TIMEOUT_MILLIS);

    m25p16_writeEnable();

    ENABLE_M25P16;

    spiTransfer(M25P16_SPI_INSTANCE, NULL, out, sizeof(out));

    DISABLE_M25P16;
}

void m25p16_eraseCompletely()
{
    m25p16_waitForReady(BULK_ERASE_TIMEOUT_MILLIS);

    m25p16_writeEnable();

    m25p16_performOneByteCommand(M25P16_INSTRUCTION_BULK_ERASE);
}

void m25p16_pageProgramBegin(uint32_t address)
{
    uint8_t command[] = { M25P16_INSTRUCTION_PAGE_PROGRAM, (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF};

    m25p16_waitForReady(DEFAULT_TIMEOUT_MILLIS);

    m25p16_writeEnable();

    ENABLE_M25P16;

    spiTransfer(M25P16_SPI_INSTANCE, NULL, command, sizeof(command));
}

void m25p16_pageProgramContinue(const uint8_t *data, int length)
{
    spiTransfer(M25P16_SPI_INSTANCE, NULL, data, length);
}

void m25p16_pageProgramFinish()
{
    DISABLE_M25P16;
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
void m25p16_pageProgram(uint32_t address, const uint8_t *data, int length)
{
    m25p16_pageProgramBegin(address);

    m25p16_pageProgramContinue(data, length);

    m25p16_pageProgramFinish();
}

/**
 * Read `length` bytes into the provided `buffer` from the flash starting from the given `address` (which need not lie
 * on a page boundary).
 *
 * Waits up to DEFAULT_TIMEOUT_MILLIS milliseconds for the flash to become ready before reading.
 *
 * The number of bytes actually read is returned, which can be zero if an error or timeout occurred.
 */
int m25p16_readBytes(uint32_t address, uint8_t *buffer, int length)
{
    uint8_t command[] = { M25P16_INSTRUCTION_READ_BYTES, (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF};

    if (!m25p16_waitForReady(DEFAULT_TIMEOUT_MILLIS)) {
        return 0;
    }

    ENABLE_M25P16;

    spiTransfer(M25P16_SPI_INSTANCE, NULL, command, sizeof(command));
    spiTransfer(M25P16_SPI_INSTANCE, buffer, NULL, length);

    DISABLE_M25P16;

    return length;
}

/**
 * Fetch information about the detected flash chip layout.
 *
 * Can be called before calling m25p16_init() (the result would have totalSize = 0).
 */
const flashGeometry_t* m25p16_getGeometry()
{
    return &geometry;
}

#endif
