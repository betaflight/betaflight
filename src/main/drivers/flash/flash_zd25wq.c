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


#include "drivers/flash/flash_impl.h"
#include "platform.h"
#include "build/debug.h"

#ifdef USE_FLASH_ZD25WQ
#ifndef USE_FLASH_QUADSPI
#error "The ZD25WQ driver only supports QSPI mode."
#endif
#ifdef USE_FLASH_MEMORY_MAPPED
#error "USE_FLASH_MEMORY_MAPPED is unsupported with QSPI mode."
#endif

#include <stdint.h>
#include <string.h>
#include "drivers/flash/flash.h"
#include "drivers/bus_quadspi.h"
#include "drivers/time.h"
#include "flash_zd25wq.h"

#define JEDEC_ID 0xBA6015
#define ZD25WQ_PAGESIZE 256

#define INSTRUCTION_1LINE_WRITE_ENABLE                           0x06
#define INSTRUCTION_1LINE_READ_STATUS_REGISTER1                  0x05
#define INSTRUCTION_1LINE_READ_STATUS_REGISTER2                  0x35
#define INSTRUCTION_1LINE_WRITE_STATUS_REGISTER1                 0x01
#define INSTRUCTION_1LINE_WRITE_STATUS_REGISTER2                 0x31
#define INSTRUCTION_1LINE_WRITE_STATUS_REGISTERS                 0x01 // same as INSTRUCTION_QPI_WRITE_STATUS_REGISTER1 but with two extra clock cycles
#define INSTRUCTION_1LINE_STATUS_REGISTERS_VOLATILE_WRITE_ENABLE 0x50
#define INSTRUCTION_1LINE_ENABLE_QPI                             0x38
#define INSTRUCTION_1LINE_RESET_ENABLE                           0x66
#define INSTRUCTION_1LINE_RESET                                  0x99

#define INSTRUCTION_QPI_WRITE_ENABLE                0x06
#define INSTRUCTION_QPI_WRITE_DISABLE               0x04
#define INSTRUCTION_QPI_READ_CONFIGURATION_REGISTER 0x45
#define INSTRUCTION_QPI_READ_STATUS_REGISTER1 0x05
#define INSTRUCTION_QPI_READ_STATUS_REGISTER2 0x35
#define INSTRUCTION_QPI_WRITE_STATUS_REGISTER1 0x01
#define INSTRUCTION_QPI_WRITE_STATUS_REGISTER2 0x31
#define INSTRUCTION_QPI_WRITE_STATUS_REGISTERS 0x01 // same as INSTRUCTION_QPI_WRITE_STATUS_REGISTER1 but with two extra clock cycles
#define INSTRUCTION_QPI_WRITE_CONFIGURATION_REGISTER 0x11
#define INSTRUCTION_QPI_STATUS_REGISTERS_VOLATILE_WRITE_ENABLE 0x50
#define INSTRUCTION_QPI_FAST_READ 0x0B
#define INSTRUCTION_QPI_FAST_READ_WITH_COMMAND_CODE 0xEB
#define INSTRUCTION_QPI_PAGE_PROGRAM 0x02
#define INSTRUCTION_QPI_PAGE_WRITE 0xA5
#define INSTRUCTION_QPI_PAGE_ERASE 0x81
#define INSTRUCTION_QPI_SECTOR_ERASE 0x20
#define INSTRUCTION_QPI_BLOCK_ERASE_32K 0x52
#define INSTRUCTION_QPI_BLOCK_ERASE_64K 0xD8
#define INSTRUCTION_QPI_CHIP_ERASE 0x60
#define INSTRUCTION_QPI_SET_READ_PARAMETERS 0xC0
#define INSTRUCTION_QPI_READ_MANUFACTURER_ID 0x90
#define INSTRUCTION_QPI_READ_ID 0x9F
#define INSTRUCTION_QPI_ERASE_SECURITY_REGISTERS 0x44
#define INSTRUCTION_QPI_PROGRAM_SECURITY_REGISTERS 0x42
#define INSTRUCTION_QPI_READ_SECURITY_REGISTERS 0x48
#define INSTRUCTION_QPI_RESET_ENABLE 0x66
#define INSTRUCTION_QPI_RESET 0x99
#define INSTRUCTION_QPI_WRAPPING_BURST_READ 0x0C
#define INSTRUCTION_QPI_WRITE_SUSPEND 0x75
#define INSTRUCTION_QPI_WRITE_RESUME 0x7A
#define INSTRUCTION_QPI_DEEP_SLEEP 0xB9
#define INSTRUCTION_QPI_DEEP_WAKEUP 0xAB
#define INSTRUCTION_QPI_POWER_DOWN_AND_READ_DEVICE_ID 0xAB // share id with INSTRUCTION_QPI_DEEP_WAKEUP but include 6 dummy clock cycles before data
#define INSTRUCTION_QPI_DISABLE_QPI 0xFF
#define INSTRUCTION_QPI_DISCOVER_FLASH_PARAMETERS 0x5A

#define STATUS_REGISTER1_FLAG_WRITE_IN_PROGRESS        0b00000001
#define STATUS_REGISTER1_FLAG_WRITE_ENABLE_LATCH       0b00000010
#define STATUS_REGISTER1_FLAG_BLOCK_PROTECT            0b01111100
#define STATUS_REGISTER1_FLAG_STATUS_REGISTER_PROTECT1 0b1000000

#define STATUS_REGISTER1_UNPACK_BLOCK_PROTECT(x) (((x) & STATUS_REGISTER1_FLAG_BLOCK_PROTECT) >> 2)
#define STATUS_REGISTER1_PACK_BLOCK_PROTECT(x)   (((x) << 2) & STATUS_REGISTER1_FLAG_BLOCK_PROTECT)

#define STATUS_REGISTER2_FLAG_STATUS_REGISTER_PROTECT2        0b00000001
#define STATUS_REGISTER2_FLAG_QUAD_ENABLE                     0b00000010
#define STATUS_REGISTER2_FLAG_WRITE_FAIL                      0b00000100
#define STATUS_REGISTER2_FLAG_SECURITY_REGISTER_WRITE_PROTECT 0b00111000
#define STATUS_REGISTER2_FLAG_CMP                             0b01000000 // I have no idea what CMP stands for, if set it mixes the memory pattern of the block protect fields.
#define STATUS_REGISTER2_FLAG_WRITE_SUSPENDED                 0b01000000

#define STATUS_REGISTER2_UNPACK_SECURITY_REGISTER_WRITE_PROTECT(x) (((x) & STATUS_REGISTER2_FLAG_SECURITY_REGISTER_WRITE_PROTECT) >> 3)
#define STATUS_REGISTER2_PACK_SECURITY_REGISTER_WRITE_PROTECT(x)   (((x) << 3) & STATUS_REGISTER2_FLAG_SECURITY_REGISTER_WRITE_PROTECT)

#define READ_PARAMETERS_FLAG_WRAP_LENGTH_MASK 0b00000011
enum {
    READ_PARAMETERS_FLAG_WRAP_LENGTH_8B  = 0,
    READ_PARAMETERS_FLAG_WRAP_LENGTH_16B = 1,
    READ_PARAMETERS_FLAG_WRAP_LENGTH_32B = 2,
    READ_PARAMETERS_FLAG_WRAP_LENGTH_64B = 3,
};
#define READ_PARAMETERS_FLAG_READ_DUMMY_CYCLES_MASK 0b00110000
enum {
    READ_PARAMETERS_FLAG_READ_DUMMY_CYCLES_10_86Mhz = 0b00,
    READ_PARAMETERS_FLAG_READ_DUMMY_CYCLES_4_68Mhz  = 0b01,
    READ_PARAMETERS_FLAG_READ_DUMMY_CYCLES_6_68Mhz  = 0b10,
    READ_PARAMETERS_FLAG_READ_DUMMY_CYCLES_8_86Mhz  = 0b11,
};

#define READ_PARAMETERS_UNPACK_WRAP_LENGTH_MASK(x)  ((x) & READ_PARAMETERS_FLAG_WRAP_LENGTH_MASK)
#define READ_PARAMETERS_PACK_WRAP_LENGTH_MASK(x)    ((x) & READ_PARAMETERS_FLAG_WRAP_LENGTH_MASK)
#define READ_PARAMETERS_UNPACK_READ_DUMMY_CYCLES(x) (((x) & READ_PARAMETERS_FLAG_READ_DUMMY_CYCLES_MASK) >> 4)
#define READ_PARAMETERS_PACK_READ_DUMMY_CYCLES(x)   (((x) << 4) & READ_PARAMETERS_FLAG_READ_DUMMY_CYCLES_MASK)

static uint8_t zd25wq_readStatus1Spi(flashDevice_t *fdevice)
{
    uint8_t status1;
    quadSpiReceive111(fdevice->io.handle.quadSpi, INSTRUCTION_QPI_READ_STATUS_REGISTER1, 0, 0, 0, &status1, sizeof(status1));
    return status1;
}

static uint8_t zd25wq_readStatus1(flashDevice_t *fdevice)
{
    uint8_t status1;
    quadSpiReceive444(fdevice->io.handle.quadSpi, INSTRUCTION_QPI_READ_STATUS_REGISTER1, 0, 0, 0, &status1, sizeof(status1));
    return status1;
}

static void zd25wq_configure(flashDevice_t *fdevice, uint32_t configurationFlags)
{
    UNUSED(configurationFlags);
    quadSpiTransmit111(fdevice->io.handle.quadSpi, INSTRUCTION_1LINE_RESET_ENABLE, 0, 0, 0, NULL, 0);
    quadSpiTransmit111(fdevice->io.handle.quadSpi, INSTRUCTION_1LINE_RESET, 0, 0, 0, NULL, 0);

    delay(15);

    uint8_t wantStatus2 = STATUS_REGISTER2_FLAG_QUAD_ENABLE;
    quadSpiTransmit111(fdevice->io.handle.quadSpi, INSTRUCTION_1LINE_STATUS_REGISTERS_VOLATILE_WRITE_ENABLE, 0, 0, 0, NULL, 0);
    quadSpiTransmit111(fdevice->io.handle.quadSpi, INSTRUCTION_1LINE_WRITE_STATUS_REGISTER2, 0, 0, 0, &wantStatus2, sizeof(wantStatus2));

    while (zd25wq_readStatus1Spi(fdevice) & STATUS_REGISTER1_FLAG_WRITE_IN_PROGRESS);

    quadSpiTransmit111(fdevice->io.handle.quadSpi, INSTRUCTION_1LINE_ENABLE_QPI, 0, 0, 0, NULL, 0);

    // FIXME: is this flag persisted? The data sheet is unclear.
    uint8_t readParameters = READ_PARAMETERS_PACK_READ_DUMMY_CYCLES(READ_PARAMETERS_FLAG_READ_DUMMY_CYCLES_4_68Mhz);

#define READ_DUMMY_CYCLES 4
    quadSpiTransmit444(fdevice->io.handle.quadSpi, INSTRUCTION_QPI_SET_READ_PARAMETERS, 0, 0, 0, &readParameters, sizeof(readParameters));
}

static bool zd25wq_isReady(flashDevice_t *fdevice)
{
    if (!fdevice->couldBeBusy) {
        return true;
    }

    fdevice->couldBeBusy = ((zd25wq_readStatus1(fdevice) & STATUS_REGISTER1_FLAG_WRITE_IN_PROGRESS) != 0);

    return !fdevice->couldBeBusy;
}

static bool zd25wq_waitForReady(flashDevice_t *fdevice)
{
    while (!zd25wq_isReady(fdevice));
    return true;
}

static void zd25wq_waitAndMakeReadyForWrite(flashDevice_t *fdevice)
{
    zd25wq_waitForReady(fdevice);

    quadSpiTransmit444(fdevice->io.handle.quadSpi, INSTRUCTION_QPI_WRITE_ENABLE, 0, 0, 0, NULL, 0);
}

static void zd25wq_eraseSector(flashDevice_t *fdevice, uint32_t address)
{
    zd25wq_waitAndMakeReadyForWrite(fdevice);
    quadSpiTransmit444(fdevice->io.handle.quadSpi, INSTRUCTION_QPI_SECTOR_ERASE, 0, address, 24, NULL, 0);
    fdevice->couldBeBusy = true;
}

static void zd25wq_eraseCompletely(flashDevice_t *fdevice)
{
    zd25wq_waitAndMakeReadyForWrite(fdevice);
    quadSpiTransmit444(fdevice->io.handle.quadSpi, INSTRUCTION_QPI_CHIP_ERASE, 0, 0, 0, NULL, 0);
    fdevice->couldBeBusy = true;
}

static void zd25wq_pageProgramBegin(flashDevice_t *fdevice, uint32_t address, void (*callback)(uintptr_t arg))
{
    fdevice->callback = callback;
    fdevice->currentWriteAddress = address;
}

static uint32_t zd25wq_pageProgramContinue(flashDevice_t *fdevice, uint8_t const **buffers, const uint32_t *bufferSizes, uint32_t bufferCount)
{
    uint8_t pageReassemblyBuffer[ZD25WQ_PAGESIZE];
    uint32_t remainingUntilEndOfPage = ZD25WQ_PAGESIZE - (fdevice->currentWriteAddress % ZD25WQ_PAGESIZE);
    const uint8_t *pageBuffer;
    uint32_t written = 0;
    if (bufferCount == 1) {
        pageBuffer = buffers[0];
        if (remainingUntilEndOfPage < bufferSizes[0]) {
            return 0;
        }
        written = bufferSizes[0];
    } else {
        pageBuffer = pageReassemblyBuffer;
        for (uint32_t i = 0; i < bufferCount; i++) {
            if (remainingUntilEndOfPage < bufferSizes[i]) {
                return 0;
            }
            memcpy(&pageReassemblyBuffer[written], buffers[i], bufferSizes[i]);
            written += bufferSizes[i];
            remainingUntilEndOfPage -= bufferSizes[i];
        }
    }

    zd25wq_waitAndMakeReadyForWrite(fdevice);
    quadSpiTransmit444(fdevice->io.handle.quadSpi, INSTRUCTION_QPI_PAGE_PROGRAM, 0, fdevice->currentWriteAddress, 24, pageBuffer, written);
    fdevice->currentWriteAddress += written;
    fdevice->couldBeBusy = true;

    if (fdevice->callback) {
        fdevice->callback(written);
    }

    return written;
}

static void zd25wq_pageProgramFinish(flashDevice_t *fdevice)
{
    fdevice->callback = NULL;
    fdevice->currentWriteAddress = 0;
}

static void zd25wq_pageProgram(flashDevice_t *fdevice, uint32_t address, const uint8_t *data, uint32_t length, void (*callback)(uintptr_t arg))
{
    zd25wq_pageProgramBegin(fdevice, address, callback);
    zd25wq_pageProgramContinue(fdevice, &data, &length, 1);
    zd25wq_pageProgramFinish(fdevice);
}

static int zd25wq_readBytes(flashDevice_t *fdevice, uint32_t address, uint8_t *buffer, uint32_t length)
{
    zd25wq_waitForReady(fdevice);
    quadSpiSetDivisor(fdevice->io.handle.quadSpi, 2); // 66.6Mhz
    quadSpiReceive444(fdevice->io.handle.quadSpi, INSTRUCTION_QPI_FAST_READ, READ_DUMMY_CYCLES, address, 24, buffer, length);
    quadSpiSetDivisor(fdevice->io.handle.quadSpi, QUADSPI_CLOCK_ULTRAFAST);
    return length;
}

static const flashGeometry_t* zd25wq_getGeometry(flashDevice_t *fdevice)
{
    return &fdevice->geometry;
}

const flashVTable_t zd25wq_vTable = {
    .configure = zd25wq_configure,
    .isReady = zd25wq_isReady,
    .waitForReady = zd25wq_waitForReady,
    .eraseSector = zd25wq_eraseSector,
    .eraseCompletely = zd25wq_eraseCompletely,
    .pageProgramBegin = zd25wq_pageProgramBegin,
    .pageProgramContinue = zd25wq_pageProgramContinue,
    .pageProgramFinish = zd25wq_pageProgramFinish,
    .pageProgram = zd25wq_pageProgram,
    .readBytes = zd25wq_readBytes,
    .getGeometry = zd25wq_getGeometry,
};

bool zd25wq_identify(flashDevice_t *fdevice, uint32_t jedecID, bool isQPI)
{
    if (jedecID != JEDEC_ID) {
        return false;
    }
    if (fdevice->io.mode != FLASHIO_QUADSPI) {
        return false;
    }
    if (isQPI) {
        // There exists a peculiar bug:
        // The chip has a QE config bit that enables IO2 & IO3 pins (normally HOLD# and WP#) for QSPI.
        // This QE bit is needed to enable QPI mode since QPI mode uses QSPI L1.
        // 
        // Following the datasheet's procedure to make it persist in NVM does not work (WRITE_ENABLE then write to status 2 reg).
        // However, QPI mode does persist in NVM (and it doesn't work to make it volatile but the datasheet doesn't mention this).
        // 
        // Problem: After a power cycle or BF reset, zd25wq_configure will reset the flash, leaving it corrupted where
        // QPI is enabled but the QE bit is disabled.
        // 
        // So we disable QPI to ensure the chip is always in a consistent state.
        quadSpiTransmit444(fdevice->io.handle.quadSpi, INSTRUCTION_QPI_DISABLE_QPI, 0, 0, 0, NULL, 0);
    }
    fdevice->vTable = &zd25wq_vTable;

    flashGeometry_t *geometry = &fdevice->geometry;

    geometry->jedecId = jedecID;
    geometry->flashType = FLASH_TYPE_NOR;
    geometry->pagesPerSector = 16;
    geometry->pageSize = ZD25WQ_PAGESIZE;
    geometry->sectors = 512;
    geometry->sectorSize = geometry->pagesPerSector * geometry->pageSize;
    geometry->totalSize = geometry->sectorSize * geometry->sectors;

    // Assume the chip could be busy even though it isn't supposed to be.
    fdevice->couldBeBusy = true; 

    return true;
}

#endif
