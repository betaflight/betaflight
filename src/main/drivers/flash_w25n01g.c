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
 *
 * Author: jflyper
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "build/debug.h"

#ifdef USE_FLASH_W25N01G

#include "flash.h"
#include "flash_impl.h"
#include "flash_w25n01g.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/time.h"

//#define FLASH_W25N01G_DPRINTF

#ifdef FLASH_W25N01G_DPRINTF
#include "common/printf.h"
#include "common/utils.h"
#include "io/serial.h"
serialPort_t *debugSerialPort = NULL;
#define DPRINTF_SERIAL_PORT SERIAL_PORT_USART3
#define DPRINTF(x) tfp_printf x
#else
#define DPRINTF(x)
#endif

// JEDEC ID
#define JEDEC_ID_WINBOND_W25N01GV    0xEFAA21
#define JEDEC_ID_WINBOND_W25M02GV    0xEFAB21

// Device size parameters
#define W25N01G_PAGE_SIZE         2048
#define W25N01G_PAGES_PER_BLOCK   64
#define W25N01G_BLOCKS_PER_DIE 1024

// Instructions

#define W25N01G_INSTRUCTION_RDID             0x9F
#define W25N01G_INSTRUCTION_DEVICE_RESET     0xFF
#define W25N01G_INSTRUCTION_READ_STATUS_REG  0x05
#define W25N01G_INSTRUCTION_WRITE_STATUS_REG 0x01
#define W25N01G_INSTRUCTION_WRITE_ENABLE     0x06
#define W25N01G_INSTRUCTION_DIE_SELECT       0xC2
#define W25N01G_INSTRUCTION_BLOCK_ERASE      0xD8
#define W25N01G_INSTRUCTION_READ_BBM_LUT     0xA5
#define W25N01G_INSTRUCTION_BB_MANAGEMENT    0xA1
#define W25N01G_INSTRUCTION_PROGRAM_DATA_LOAD        0x02
#define W25N01G_INSTRUCTION_RANDOM_PROGRAM_DATA_LOAD 0x84
#define W25N01G_INSTRUCTION_PROGRAM_EXECUTE  0x10
#define W25N01G_INSTRUCTION_PAGE_DATA_READ   0x13
#define W25N01G_INSTRUCTION_READ_DATA        0x03
#define W25N01G_INSTRUCTION_FAST_READ        0x1B

// Configu/status register addresses
#define W25N01G_PROT_REG 0xA0
#define W25N01G_CONF_REG 0xB0
#define W25N01G_STAT_REG 0xC0

// Bits in config/status register 2 (W25N01G_CONF_REG)
#define W25N01G_CONFIG_ECC_ENABLE         (1 << 4)
#define W25N01G_CONFIG_BUFFER_READ_MODE   (1 << 3)

// Bits in config/status register 3 (W25N01G_STATREG)
#define W25N01G_STATUS_BBM_LUT_FULL       (1 << 6)
#define W25N01G_STATUS_FLAG_ECC_POS       4
#define W25N01G_STATUS_FLAG_ECC_MASK      ((1 << 5)|(1 << 4))
#define W25N01G_STATUS_FLAG_ECC(status)   (((status) & W25N01G_STATUS_FLAG_ECC_MASK) >> 4)
#define W25N01G_STATUS_PROGRAM_FAIL       (1 << 3)
#define W25N01G_STATUS_ERASE_FAIL         (1 << 2)
#define W25N01G_STATUS_FLAG_WRITE_ENABLED (1 << 1)
#define W25N01G_STATUS_FLAG_BUSY          (1 << 0)

// Bits in LBA for BB LUT
#define W25N01G_BBLUT_STATUS_ENABLED (1 << 15)
#define W25N01G_BBLUT_STATUS_INVALID (1 << 14)
#define W25N01G_BBLUT_STATUS_MASK    (W25N01G_BBLUT_STATUS_ENABLED | W25N01G_BBLUT_STATUS_INVALID)

// Some useful defs and macros
#define W25N01G_LINEAR_TO_COLUMN(laddr) ((laddr) % W25N01G_PAGE_SIZE)
#define W25N01G_LINEAR_TO_PAGE(laddr) ((laddr) / W25N01G_PAGE_SIZE)
#define W25N01G_LINEAR_TO_BLOCK(laddr) (W25N01G_LINEAR_TO_PAGE(laddr) / W25N01G_PAGES_PER_BLOCK)
#define W25N01G_BLOCK_TO_PAGE(block) ((block) * W25N01G_PAGES_PER_BLOCK)
#define W25N01G_BLOCK_TO_LINEAR(block) (W25N01G_BLOCK_TO_PAGE(block) * W25N01G_PAGE_SIZE)

// BB replacement area
#define W25N01G_BB_MARKER_BLOCKS           1
#define W25N01G_BB_REPLACEMENT_BLOCKS      21
#define W25N01G_BB_REPLACEMENT_START_BLOCK (W25N01G_BLOCKS_PER_DIE - W25N01G_BB_REPLACEMENT_BLOCKS)
#define W25N01G_BB_MARKER_BLOCK            (W25N01G_BB_REPLACEMENT_START_BLOCK - W25N01G_BB_MARKER_BLOCKS)

// The timeout values (2ms minimum to avoid 1 tick advance in consecutive calls to millis).
#define W25N01G_TIMEOUT_PAGE_READ_MS      2   // tREmax = 60us (ECC enabled)
#define W25N01G_TIMEOUT_PAGE_PROGRAM_MS   2   // tPPmax = 700us
#define W25N01G_TIMEOUT_BLOCK_ERASE_MS   15   // tBEmax = 10ms

// These will be gone

#define DISABLE(busdev)       IOHi((busdev)->busdev_u.spi.csnPin); __NOP()
#define ENABLE(busdev)        __NOP(); IOLo((busdev)->busdev_u.spi.csnPin)

/**
 * Send the given command byte to the device.
 */
static void w25n01g_performOneByteCommand(busDevice_t *busdev, uint8_t command)
{
    ENABLE(busdev);
    spiTransferByte(busdev->busdev_u.spi.instance, command);
    DISABLE(busdev);
}

static uint8_t w25n01g_readRegister(busDevice_t *busdev, uint8_t reg)
{
    const uint8_t cmd[3] = { W25N01G_INSTRUCTION_READ_STATUS_REG, reg, 0 };
    uint8_t in[3];

    ENABLE(busdev);
    spiTransfer(busdev->busdev_u.spi.instance, cmd, in, sizeof(cmd));
    DISABLE(busdev);

    return in[2];
}

static void w25n01g_writeRegister(busDevice_t *busdev, uint8_t reg, uint8_t data)
{
    const uint8_t cmd[3] = { W25N01G_INSTRUCTION_WRITE_STATUS_REG, reg, data };

    ENABLE(busdev);
    spiTransfer(busdev->busdev_u.spi.instance, cmd, NULL, sizeof(cmd));
    DISABLE(busdev);
}

static void w25n01g_deviceReset(busDevice_t *busdev)
{
    w25n01g_performOneByteCommand(busdev, W25N01G_INSTRUCTION_DEVICE_RESET);

    // Protection for upper 1/32 (BP[3:0] = 0101, TB=0), WP-E on; to protect bad block replacement area
    // DON'T DO THIS. This will prevent writes through the bblut as well.
    // w25n01g_writeRegister(busdev, W25N01G_PROT_REG, (5 << 3)|(0 << 2)|(1 << 1));

    // No protection, WP-E on
    w25n01g_writeRegister(busdev, W25N01G_PROT_REG, (0 << 3)|(0 << 2)|(1 << 1));

    // Buffered read mode (BUF = 1), ECC enabled (ECC = 1)
    w25n01g_writeRegister(busdev, W25N01G_CONF_REG, W25N01G_CONFIG_ECC_ENABLE|W25N01G_CONFIG_BUFFER_READ_MODE);
}

bool w25n01g_isReady(flashDevice_t *fdevice)
{
    // XXX Study device busy behavior and reinstate couldBeBusy facility.

#if 0
    // If couldBeBusy is false, don't bother to poll the flash chip for its status
    fdevice->couldBeBusy = fdevice->couldBeBusy && ((w25n01g_readRegister(fdevice->busdev, W25N01G_STAT_REG) & W25N01G_STATUS_FLAG_BUSY) != 0);

    return !couldBeBusy;
#else
    uint8_t status = w25n01g_readRegister(fdevice->busdev, W25N01G_STAT_REG);

    if (status & W25N01G_STATUS_PROGRAM_FAIL) {
        DPRINTF(("*** PROGRAM_FAIL\r\n"));
    }

    if (status & W25N01G_STATUS_ERASE_FAIL) {
        DPRINTF(("*** ERASE_FAIL\r\n"));
    }

    uint8_t eccCode;
    if ((eccCode = W25N01G_STATUS_FLAG_ECC(status))) {
        DPRINTF(("*** ECC %x\r\n", eccCode));
    }

    return ((status & W25N01G_STATUS_FLAG_BUSY) == 0);
#endif
}

bool w25n01g_waitForReady(flashDevice_t *fdevice, uint32_t timeoutMillis)
{
    uint32_t time = millis();
    while (!w25n01g_isReady(fdevice)) {
        if (millis() - time > timeoutMillis) {
            DPRINTF(("*** TIMEOUT %d\r\n", timeoutMillis));
            return false;
        }
    }

    return true;
}

/**
 * The flash requires this write enable command to be sent before commands that would cause
 * a write like program and erase.
 */
static void w25n01g_writeEnable(flashDevice_t *fdevice)
{
    w25n01g_performOneByteCommand(fdevice->busdev, W25N01G_INSTRUCTION_WRITE_ENABLE);

    // Assume that we're about to do some writing, so the device is just about to become busy
    fdevice->couldBeBusy = true;
}

/**
 * Read chip identification and geometry information (into global `geometry`).
 *
 * Returns true if we get valid ident, false if something bad happened like there is no M25P16.
 */
const flashVTable_t w25n01g_vTable;

static void w25n01g_deviceInit(flashDevice_t *flashdev);

bool w25n01g_detect(flashDevice_t *fdevice, uint32_t chipID)
{
#ifdef FLASH_W25N01G_DPRINTF
    // Setup debugSerialPort
    debugSerialPort = openSerialPort(DPRINTF_SERIAL_PORT, FUNCTION_NONE, NULL, NULL, 115200, MODE_RXTX, 0);

    if (debugSerialPort) {
        setPrintfSerialPort(debugSerialPort);
        DPRINTF(("debug print init: OK\r\n"));
    }
#endif

    switch (chipID) {
    case JEDEC_ID_WINBOND_W25N01GV:
    case JEDEC_ID_WINBOND_W25M02GV: // XXX Treat this as 1G part atm, as stacked W25N01G is not tested at all (yet).
        fdevice->geometry.sectors = 1024;      // Blocks
        fdevice->geometry.pagesPerSector = 64; // Pages/Blocks
        fdevice->geometry.pageSize = 2048;
        break;

    default:
        // Unsupported chip
        fdevice->geometry.sectors = 0;
        fdevice->geometry.pagesPerSector = 0;

        fdevice->geometry.sectorSize = 0;
        fdevice->geometry.totalSize = 0;
        return false;
    }

    fdevice->geometry.flashType = FLASH_TYPE_NAND;
    fdevice->geometry.sectors -= W25N01G_BB_REPLACEMENT_BLOCKS;
    fdevice->geometry.sectorSize = fdevice->geometry.pagesPerSector * fdevice->geometry.pageSize;
    fdevice->geometry.totalSize = fdevice->geometry.sectorSize * fdevice->geometry.sectors;

    fdevice->couldBeBusy = true; // Just for luck we'll assume the chip could be busy even though it isn't specced to be

    w25n01g_deviceReset(fdevice->busdev);

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

#if 0
    // Protection to upper 1/32 (BP[3:0] = 0101, TB=0), WP-E on
    //w25n01g_writeRegister(fdevice->busdev, W25N01G_PROT_REG, (5 << 3)|(0 << 2)|(1 << 1));

    // No protection, WP-E on
    w25n01g_writeRegister(fdevice->busdev, W25N01G_PROT_REG, (0 << 3)|(0 << 2)|(1 << 1));

    // Continuous mode (BUF = 0), ECC enabled (ECC = 1)
    w25n01g_writeRegister(fdevice->busdev, W25N01G_CONF_REG, W25N01G_CONFIG_ECC_ENABLE);
#endif

#if 0
    // XXX Should be gone in production
    uint8_t sr1, sr2, sr3;
    sr1 = w25n01g_readRegister(fdevice->busdev, W25N01G_PROT_REG);
    sr2 = w25n01g_readRegister(fdevice->busdev, W25N01G_CONF_REG);
    sr3 = w25n01g_readRegister(fdevice->busdev, W25N01G_STAT_REG);

    debug[1] = sr1;
    debug[2] = sr2;
    debug[3] = sr3;

    DPRINTF(("Detect: PROT 0x%x CONF 0x%x STAT 0x%x\r\n", sr1 & 0xff, sr2 & 0xff, sr3 & 0xff));
#endif

    w25n01g_deviceInit(fdevice);

    fdevice->vTable = &w25n01g_vTable;

    return true;
}

/**
 * Erase a sector full of bytes to all 1's at the given byte offset in the flash chip.
 */
void w25n01g_eraseSector(flashDevice_t *fdevice, uint32_t address)
{
    const uint8_t cmd[] = { W25N01G_INSTRUCTION_BLOCK_ERASE, 0, W25N01G_LINEAR_TO_PAGE(address) >> 8, W25N01G_LINEAR_TO_PAGE(address) & 0xff };

    w25n01g_waitForReady(fdevice, W25N01G_TIMEOUT_BLOCK_ERASE_MS);

    w25n01g_writeEnable(fdevice);

    ENABLE(fdevice->busdev);
    spiTransfer(fdevice->busdev->busdev_u.spi.instance, cmd, NULL, sizeof(cmd));
    DISABLE(fdevice->busdev);
}

//
// W25N01G does not support full chip erase.
// Call eraseSector repeatedly.

void w25n01g_eraseCompletely(flashDevice_t *fdevice)
{
    for (uint32_t block = 0; block < fdevice->geometry.sectors; block++) {
        w25n01g_waitForReady(fdevice, W25N01G_TIMEOUT_BLOCK_ERASE_MS);

        // Issue erase block command
        w25n01g_writeEnable(fdevice);
        w25n01g_eraseSector(fdevice, W25N01G_BLOCK_TO_LINEAR(block));
    }
}

static void w25n01g_programDataLoad(flashDevice_t *fdevice, uint16_t columnAddress, const uint8_t *data, int length)
{
    const uint8_t cmd[] = { W25N01G_INSTRUCTION_PROGRAM_DATA_LOAD, columnAddress >> 8, columnAddress& 0xff };

    //DPRINTF(("    load WaitForReady\r\n"));
    w25n01g_waitForReady(fdevice, W25N01G_TIMEOUT_PAGE_PROGRAM_MS);

    //DPRINTF(("    load Issuing command\r\n"));
    ENABLE(fdevice->busdev);
    spiTransfer(fdevice->busdev->busdev_u.spi.instance, cmd, NULL, sizeof(cmd));
    spiTransfer(fdevice->busdev->busdev_u.spi.instance, data, NULL, length);
    DISABLE(fdevice->busdev);
    //DPRINTF(("    load Done\r\n"));
}

static void w25n01g_randomProgramDataLoad(flashDevice_t *fdevice, uint16_t columnAddress, const uint8_t *data, int length)
{
    const uint8_t cmd[] = { W25N01G_INSTRUCTION_RANDOM_PROGRAM_DATA_LOAD, columnAddress >> 8, columnAddress& 0xff };

    //DPRINTF(("    random WaitForReady\r\n"));
    w25n01g_waitForReady(fdevice, W25N01G_TIMEOUT_PAGE_PROGRAM_MS);

    //DPRINTF(("    random Issuing command\r\n"));
    ENABLE(fdevice->busdev);
    spiTransfer(fdevice->busdev->busdev_u.spi.instance, cmd, NULL, sizeof(cmd));
    spiTransfer(fdevice->busdev->busdev_u.spi.instance, data, NULL, length);
    DISABLE(fdevice->busdev);
    //DPRINTF(("    random Done\r\n"));
}

static void w25n01g_programExecute(flashDevice_t *fdevice, uint32_t pageAddress)
{
    const uint8_t cmd[] = { W25N01G_INSTRUCTION_PROGRAM_EXECUTE, 0, pageAddress >> 8, pageAddress & 0xff };

    //DPRINTF(("    execute WaitForReady\r\n"));
    w25n01g_waitForReady(fdevice, W25N01G_TIMEOUT_PAGE_PROGRAM_MS);

    //DPRINTF(("    execute Issueing command\r\n"));
    ENABLE(fdevice->busdev);
    spiTransfer(fdevice->busdev->busdev_u.spi.instance, cmd, NULL, sizeof(cmd));
    DISABLE(fdevice->busdev);
    //DPRINTF(("    execute Done\r\n"));
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

pageProgramBegin:
If buffer is dirty and programLoadAddress != address, then the last page is a partial write;
issue PAGE_PROGRAM_EXECUTE to flash buffer contents, clear dirty and record the address as programLoadAddress and programStartAddress.
Else do nothing.

pageProgramContinue:
Mark buffer as dirty.
If programLoadAddress is on page boundary, then issue PROGRAM_LOAD_DATA, else issue RANDOM_PROGRAM_LOAD_DATA.
Update programLoadAddress.
Optionally observe the programLoadAddress, and if it's on page boundary, issue PAGE_PROGRAM_EXECUTE.

pageProgramFinish:
Observe programLoadAddress. If it's on page boundary, issue PAGE_PROGRAM_EXECUTE and clear dirty, else just return.
If pageProgramContinue observes the page boundary, then do nothing(?).
*/

static uint32_t programStartAddress;
static uint32_t programLoadAddress;
bool bufferDirty = false;
bool isProgramming = false;

#define DEBUG_PAGE_PROGRAM

//#define PAGEPROG_DPRINTF(x) DPRINTF(x)
#define PAGEPROG_DPRINTF(x)

void w25n01g_pageProgramBegin(flashDevice_t *fdevice, uint32_t address)
{
    PAGEPROG_DPRINTF(("pageProgramBegin: address 0x%x\r\n", address));

    if (bufferDirty) {
        if (address != programLoadAddress) {
            PAGEPROG_DPRINTF(("    Buffer dirty and address != programLoadAddress (0x%x), flushing\r\n", programLoadAddress));
            PAGEPROG_DPRINTF(("    Wait for ready\r\n"));
            w25n01g_waitForReady(fdevice, W25N01G_TIMEOUT_PAGE_PROGRAM_MS);

            isProgramming = false;

            PAGEPROG_DPRINTF(("    Write enable\r\n"));
            w25n01g_writeEnable(fdevice);

            PAGEPROG_DPRINTF(("    PROGRAM_EXECUTE PA 0x%x\r\n", W25N01G_LINEAR_TO_PAGE(programStartAddress)));
            w25n01g_programExecute(fdevice, W25N01G_LINEAR_TO_PAGE(programStartAddress));

            bufferDirty = false;
            isProgramming = true;
        } else {
            PAGEPROG_DPRINTF(("    Continuation\r\n"));
        }
    } else {
        PAGEPROG_DPRINTF(("    Fresh page\r\n"));
        programStartAddress = programLoadAddress = address;
    }
}

void w25n01g_pageProgramContinue(flashDevice_t *fdevice, const uint8_t *data, int length)
{
    PAGEPROG_DPRINTF(("pageProgramContinue: length 0x%x (programLoadAddress 0x%x)\r\n", length, programLoadAddress));

    // Check for page boundary overrun

    if (W25N01G_LINEAR_TO_PAGE(programLoadAddress + length - 1) != W25N01G_LINEAR_TO_PAGE(programStartAddress)) {
        PAGEPROG_DPRINTF(("    **** PAGE BOUNDARY OVERRUN **** (page 0x%x)\r\n", W25N01G_LINEAR_TO_PAGE(programLoadAddress)));
    }

    PAGEPROG_DPRINTF(("    Wait for ready\r\n"));
    w25n01g_waitForReady(fdevice, W25N01G_TIMEOUT_PAGE_PROGRAM_MS);

    PAGEPROG_DPRINTF(("    Write enable\r\n"));
    w25n01g_writeEnable(fdevice);

    isProgramming = false;

    if (!bufferDirty) {
        PAGEPROG_DPRINTF(("    DATA_LOAD CA 0x%x length 0x%x\r\n", W25N01G_LINEAR_TO_COLUMN(programLoadAddress), length));
        w25n01g_programDataLoad(fdevice, W25N01G_LINEAR_TO_COLUMN(programLoadAddress), data, length);
    } else {
        PAGEPROG_DPRINTF(("    RANDOM_DATA_LOAD CA 0x%x length 0x%x\r\n", W25N01G_LINEAR_TO_COLUMN(programLoadAddress), length));
        w25n01g_randomProgramDataLoad(fdevice, W25N01G_LINEAR_TO_COLUMN(programLoadAddress), data, length);
    }

    // XXX Test if write enable is reset after each data loading.

    bufferDirty = true;
    programLoadAddress += length;
}

void w25n01g_pageProgramFinish(flashDevice_t *fdevice)
{
    PAGEPROG_DPRINTF(("pageProgramFinish: (loaded 0x%x bytes)\r\n", programLoadAddress - programStartAddress));

    if (bufferDirty && W25N01G_LINEAR_TO_COLUMN(programLoadAddress) == 0) {
        PAGEPROG_DPRINTF(("    PROGRAM_EXECUTE PA 0x%x\r\n", W25N01G_LINEAR_TO_PAGE(programStartAddress)));
        w25n01g_programExecute(fdevice, W25N01G_LINEAR_TO_PAGE(programStartAddress));

        bufferDirty = false;
        isProgramming = true;

        programStartAddress = programLoadAddress;
    } else {
        PAGEPROG_DPRINTF(("    Ignoring\r\n"));
    }
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

void w25n01g_pageProgram(flashDevice_t *fdevice, uint32_t address, const uint8_t *data, int length)
{
    w25n01g_pageProgramBegin(fdevice, address);
    w25n01g_pageProgramContinue(fdevice, data, length);
    w25n01g_pageProgramFinish(fdevice);
}

void w25n01g_flush(flashDevice_t *fdevice)
{
    PAGEPROG_DPRINTF(("close:\r\n"));

    if (bufferDirty) {
        PAGEPROG_DPRINTF(("    Buffer is partially loaded (0x%x bytes)\r\n", programLoadAddress - programStartAddress));
        PAGEPROG_DPRINTF(("    PROGRAM_EXECUTE PA 0x%x\r\n", W25N01G_LINEAR_TO_PAGE(programStartAddress)));

        w25n01g_programExecute(fdevice, W25N01G_LINEAR_TO_PAGE(programStartAddress));

        bufferDirty = false;
        isProgramming = true;
    } else {
        PAGEPROG_DPRINTF(("    Buffer is clean\r\n"));
        isProgramming = false;
    }
}

void w25n01g_addError(uint32_t address, uint8_t code)
{
    UNUSED(address);
    UNUSED(code);
    DPRINTF(("addError: PA %x BA %x code %d\r\n", W25N01G_LINEAR_TO_PAGE(address), W25N01G_LINEAR_TO_BLOCK(address), code));
}

/**
 * Read `length` bytes into the provided `buffer` from the flash starting from the given `address` (which need not lie
 * on a page boundary).
 *
 * Waits up to W25N01G_TIMEOUT_PAGE_READ_MS milliseconds for the flash to become ready before reading.
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

//#define READBYTES_DPRINTF DPRINTF
#define READBYTES_DPRINTF(x)

int w25n01g_readBytes(flashDevice_t *fdevice, uint32_t address, uint8_t *buffer, int length)
{
    static uint32_t currentPage = UINT32_MAX;
    uint8_t cmd[4];

    READBYTES_DPRINTF(("readBytes: address 0x%x length %d\r\n", address, length));

    uint32_t targetPage = W25N01G_LINEAR_TO_PAGE(address);

    if (currentPage != targetPage) {
        READBYTES_DPRINTF(("readBytes: PAGE_DATA_READ page 0x%x\r\n", targetPage));

        cmd[0] = W25N01G_INSTRUCTION_PAGE_DATA_READ;
        cmd[1] = 0;
        cmd[2] = targetPage >> 8;
        cmd[3] = targetPage;

        if (!w25n01g_waitForReady(fdevice, W25N01G_TIMEOUT_PAGE_READ_MS)) {
            return 0;
        }

        currentPage = UINT32_MAX;

        ENABLE(fdevice->busdev);
        spiTransfer(fdevice->busdev->busdev_u.spi.instance, cmd, NULL, 4);
        DISABLE(fdevice->busdev);

        if (!w25n01g_waitForReady(fdevice, W25N01G_TIMEOUT_PAGE_READ_MS)) {
            return 0;
        }

        currentPage = targetPage;
    }

    uint16_t column = W25N01G_LINEAR_TO_COLUMN(address);
    uint16_t transferLength;

    if (length > W25N01G_PAGE_SIZE - column) {
        transferLength = W25N01G_PAGE_SIZE - column;
    } else {
        transferLength = length;
    }

    cmd[0] = W25N01G_INSTRUCTION_READ_DATA;
    cmd[1] = column >> 8;
    cmd[2] = column;
    cmd[3] = 0;

    READBYTES_DPRINTF(("readBytes: READ_DATA column 0x%x transferLength 0x%x\r\n", column, transferLength));

    ENABLE(fdevice->busdev);
    spiTransfer(fdevice->busdev->busdev_u.spi.instance, cmd, NULL, 4);
    spiTransfer(fdevice->busdev->busdev_u.spi.instance, NULL, buffer, length);
    DISABLE(fdevice->busdev);

    // XXX Don't need this?
    if (!w25n01g_waitForReady(fdevice, W25N01G_TIMEOUT_PAGE_READ_MS)) {
        return 0;
    }

    // Check ECC

    uint8_t statReg = w25n01g_readRegister(fdevice->busdev, W25N01G_STAT_REG);
    uint8_t eccCode = W25N01G_STATUS_FLAG_ECC(statReg);

    switch (eccCode) {
    case 0: // Successful read, no ECC correction
        break;
    case 1: // Successful read with ECC correction
    case 2: // Uncorrectable ECC in a single page
    case 3: // Uncorrectable ECC in multiple pages
        w25n01g_addError(address, eccCode);
        w25n01g_deviceReset(fdevice->busdev);
        break;
    }

    READBYTES_DPRINTF(("readBytes: transfered 0x%x bytes\r\n", transferLength));

    return transferLength;
}

int w25n01g_readExtensionBytes(flashDevice_t *fdevice, uint32_t address, uint8_t *buffer, int length)
{
    uint8_t cmd[4];

    cmd[0] = W25N01G_INSTRUCTION_PAGE_DATA_READ;
    cmd[1] = 0;
    cmd[2] = W25N01G_LINEAR_TO_PAGE(address) >> 8;
    cmd[3] = W25N01G_LINEAR_TO_PAGE(address);

    ENABLE(fdevice->busdev);
    spiTransfer(fdevice->busdev->busdev_u.spi.instance, cmd, NULL, 4);
    DISABLE(fdevice->busdev);

    if (!w25n01g_waitForReady(fdevice, W25N01G_TIMEOUT_PAGE_READ_MS)) {
        return 0;
    }

    cmd[0] = W25N01G_INSTRUCTION_READ_DATA;
    cmd[1] = 0;
    cmd[2] = (2048 >> 8) & 0xff;
    cmd[3] = 2048 & 0xff;

    ENABLE(fdevice->busdev);
    spiTransfer(fdevice->busdev->busdev_u.spi.instance, cmd, NULL, 4);
    spiTransfer(fdevice->busdev->busdev_u.spi.instance, NULL, buffer, length);
    DISABLE(fdevice->busdev);

    return length;
}

/**
 * Fetch information about the detected flash chip layout.
 *
 * Can be called before calling w25n01g_init() (the result would have totalSize = 0).
 */
const flashGeometry_t* w25n01g_getGeometry(flashDevice_t *fdevice)
{
    return &fdevice->geometry;
}

const flashVTable_t w25n01g_vTable = {
    .isReady = w25n01g_isReady,
    .waitForReady = w25n01g_waitForReady,
    .eraseSector = w25n01g_eraseSector,
    .eraseCompletely = w25n01g_eraseCompletely,
    .pageProgramBegin = w25n01g_pageProgramBegin,
    .pageProgramContinue = w25n01g_pageProgramContinue,
    .pageProgramFinish = w25n01g_pageProgramFinish,
    .pageProgram = w25n01g_pageProgram,
    .flush = w25n01g_flush,
    .readBytes = w25n01g_readBytes,
    .getGeometry = w25n01g_getGeometry,
};

void w25n01g_readBBLUT(flashDevice_t *fdevice, bblut_t *bblut, int lutsize)
{
    uint8_t cmd[4];
    uint8_t in[4];

    cmd[0] = W25N01G_INSTRUCTION_READ_BBM_LUT;
    cmd[1] = 0;

    ENABLE(fdevice->busdev);

    spiTransfer(fdevice->busdev->busdev_u.spi.instance, cmd, NULL, 2);

    for (int i = 0 ; i < lutsize ; i++) {
        spiTransfer(fdevice->busdev->busdev_u.spi.instance, NULL, in, 4);
        bblut[i].pba = (in[0] << 16)|in[1];
        bblut[i].lba = (in[2] << 16)|in[3];
    }

    DISABLE(fdevice->busdev);
}

void w25n01g_writeBBLUT(flashDevice_t *fdevice, uint16_t lba, uint16_t pba)
{
    uint8_t cmd[5] = { W25N01G_INSTRUCTION_BB_MANAGEMENT, lba >> 8, lba, pba >> 8, pba };

    ENABLE(fdevice->busdev);
    spiTransfer(fdevice->busdev->busdev_u.spi.instance, cmd, NULL, sizeof(cmd));
    DISABLE(fdevice->busdev);

    w25n01g_waitForReady(fdevice, W25N01G_TIMEOUT_PAGE_PROGRAM_MS);
}

static void w25n01g_deviceInit(flashDevice_t *flashdev)
{
    UNUSED(flashdev);
}
#endif
