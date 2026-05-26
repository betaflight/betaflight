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

/*
 * Macronix MX66UW1G45G — 1Gbit (128 MiB) Octal NOR Flash.
 *
 * 1S-1S-1S indirect read/write/sector-erase through the platform HAL's
 * XSPI primitives. The OBL flash-update path uses the same pattern
 * against the same chip and is the proven reference for post-erase
 * RDSR while WIP=1 — BF's hand-rolled XSPI sequencer in
 * bus_octospi_stm32n6xx.c does not handle that case reliably.
 *
 * The N6 boot ROM hands BF a chip that the FSBL stub / OpenBootloader
 * has already walked back from 8S-8S-8D OPI/DTR to 1S-1S-1S via a soft
 * reset, with the XSPI controller configured for memory-mapped
 * FAST_READ_4B (0x0C + 8 dummy, 4-byte address). Everything here
 * assumes that handoff state and never re-initialises the controller.
 *
 * HAL XSPI module and HAL core (HAL_GetTick) are pulled into the
 * .ram_code section by the N657 XIP linker script so these calls are
 * safe while memory-mapped mode is disabled.
 *
 * Selection is build-time via OCTOSPI_FLASH_CHIP=MX66UW1G45G in the
 * per-config config.mk, which emits both USE_FLASH_MX66UW1G45G and
 * OCTOSPI_FLASH_CHIP_MX66UW1G45G — the chip cannot answer 1/4-line
 * RDID while configured for OPI, so JEDEC probing isn't an option.
 *
 * Read path uses indirect FAST_READ_4B regardless of MM state, so the
 * caller (config_eeprom.c) can disable MM mode before reading config
 * back to RAM without special-casing the load.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_FLASH_MX66UW1G45G) && defined(USE_OCTOSPI)

#include "stm32n6xx_hal.h"

#include "common/utils.h"

#include "drivers/bus_octospi.h"
#include "drivers/flash/flash.h"
#include "drivers/flash/flash_impl.h"
#include "drivers/flash/flash_mx66uw1g45g.h"
#include "drivers/system.h"
#include "drivers/time.h"

// Geometry: 1Gbit / 128 MiB, 4 KiB sectors, 256 B pages.
#define MX66UW1G45G_PAGE_SIZE           256U
#define MX66UW1G45G_SECTOR_SIZE         4096U
#define MX66UW1G45G_PAGES_PER_SECTOR    (MX66UW1G45G_SECTOR_SIZE / MX66UW1G45G_PAGE_SIZE)
#define MX66UW1G45G_SECTORS             32768U   // 32768 * 4 KiB = 128 MiB

#ifndef MX66UW1G45G_MEMORY_MAPPED_BASE
#define MX66UW1G45G_MEMORY_MAPPED_BASE  0x70000000U
#endif

// 1S-1S-1S command set (post-reset state).
#define MX66_CMD_RDSR                   0x05U   // read status register
#define MX66_CMD_WREN                   0x06U   // write enable
#define MX66_CMD_READ_4B                0x0CU   // 4-byte FAST_READ
#define MX66_CMD_READ_4B_DUMMY          8U
#define MX66_CMD_PP_4B                  0x12U   // 4-byte page program
#define MX66_CMD_SE_4B                  0x21U   // 4-byte 4 KiB sector erase

#define MX66_SR_WIP                     0x01U   // write-in-progress bit
#define MX66_SR_WEL                     0x02U   // write-enable latch bit

// Datasheet typ/max values (Rev 0.91, Macronix MX66UW1G45G):
//   tPP  page program:  150 us typ / 1 ms max   -> 50 ms safety margin
//   tSE  sector erase:  35 ms typ  / 400 ms max -> 800 ms safety margin
#define MX66_TIMEOUT_PROGRAM_MS         50U
#define MX66_TIMEOUT_SECTOR_MS          800U

// HAL handle is allocated in fastram_data and statically initialised
// against XSPI2. The controller itself is left as the bootloader set
// it up; this handle exists only so HAL_XSPI_Command/_Receive/_Transmit
// have somewhere to read Init.MemoryMode/.MemoryType and track State.
MMFLASH_DATA static XSPI_HandleTypeDef hxspi_mx66 = {
    .Instance = XSPI2,
    .Init = {
        .MemoryMode     = HAL_XSPI_SINGLE_MEM,
        .MemoryType     = HAL_XSPI_MEMTYPE_MACRONIX,
        .SampleShifting = HAL_XSPI_SAMPLE_SHIFT_NONE,
    },
    .State     = HAL_XSPI_STATE_READY,
    .ErrorCode = HAL_XSPI_ERROR_NONE,
};

MMFLASH_DATA static flashVTable_t mx66uw1g45g_vTable;

// Latched once the chip driver hits an unrecoverable error during erase
// or program (timeout waiting for WIP=0, WREN failed, HAL_XSPI_Command
// rejected the opcode). Drives mx66uw1g45g_isReady / _waitForReady to
// report "not ready" so the meta-driver's flashWaitForReadyOrFail trips
// failureMode instead of treating a skipped operation as success.
MMFLASH_DATA static bool mx66_error_latched;

MMFLASH_CODE_NOINLINE bool mx66uw1g45g_identify(flashDevice_t *fdevice, uint32_t jedecID)
{
    if (jedecID != MX66UW1G45G_JEDEC_ID) {
        fdevice->geometry.sectors = 0;
        fdevice->geometry.pagesPerSector = 0;
        fdevice->geometry.sectorSize = 0;
        fdevice->geometry.totalSize = 0;
        return false;
    }

    fdevice->geometry.flashType = FLASH_TYPE_NOR;
    fdevice->geometry.sectors = MX66UW1G45G_SECTORS;
    fdevice->geometry.pagesPerSector = MX66UW1G45G_PAGES_PER_SECTOR;
    fdevice->geometry.pageSize = MX66UW1G45G_PAGE_SIZE;
    fdevice->geometry.sectorSize = MX66UW1G45G_SECTOR_SIZE;
    fdevice->geometry.totalSize = (uint32_t)MX66UW1G45G_SECTOR_SIZE * MX66UW1G45G_SECTORS;

    fdevice->vTable = &mx66uw1g45g_vTable;

    return true;
}

static void mx66uw1g45g_configure(flashDevice_t *fdevice, uint32_t configurationFlags)
{
    UNUSED(fdevice);
    UNUSED(configurationFlags);
    // Bootloader has configured OPI memory-mapped mode; nothing to do.
}

// Pre-fill the command struct with the immutable fields shared by every
// 1S-1S-1S command this driver issues. Caller adjusts Instruction /
// AddressMode / AddressWidth / Address / DataMode / DataLength.
MMFLASH_CODE static void mx66_prepare_cmd_1s(XSPI_RegularCmdTypeDef *cmd)
{
    cmd->OperationType         = HAL_XSPI_OPTYPE_COMMON_CFG;
    cmd->IOSelect              = HAL_XSPI_SELECT_IO_7_0;
    cmd->InstructionMode       = HAL_XSPI_INSTRUCTION_1_LINE;
    cmd->InstructionWidth      = HAL_XSPI_INSTRUCTION_8_BITS;
    cmd->InstructionDTRMode    = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    cmd->AddressMode           = HAL_XSPI_ADDRESS_NONE;
    cmd->AddressWidth          = HAL_XSPI_ADDRESS_32_BITS;
    cmd->AddressDTRMode        = HAL_XSPI_ADDRESS_DTR_DISABLE;
    cmd->Address               = 0;
    cmd->AlternateBytes        = 0;
    cmd->AlternateBytesMode    = HAL_XSPI_ALT_BYTES_NONE;
    cmd->AlternateBytesWidth   = HAL_XSPI_ALT_BYTES_8_BITS;
    cmd->AlternateBytesDTRMode = HAL_XSPI_ALT_BYTES_DTR_DISABLE;
    cmd->DataMode              = HAL_XSPI_DATA_NONE;
    cmd->DataLength            = 0;
    cmd->DataDTRMode           = HAL_XSPI_DATA_DTR_DISABLE;
    cmd->DummyCycles           = 0;
    cmd->DQSMode               = HAL_XSPI_DQS_DISABLE;
}

MMFLASH_CODE static bool mx66_wait_ready(uint32_t timeoutMs)
{
    XSPI_RegularCmdTypeDef cmd = {0};
    uint8_t status;
    uint32_t tickstart = HAL_GetTick();

    mx66_prepare_cmd_1s(&cmd);
    cmd.Instruction = MX66_CMD_RDSR;
    cmd.DataMode    = HAL_XSPI_DATA_1_LINE;
    cmd.DataLength  = 1;

    do {
        if (HAL_XSPI_Command(&hxspi_mx66, &cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
            return false;
        }
        if (HAL_XSPI_Receive(&hxspi_mx66, &status, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
            return false;
        }
        if ((status & MX66_SR_WIP) == 0) {
            return true;
        }
    } while ((HAL_GetTick() - tickstart) < timeoutMs);

    return false;
}

MMFLASH_CODE static uint8_t mx66_readStatus(void)
{
    XSPI_RegularCmdTypeDef cmd = {0};
    uint8_t status = 0xFFU;

    mx66_prepare_cmd_1s(&cmd);
    cmd.Instruction = MX66_CMD_RDSR;
    cmd.DataMode    = HAL_XSPI_DATA_1_LINE;
    cmd.DataLength  = 1;

    if (HAL_XSPI_Command(&hxspi_mx66, &cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return 0xFFU;
    }
    if (HAL_XSPI_Receive(&hxspi_mx66, &status, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return 0xFFU;
    }
    return status;
}

MMFLASH_CODE static bool mx66_write_enable(void)
{
    XSPI_RegularCmdTypeDef cmd = {0};
    mx66_prepare_cmd_1s(&cmd);
    cmd.Instruction = MX66_CMD_WREN;
    if (HAL_XSPI_Command(&hxspi_mx66, &cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return false;
    }
    // Verify the chip actually latched WEL before treating WREN as a
    // success. Per the MX66 datasheet, PP_4B / SE_4B are ignored when
    // WEL=0, and the subsequent waitForReady poll sees WIP=0
    // immediately — silently turning a dropped WREN into a no-op
    // write. mx66_readStatus returns 0xFF on its own HAL failures, so
    // treat that as a fault too.
    const uint8_t status = mx66_readStatus();
    return (status != 0xFFU) && ((status & MX66_SR_WEL) != 0U);
}

MMFLASH_CODE static bool mx66uw1g45g_isReady(flashDevice_t *fdevice)
{
    UNUSED(fdevice);
    if (mx66_error_latched) {
        return false;
    }
    return (mx66_readStatus() & MX66_SR_WIP) == 0;
}

MMFLASH_CODE static bool mx66uw1g45g_waitForReady(flashDevice_t *fdevice)
{
    UNUSED(fdevice);
    if (mx66_error_latched) {
        return false;
    }
    return mx66_wait_ready(MX66_TIMEOUT_SECTOR_MS);
}

MMFLASH_CODE static void mx66uw1g45g_eraseSector(flashDevice_t *fdevice, uint32_t address)
{
    UNUSED(fdevice);

    if (!mx66_wait_ready(MX66_TIMEOUT_SECTOR_MS) ||
        !mx66_write_enable()) {
        mx66_error_latched = true;
        return;
    }

    XSPI_RegularCmdTypeDef cmd = {0};
    mx66_prepare_cmd_1s(&cmd);
    cmd.Instruction  = MX66_CMD_SE_4B;
    cmd.AddressMode  = HAL_XSPI_ADDRESS_1_LINE;
    cmd.AddressWidth = HAL_XSPI_ADDRESS_32_BITS;
    cmd.Address      = address;

    if (HAL_XSPI_Command(&hxspi_mx66, &cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        mx66_error_latched = true;
        return;
    }
    // The wait-for-completion poll happens via flashWaitForReadyOrFail
    // after we return; tSE allowance lives in MX66_TIMEOUT_SECTOR_MS.
}

static void mx66uw1g45g_eraseCompletely(flashDevice_t *fdevice)
{
    // No use case in BF for whole-chip erase on the boot chip; refuse rather
    // than silently destroying the firmware that's running.
    UNUSED(fdevice);
    failureMode(FAILURE_FLASH_WRITE_FAILED);
}

MMFLASH_CODE static void mx66uw1g45g_pageProgramBegin(flashDevice_t *fdevice, uint32_t address, void (*callback)(uintptr_t arg))
{
    fdevice->callback = callback;
    fdevice->currentWriteAddress = address;
    fdevice->bytesWritten = 0;
}

MMFLASH_CODE static bool mx66_page_program(uint32_t address, const uint8_t *data, uint32_t length)
{
    if (length == 0 || length > MX66UW1G45G_PAGE_SIZE) {
        return false;
    }
    // The chip wraps writes at the page boundary; caller must split
    // straddles or the early bytes of the same page get clobbered.
    if (((address & (MX66UW1G45G_PAGE_SIZE - 1)) + length) > MX66UW1G45G_PAGE_SIZE) {
        return false;
    }

    if (!mx66_wait_ready(MX66_TIMEOUT_PROGRAM_MS)) {
        return false;
    }
    if (!mx66_write_enable()) {
        return false;
    }

    XSPI_RegularCmdTypeDef cmd = {0};
    mx66_prepare_cmd_1s(&cmd);
    cmd.Instruction  = MX66_CMD_PP_4B;
    cmd.AddressMode  = HAL_XSPI_ADDRESS_1_LINE;
    cmd.AddressWidth = HAL_XSPI_ADDRESS_32_BITS;
    cmd.Address      = address;
    cmd.DataMode     = HAL_XSPI_DATA_1_LINE;
    cmd.DataLength   = length;

    if (HAL_XSPI_Command(&hxspi_mx66, &cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return false;
    }
    return HAL_XSPI_Transmit(&hxspi_mx66, (uint8_t *)data, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
}

MMFLASH_CODE static uint32_t mx66uw1g45g_pageProgramContinue(flashDevice_t *fdevice, uint8_t const **buffers, const uint32_t *bufferSizes, uint32_t bufferCount)
{
    uint32_t totalWritten = 0;

    for (uint32_t i = 0; i < bufferCount; i++) {
        const uint8_t *data  = buffers[i];
        uint32_t       remaining = bufferSizes[i];

        while (remaining > 0) {
            uint32_t pageOffset = fdevice->currentWriteAddress & (MX66UW1G45G_PAGE_SIZE - 1);
            uint32_t chunk      = MX66UW1G45G_PAGE_SIZE - pageOffset;
            if (chunk > remaining) {
                chunk = remaining;
            }
            if (!mx66_page_program(fdevice->currentWriteAddress, data, chunk)) {
                // Latch the error so the meta-driver's next
                // flashWaitForReadyOrFail trips failureMode instead of
                // accepting a short write as success.
                mx66_error_latched = true;
                fdevice->bytesWritten += totalWritten;
                return totalWritten;
            }
            fdevice->currentWriteAddress += chunk;
            data         += chunk;
            remaining    -= chunk;
            totalWritten += chunk;
        }
    }

    fdevice->bytesWritten += totalWritten;
    if (fdevice->callback) {
        fdevice->callback(totalWritten);
    }
    return totalWritten;
}

MMFLASH_CODE static void mx66uw1g45g_pageProgramFinish(flashDevice_t *fdevice)
{
    UNUSED(fdevice);
}

MMFLASH_CODE static void mx66uw1g45g_pageProgram(flashDevice_t *fdevice, uint32_t address, const uint8_t *data, uint32_t length, void (*callback)(uintptr_t arg))
{
    mx66uw1g45g_pageProgramBegin(fdevice, address, callback);
    mx66uw1g45g_pageProgramContinue(fdevice, &data, &length, 1);
    mx66uw1g45g_pageProgramFinish(fdevice);
}

MMFLASH_CODE static void mx66uw1g45g_flush(flashDevice_t *fdevice)
{
    UNUSED(fdevice);
}

MMFLASH_CODE static int mx66uw1g45g_readBytes(flashDevice_t *fdevice, uint32_t address, uint8_t *buffer, uint32_t length)
{
    if (address >= fdevice->geometry.totalSize) {
        return 0;
    }
    const uint32_t remaining = fdevice->geometry.totalSize - address;
    if (length > remaining) {
        length = remaining;
    }

    XSPI_RegularCmdTypeDef cmd = {0};
    mx66_prepare_cmd_1s(&cmd);
    cmd.Instruction  = MX66_CMD_READ_4B;
    cmd.AddressMode  = HAL_XSPI_ADDRESS_1_LINE;
    cmd.AddressWidth = HAL_XSPI_ADDRESS_32_BITS;
    cmd.Address      = address;
    cmd.DataMode     = HAL_XSPI_DATA_1_LINE;
    cmd.DataLength   = length;
    cmd.DummyCycles  = MX66_CMD_READ_4B_DUMMY;

    if (HAL_XSPI_Command(&hxspi_mx66, &cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return 0;
    }
    if (HAL_XSPI_Receive(&hxspi_mx66, buffer, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return 0;
    }
    return (int)length;
}

static const flashGeometry_t *mx66uw1g45g_getGeometry(flashDevice_t *fdevice)
{
    return &fdevice->geometry;
}

MMFLASH_DATA static flashVTable_t mx66uw1g45g_vTable = {
    .configure = mx66uw1g45g_configure,
    .isReady = mx66uw1g45g_isReady,
    .waitForReady = mx66uw1g45g_waitForReady,
    .eraseSector = mx66uw1g45g_eraseSector,
    .eraseCompletely = mx66uw1g45g_eraseCompletely,
    .pageProgramBegin = mx66uw1g45g_pageProgramBegin,
    .pageProgramContinue = mx66uw1g45g_pageProgramContinue,
    .pageProgramFinish = mx66uw1g45g_pageProgramFinish,
    .pageProgram = mx66uw1g45g_pageProgram,
    .flush = mx66uw1g45g_flush,
    .readBytes = mx66uw1g45g_readBytes,
    .getGeometry = mx66uw1g45g_getGeometry,
};

#endif
