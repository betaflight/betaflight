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
 * Two handoff states are supported, because the first stage decides
 * which one BF inherits and never tells it:
 *
 *   1S-1S-1S — the FSBL stub / OpenBootloader has walked the chip back
 *     from OPI via a soft reset, and configured the controller for
 *     memory-mapped FAST_READ_4B (0x0C + 8 dummy, 4-byte address).
 *
 *   8D-8D-8D — the first stage left the chip in octal DTR, the mode it
 *     boots fastest in. A 1-line command then gets no answer at all:
 *     RDSR returns nothing, WIP never clears, and mx66_wait_ready spins
 *     until the timeout with no fault raised.
 *
 * Which one is in force is read back from the controller itself at
 * identify time (mx66_sample_boot_configuration), before anything here
 * touches it. That also captures the read opcode and dummy-cycle count
 * the first stage chose, so our indirect reads are byte-for-byte the
 * command the memory-mapped window is already using successfully —
 * rather than a second guess at the chip's latency configuration.
 *
 * Either way the controller is never re-initialised.
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

// 8D-8D-8D command set. Octal opcodes are complemented 16-bit pairs,
// (op << 8) | (~op & 0xFF), and the chip rejects anything else once it
// is in OPI. Values cross-checked against the pair the vendor first
// stage had left in XSPI2->IR (0xEE11) and WPIR (0x12ED).
#define MX66_OCMD_RDSR                  0x05FAU
#define MX66_OCMD_WREN                  0x06F9U
#define MX66_OCMD_READ_DTR              0xEE11U
#define MX66_OCMD_PP_4B                 0x12EDU
#define MX66_OCMD_SE_4B                 0x21DEU

// Register reads carry their own latency, independent of the data
// latency in CR2. ST's component driver uses 5 for the DTR case across
// every N6 board support package; the STR case is 4.
#ifndef MX66_OCMD_REG_DUMMY
#define MX66_OCMD_REG_DUMMY             5U
#endif

// Fallback data latency, used only if the first stage left no usable
// dummy-cycle count behind. 20 is the chip's CR2 reset default.
#define MX66_OCMD_READ_DUMMY_DEFAULT    20U

// In OPI the status register comes back duplicated, one copy per edge
// of the DTR pair, and the controller refuses odd transfer lengths.
#define MX66_OCMD_RDSR_LENGTH           2U

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

// Sampled from the controller once, at identify time, while the first
// stage's memory-mapped configuration is still untouched. See
// mx66_sample_boot_configuration.
MMFLASH_DATA static bool mx66_opi;
MMFLASH_DATA static uint16_t mx66_read_instruction = MX66_OCMD_READ_DTR;
MMFLASH_DATA static uint8_t mx66_read_dummy = MX66_OCMD_READ_DUMMY_DEFAULT;

/*
 * Work out which wire format the first stage left the chip in, and on
 * what terms.
 *
 * The controller is the only witness: the chip cannot be asked, since
 * asking requires already knowing how to address it. But CCR/IR/TCR
 * still hold the command the first stage installed for memory-mapped
 * reads, and that command is known-good — it is what the 0x70000000
 * window has been serving since boot.
 *
 * CCR.IMODE == 4 means the instruction phase runs on eight lines, which
 * only happens once the chip is in OPI. Anything else is treated as the
 * 1S-1S-1S handoff, including a controller that was never configured.
 *
 * Must be called before memory-mapped mode is disabled for the first
 * time; afterwards these registers hold whatever indirect command ran
 * last.
 */
MMFLASH_CODE_NOINLINE static void mx66_sample_boot_configuration(void)
{
    const XSPI_TypeDef *instance = hxspi_mx66.Instance;

    mx66_opi = (READ_BIT(instance->CCR, XSPI_CCR_IMODE) == XSPI_CCR_IMODE_2);

    if (!mx66_opi) {
        return;
    }

    // Inherit the first stage's read command rather than re-deriving
    // it. The dummy-cycle count in particular depends on the chip's CR2
    // latency configuration, which we have no way to read back without
    // first getting a register read to work.
    const uint32_t bootInstruction = READ_REG(instance->IR) & 0xFFFFU;
    if (bootInstruction != 0) {
        mx66_read_instruction = (uint16_t)bootInstruction;
    }

    const uint32_t bootDummy = READ_BIT(instance->TCR, XSPI_TCR_DCYC) >> XSPI_TCR_DCYC_Pos;
    if (bootDummy != 0) {
        mx66_read_dummy = (uint8_t)bootDummy;
    }
}

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

    mx66_sample_boot_configuration();

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

// Same, for 8D-8D-8D. Address fields are pre-filled even though the
// address phase starts disabled, so a caller only has to set
// AddressMode to turn it on.
//
// DQS stays off here. The chip drives it as a read strobe only, so it
// belongs to the commands that receive data (read, RDSR) and not to
// WREN, erase or page program — which is also how ST's component driver
// splits it.
MMFLASH_CODE static void mx66_prepare_cmd_8d(XSPI_RegularCmdTypeDef *cmd)
{
    cmd->OperationType         = HAL_XSPI_OPTYPE_COMMON_CFG;
    cmd->IOSelect              = HAL_XSPI_SELECT_IO_7_0;
    cmd->InstructionMode       = HAL_XSPI_INSTRUCTION_8_LINES;
    cmd->InstructionWidth      = HAL_XSPI_INSTRUCTION_16_BITS;
    cmd->InstructionDTRMode    = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
    cmd->AddressMode           = HAL_XSPI_ADDRESS_NONE;
    cmd->AddressWidth          = HAL_XSPI_ADDRESS_32_BITS;
    cmd->AddressDTRMode        = HAL_XSPI_ADDRESS_DTR_ENABLE;
    cmd->Address               = 0;
    cmd->AlternateBytes        = 0;
    cmd->AlternateBytesMode    = HAL_XSPI_ALT_BYTES_NONE;
    cmd->AlternateBytesWidth   = HAL_XSPI_ALT_BYTES_8_BITS;
    cmd->AlternateBytesDTRMode = HAL_XSPI_ALT_BYTES_DTR_DISABLE;
    cmd->DataMode              = HAL_XSPI_DATA_NONE;
    cmd->DataLength            = 0;
    cmd->DataDTRMode           = HAL_XSPI_DATA_DTR_ENABLE;
    cmd->DummyCycles           = 0;
    cmd->DQSMode               = HAL_XSPI_DQS_DISABLE;
}

/*
 * Build a status-register read for whichever mode is in force.
 *
 * The octal form is where the two wire formats differ most, and where
 * getting it wrong is hardest to see: RDSR takes a full 32-bit address
 * phase in OPI, where in 1S it takes none at all. Issue the 1-line form
 * to a chip in OPI and it simply does not answer — the poll loop below
 * then runs to its timeout without a single fault being raised.
 */
MMFLASH_CODE static void mx66_prepare_readStatus(XSPI_RegularCmdTypeDef *cmd)
{
    if (mx66_opi) {
        mx66_prepare_cmd_8d(cmd);
        cmd->Instruction = MX66_OCMD_RDSR;
        cmd->AddressMode = HAL_XSPI_ADDRESS_8_LINES;
        cmd->Address     = 0;
        cmd->DataMode    = HAL_XSPI_DATA_8_LINES;
        cmd->DataLength  = MX66_OCMD_RDSR_LENGTH;
        cmd->DummyCycles = MX66_OCMD_REG_DUMMY;
        cmd->DQSMode     = HAL_XSPI_DQS_ENABLE;
        return;
    }

    mx66_prepare_cmd_1s(cmd);
    cmd->Instruction = MX66_CMD_RDSR;
    cmd->DataMode    = HAL_XSPI_DATA_1_LINE;
    cmd->DataLength  = 1;
}

MMFLASH_CODE static bool mx66_wait_ready(uint32_t timeoutMs)
{
    XSPI_RegularCmdTypeDef cmd = {0};
    uint8_t status[MX66_OCMD_RDSR_LENGTH];
    uint32_t tickstart = HAL_GetTick();

    mx66_prepare_readStatus(&cmd);

    do {
        if (HAL_XSPI_Command(&hxspi_mx66, &cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
            return false;
        }
        if (HAL_XSPI_Receive(&hxspi_mx66, status, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
            return false;
        }
        if ((status[0] & MX66_SR_WIP) == 0) {
            return true;
        }
    } while ((HAL_GetTick() - tickstart) < timeoutMs);

    return false;
}

MMFLASH_CODE static uint8_t mx66_readStatus(void)
{
    XSPI_RegularCmdTypeDef cmd = {0};
    uint8_t status[MX66_OCMD_RDSR_LENGTH] = { 0xFFU, 0xFFU };

    mx66_prepare_readStatus(&cmd);

    if (HAL_XSPI_Command(&hxspi_mx66, &cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return 0xFFU;
    }
    if (HAL_XSPI_Receive(&hxspi_mx66, status, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return 0xFFU;
    }
    return status[0];
}

MMFLASH_CODE static bool mx66_write_enable(void)
{
    XSPI_RegularCmdTypeDef cmd = {0};

    // WREN is the one octal command with neither an address phase nor a
    // data phase, so it does not follow the shape of its neighbours.
    if (mx66_opi) {
        mx66_prepare_cmd_8d(&cmd);
        cmd.Instruction = MX66_OCMD_WREN;
    } else {
        mx66_prepare_cmd_1s(&cmd);
        cmd.Instruction = MX66_CMD_WREN;
    }

    if (HAL_XSPI_Command(&hxspi_mx66, &cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return false;
    }
    // Verify the chip actually latched WEL before treating WREN as a
    // success. Per the MX66 datasheet, PP_4B / SE_4B are ignored when
    // WEL=0, and the subsequent waitForReady poll sees WIP=0
    // immediately — silently turning a dropped WREN into a no-op
    // write. mx66_readStatus returns 0xFF on its own HAL failures, so
    // treat that as a fault too.
    //
    // In OPI this readback is also the first thing that proves the
    // register latency is right: a wrong MX66_OCMD_REG_DUMMY shifts the
    // byte and WEL fails to show, which is a loud failure here instead
    // of a quiet corruption later.
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
    if (mx66_opi) {
        mx66_prepare_cmd_8d(&cmd);
        cmd.Instruction  = MX66_OCMD_SE_4B;
        cmd.AddressMode  = HAL_XSPI_ADDRESS_8_LINES;
    } else {
        mx66_prepare_cmd_1s(&cmd);
        cmd.Instruction  = MX66_CMD_SE_4B;
        cmd.AddressMode  = HAL_XSPI_ADDRESS_1_LINE;
    }
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

// One page-program command, issued as given. Caller owns the page
// boundary and, in OPI, the address/length parity.
MMFLASH_CODE static bool mx66_page_program_raw(uint32_t address, const uint8_t *data, uint32_t length)
{
    if (!mx66_wait_ready(MX66_TIMEOUT_PROGRAM_MS)) {
        return false;
    }
    if (!mx66_write_enable()) {
        return false;
    }

    XSPI_RegularCmdTypeDef cmd = {0};
    if (mx66_opi) {
        mx66_prepare_cmd_8d(&cmd);
        cmd.Instruction = MX66_OCMD_PP_4B;
        cmd.AddressMode = HAL_XSPI_ADDRESS_8_LINES;
        cmd.DataMode    = HAL_XSPI_DATA_8_LINES;
    } else {
        mx66_prepare_cmd_1s(&cmd);
        cmd.Instruction = MX66_CMD_PP_4B;
        cmd.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
        cmd.DataMode    = HAL_XSPI_DATA_1_LINE;
    }
    cmd.AddressWidth = HAL_XSPI_ADDRESS_32_BITS;
    cmd.Address      = address;
    cmd.DataLength   = length;

    if (HAL_XSPI_Command(&hxspi_mx66, &cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return false;
    }
    return HAL_XSPI_Transmit(&hxspi_mx66, (uint8_t *)data, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
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

    if (!mx66_opi) {
        return mx66_page_program_raw(address, data, length);
    }

    // DTR moves two bytes per clock, so the controller only accepts an
    // even address and an even length. Peel an odd head and an odd tail
    // off into two-byte programs padded with 0xFF: programming 0xFF
    // clears no bits, so the byte sharing the pair is left untouched.
    //
    // Config saves never reach this — they arrive as whole 256-byte
    // pages — but flashfs and the CLI do not promise that.
    uint8_t pair[2];

    if ((address & 1) != 0) {
        // Page offset is odd here, so address - 1 is still inside the page.
        pair[0] = 0xFFU;
        pair[1] = data[0];
        if (!mx66_page_program_raw(address - 1, pair, sizeof(pair))) {
            return false;
        }
        address++;
        data++;
        length--;
    }

    const uint32_t bulk = length & ~1U;
    if (bulk != 0 && !mx66_page_program_raw(address, data, bulk)) {
        return false;
    }

    if ((length & 1) != 0) {
        // The 0xFF lands on the following byte, or wraps to the start
        // of the same page if this was its last one. Either way it is
        // a no-op.
        pair[0] = data[bulk];
        pair[1] = 0xFFU;
        return mx66_page_program_raw(address + bulk, pair, sizeof(pair));
    }

    return true;
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

// One read command, issued as given. Caller owns the OPI address/length
// parity.
MMFLASH_CODE static bool mx66_read_raw(uint32_t address, uint8_t *buffer, uint32_t length)
{
    XSPI_RegularCmdTypeDef cmd = {0};
    if (mx66_opi) {
        mx66_prepare_cmd_8d(&cmd);
        // Opcode and latency come from the first stage rather than from
        // a constant here: they are whatever the memory-mapped window
        // has been reading with successfully all along.
        cmd.Instruction = mx66_read_instruction;
        cmd.AddressMode = HAL_XSPI_ADDRESS_8_LINES;
        cmd.DataMode    = HAL_XSPI_DATA_8_LINES;
        cmd.DummyCycles = mx66_read_dummy;
        cmd.DQSMode     = HAL_XSPI_DQS_ENABLE;
    } else {
        mx66_prepare_cmd_1s(&cmd);
        cmd.Instruction = MX66_CMD_READ_4B;
        cmd.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
        cmd.DataMode    = HAL_XSPI_DATA_1_LINE;
        cmd.DummyCycles = MX66_CMD_READ_4B_DUMMY;
    }
    cmd.AddressWidth = HAL_XSPI_ADDRESS_32_BITS;
    cmd.Address      = address;
    cmd.DataLength   = length;

    if (HAL_XSPI_Command(&hxspi_mx66, &cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return false;
    }
    return HAL_XSPI_Receive(&hxspi_mx66, buffer, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
}

MMFLASH_CODE static int mx66uw1g45g_readBytes(flashDevice_t *fdevice, uint32_t address, uint8_t *buffer, uint32_t length)
{
    if (length == 0 || address >= fdevice->geometry.totalSize) {
        return 0;
    }
    const uint32_t remaining = fdevice->geometry.totalSize - address;
    if (length > remaining) {
        length = remaining;
    }

    if (!mx66_opi) {
        return mx66_read_raw(address, buffer, length) ? (int)length : 0;
    }

    // Same even address / even length rule as the program path, handled
    // by reading the straddling pair and keeping the half we want.
    const uint32_t requested = length;
    uint8_t pair[2];

    if ((address & 1) != 0) {
        if (!mx66_read_raw(address - 1, pair, sizeof(pair))) {
            return 0;
        }
        buffer[0] = pair[1];
        address++;
        buffer++;
        length--;
    }

    const uint32_t bulk = length & ~1U;
    if (bulk != 0 && !mx66_read_raw(address, buffer, bulk)) {
        return 0;
    }

    if ((length & 1) != 0) {
        if (!mx66_read_raw(address + bulk, pair, sizeof(pair))) {
            return 0;
        }
        buffer[bulk] = pair[0];
    }

    return (int)requested;
}

MMFLASH_CODE_NOINLINE static const flashGeometry_t *mx66uw1g45g_getGeometry(flashDevice_t *fdevice)
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
