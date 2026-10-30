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
 * Erase and program of the octal NOR through FlexSPI IP commands, with the controller in mcuboot's 8D-8D-8D mode.
 * Runs from OCRAM with interrupts masked and writes only the configuration region and its backup.
 * The MX25UM51345G octal DTR sequences sit in the top four LUT slots.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(NAVQ95_USE_FLEXSPI_NOR)

#include "flexspi_nor_imxrt.h"

extern struct linker_symbol __config_start;
extern struct linker_symbol __config_end;

// LUT opcodes and the octal pad count.
#define NOR_CMD_DDR         0x21u
#define NOR_RADDR_DDR       0x22u
#define NOR_WRITE_DDR       0x28u
#define NOR_READ_DDR        0x29u
#define NOR_DUMMY_DDR       0x2cu
#define NOR_8PAD            3u

// One octal LUT instruction; a LUT word holds two, the first in the low half.
#define NOR_I(op, arg)      (((op) << 10) | (NOR_8PAD << 8) | (arg))
#define NOR_LUT(i0, i1)     ((uint32_t)(i0) | ((uint32_t)(i1) << 16))

#define NOR_SEQ_WRITE_ENABLE    28u
#define NOR_SEQ_ERASE_SECTOR    29u
#define NOR_SEQ_PAGE_PROGRAM    30u
#define NOR_SEQ_READ_STATUS     31u

#define NOR_LUT_KEY             0x5AF05AF0u
#define NOR_STATUS_WIP          0x01u

// Bounds every wait on a controller status bit.
#define FLEXSPI_NOR_SPIN        200000u

// Covers the 512 serial root clocks a relocked DLL needs before a read, errata ERR011377.
#define FLEXSPI_NOR_DLL_SETTLE  4000u
#define FLEXSPI_NOR_DLL_LOCK    (FLEXSPI_STS2_AREFLOCK_MASK | FLEXSPI_STS2_ASLVLOCK_MASK)

// Status reads are counted against a 400 ms worst case erase at the current FlexSPI clock.
#define FLEXSPI_NOR_BUSY_POLLS  4000000u

#define NOR_IP_FLAGS (FLEXSPI_INTR_IPCMDDONE_MASK | FLEXSPI_INTR_IPCMDERR_MASK | FLEXSPI_INTR_IPCMDGE_MASK \
                      | FLEXSPI_INTR_IPTXWE_MASK | FLEXSPI_INTR_IPRXWA_MASK)
#define NOR_BUS_IDLE (FLEXSPI_STS0_ARBIDLE_MASK | FLEXSPI_STS0_SEQIDLE_MASK)

// The configuration region and the backup of the same size that follows it.
#define NOR_WRITE_START     ((uint32_t)(uintptr_t)&__config_start)
#define NOR_WRITE_END       (2u * (uint32_t)(uintptr_t)&__config_end - NOR_WRITE_START)

static bool norReady;

static OCRAM_CODE bool flexspiNorWaitIdle(void)
{
    for (uint32_t spin = FLEXSPI_NOR_SPIN; spin--;) {
        if ((FLEXSPI->STS0 & NOR_BUS_IDLE) == NOR_BUS_IDLE) {
            return true;
        }
    }
    return false;
}

// Software reset drops the AHB read buffers and restarts the DLL, which must lock again before a read.
static OCRAM_CODE bool flexspiNorReset(void)
{
    FLEXSPI->MCR0 |= FLEXSPI_MCR0_SWRESET_MASK;
    uint32_t spin = FLEXSPI_NOR_SPIN;
    while ((FLEXSPI->MCR0 & FLEXSPI_MCR0_SWRESET_MASK) && --spin) {
    }
    if (!spin) {
        return false;
    }

    if (!(FLEXSPI->DLLCR[0] & FLEXSPI_DLLCR_DLLEN_MASK)) {
        return true;
    }
    spin = FLEXSPI_NOR_SPIN;
    while ((FLEXSPI->STS2 & FLEXSPI_NOR_DLL_LOCK) != FLEXSPI_NOR_DLL_LOCK && --spin) {
    }
    for (volatile uint32_t settle = FLEXSPI_NOR_DLL_SETTLE; settle--;) {
    }
    return spin != 0;
}

// One IP command on port A1; data, when given, is fed to the TX FIFO 8 bytes at a time.
static OCRAM_CODE bool flexspiNorCommand(uint32_t seq, uint32_t offset, const uint32_t *data, uint32_t bytes)
{
    if (!flexspiNorWaitIdle()) {
        return false;
    }

    FLEXSPI->FLSHCR2[0] |= FLEXSPI_FLSHCR2_CLRINSTRPTR_MASK;
    FLEXSPI->INTR = NOR_IP_FLAGS;
    FLEXSPI->IPTXFCR = FLEXSPI_IPTXFCR_CLRIPTXF_MASK;
    FLEXSPI->IPRXFCR = FLEXSPI_IPRXFCR_CLRIPRXF_MASK;
    FLEXSPI->IPCR0 = offset;
    FLEXSPI->IPCR1 = FLEXSPI_IPCR1_ISEQID(seq) | FLEXSPI_IPCR1_ISEQNUM(0) | FLEXSPI_IPCR1_IDATSZ(bytes);
    FLEXSPI->IPCMD = FLEXSPI_IPCMD_TRG_MASK;

    for (uint32_t word = 0; data && word < bytes / 4u; word += 2) {
        uint32_t spin = FLEXSPI_NOR_SPIN;
        while (!(FLEXSPI->INTR & FLEXSPI_INTR_IPTXWE_MASK) && --spin) {
        }
        if (!spin) {
            return false;
        }
        FLEXSPI->TFDR[0] = data[word];
        FLEXSPI->TFDR[1] = data[word + 1];
        FLEXSPI->INTR = FLEXSPI_INTR_IPTXWE_MASK;
    }

    for (uint32_t spin = FLEXSPI_NOR_SPIN; spin--;) {
        const uint32_t interrupts = FLEXSPI->INTR;
        if (interrupts & (FLEXSPI_INTR_IPCMDERR_MASK | FLEXSPI_INTR_IPCMDGE_MASK)) {
            return false;
        }
        if (interrupts & FLEXSPI_INTR_IPCMDDONE_MASK) {
            return true;
        }
    }
    return false;
}

// Polls the status register until the part is done.
static OCRAM_CODE bool flexspiNorWaitReady(void)
{
    for (uint32_t poll = FLEXSPI_NOR_BUSY_POLLS; poll--;) {
        if (!flexspiNorCommand(NOR_SEQ_READ_STATUS, 0, NULL, 4)) {
            return false;
        }
        // The fill level counts 64 bit words.
        uint32_t spin = FLEXSPI_NOR_SPIN;
        while (!(FLEXSPI->IPRXFSTS & FLEXSPI_IPRXFSTS_FILL_MASK) && --spin) {
        }
        if (!spin) {
            return false;
        }
        const uint32_t status = FLEXSPI->RFDR[0];
        FLEXSPI->INTR = FLEXSPI_INTR_IPRXWA_MASK;
        if (!(status & NOR_STATUS_WIP)) {
            return true;
        }
    }
    return false;
}

// Nothing may read the written range until flexspiNorFlush().
static OCRAM_CODE bool flexspiNorWrite(uint32_t address, uint32_t seq, const uint32_t *data, uint32_t bytes)
{
    const uint32_t offset = address - FlexSPI_AMBA_BASE;

    // Masked for the whole erase or program: saves run disarmed and the NOR cannot be read meanwhile.
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    const bool ok = flexspiNorCommand(NOR_SEQ_WRITE_ENABLE, offset, NULL, 0)
        && flexspiNorCommand(seq, offset, data, bytes)
        && flexspiNorWaitReady();

    __set_PRIMASK(primask);

    return ok;
}

// The AHB read buffers have no clear bit, so a software reset drops them.
OCRAM_CODE bool flexspiNorFlush(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    const bool idle = flexspiNorWaitIdle();
    const bool ok = flexspiNorReset() && idle;

    SCB_InvalidateDCache_by_Addr((void *)(uintptr_t)NOR_WRITE_START, (int32_t)(NOR_WRITE_END - NOR_WRITE_START));
    __DSB();
    __ISB();

    __set_PRIMASK(primask);

    return ok;
}

OCRAM_CODE bool flexspiNorInit(void)
{
    // The part must already read in octal DTR, with the read sequences below the write slots.
    const uint32_t readCtrl = FLEXSPI->FLSHCR2[0];
    const uint32_t readSeq = (readCtrl & FLEXSPI_FLSHCR2_ARDSEQID_MASK) >> FLEXSPI_FLSHCR2_ARDSEQID_SHIFT;
    const uint32_t readLast = readSeq
                              + ((readCtrl & FLEXSPI_FLSHCR2_ARDSEQNUM_MASK) >> FLEXSPI_FLSHCR2_ARDSEQNUM_SHIFT);
    const uint32_t readFirst = FLEXSPI->LUT[readSeq * 4u];
    if (readLast >= NOR_SEQ_WRITE_ENABLE || (readFirst & 0xFF00u) != NOR_I(NOR_CMD_DDR, 0)) {
        norReady = false;
        return false;
    }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    const bool idle = flexspiNorWaitIdle();
    if (idle) {
        FLEXSPI->LUTKEY = NOR_LUT_KEY;
        FLEXSPI->LUTCR = FLEXSPI_LUTCR_UNLOCK_MASK;

        volatile uint32_t *lut = &FLEXSPI->LUT[NOR_SEQ_WRITE_ENABLE * 4u];
        lut[0] = NOR_LUT(NOR_I(NOR_CMD_DDR, 0x06u), NOR_I(NOR_CMD_DDR, 0xF9u));
        lut[1] = 0;
        lut[2] = 0;
        lut[3] = 0;
        lut[4] = NOR_LUT(NOR_I(NOR_CMD_DDR, 0x21u), NOR_I(NOR_CMD_DDR, 0xDEu));
        lut[5] = NOR_LUT(NOR_I(NOR_RADDR_DDR, 0x20u), 0);
        lut[6] = 0;
        lut[7] = 0;
        lut[8] = NOR_LUT(NOR_I(NOR_CMD_DDR, 0x12u), NOR_I(NOR_CMD_DDR, 0xEDu));
        lut[9] = NOR_LUT(NOR_I(NOR_RADDR_DDR, 0x20u), NOR_I(NOR_WRITE_DDR, 0x04u));
        lut[10] = 0;
        lut[11] = 0;
        lut[12] = NOR_LUT(NOR_I(NOR_CMD_DDR, 0x05u), NOR_I(NOR_CMD_DDR, 0xFAu));
        lut[13] = NOR_LUT(NOR_I(NOR_RADDR_DDR, 0x20u), NOR_I(NOR_DUMMY_DDR, 0x28u));
        lut[14] = NOR_LUT(NOR_I(NOR_READ_DDR, 0x04u), 0);
        lut[15] = 0;

        FLEXSPI->LUTKEY = NOR_LUT_KEY;
        FLEXSPI->LUTCR = FLEXSPI_LUTCR_LOCK_MASK;
    }

    __set_PRIMASK(primask);

    norReady = idle;
    return idle;
}

OCRAM_CODE bool flexspiNorEraseSector(uint32_t address)
{
    if (!norReady
        || !flexspiNorRangeAllowed(NOR_WRITE_START, NOR_WRITE_END, address,
                                   FLEXSPI_NOR_SECTOR_SIZE, FLEXSPI_NOR_SECTOR_SIZE)) {
        return false;
    }

    return flexspiNorWrite(address, NOR_SEQ_ERASE_SECTOR, NULL, 0);
}

OCRAM_CODE bool flexspiNorProgramPage(uint32_t address, const uint32_t *data)
{
    if (!norReady || !data || ((uintptr_t)data & 3u)
        || !flexspiNorRangeAllowed(NOR_WRITE_START, NOR_WRITE_END, address,
                                   FLEXSPI_NOR_PAGE_SIZE, FLEXSPI_NOR_PAGE_SIZE)) {
        return false;
    }

    return flexspiNorWrite(address, NOR_SEQ_PAGE_PROGRAM, data, FLEXSPI_NOR_PAGE_SIZE);
}

#endif /* NAVQ95_USE_FLEXSPI_NOR */
