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
 * FlexSPI octal-NOR erase/program interface, ported from Betaflight PR #15853
 * (PetervdPerk, GPLv3). The i.MXRT and i.MX95 FlexSPI IP is identical, so the
 * same header serves both; on the NAVQ95 (no-SDK) port the register map + masks
 * + FlexSPI_AMBA_BASE + OCRAM_CODE come from platform/flexspi_regs.h instead of
 * the MCUXpresso device header.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "platform/flexspi_regs.h"

// Page and sector size of the NOR parts these boards carry.
#define FLEXSPI_NOR_PAGE_SIZE       256u
#define FLEXSPI_NOR_SECTOR_SIZE     4096u

// True when [address, address + length) fits inside [regionStart, regionEnd)
// and address is a multiple of alignment, without wrapping.
static inline bool flexspiNorRangeAllowed(uint32_t regionStart, uint32_t regionEnd,
                                          uint32_t address, uint32_t length, uint32_t alignment)
{
    return length != 0
        && alignment != 0
        && (address % alignment) == 0
        && regionStart < regionEnd
        && address >= regionStart
        && address < regionEnd
        && length <= regionEnd - address;
}

// Middle of the widest run of set bits in a DLL delay pass mask (lowest run on
// a tie), or -1 when no bit is set.
static inline int flexspiNorDllCenter(uint16_t passMask)
{
    int bestStart = -1;
    int bestLength = 0;
    int run = 0;

    for (int code = 0; code < 16; code++) {
        if (passMask & (1u << code)) {
            run++;
            if (run > bestLength) {
                bestLength = run;
                bestStart = code - run + 1;
            }
        } else {
            run = 0;
        }
    }

    return bestStart < 0 ? -1 : bestStart + bestLength / 2;
}

// Prepare the ROM FlexSPI NOR driver. Safe to call more than once; false
// leaves the flash readable but not writable.
bool flexspiNorInit(void);

// Erase one 4 KB sector at a memory mapped address inside the configuration region.
bool flexspiNorEraseSector(uint32_t address);

// Program one 256 byte page at a page aligned address inside the configuration
// region, from 32 bit aligned data.
bool flexspiNorProgramPage(uint32_t address, const uint32_t *data);

// Make the configuration region and its backup readable again after erases and
// programs; nothing may read that range from the first write until this returns.
bool flexspiNorFlush(void);

// Non-destructive read of the NOR status register (RDSR) via the IP engine, to
// prove M7<->NOR command comms. Requires a prior successful flexspiNorInit().
bool flexspiNorReadStatus(uint32_t *status);
