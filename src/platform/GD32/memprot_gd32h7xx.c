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

#include "platform.h"

#include "drivers/memprot.h"

// Defined in linker script
extern uint8_t dmaram_start;
extern uint8_t dmaram_end;

extern uint8_t dmarwaxi_start;
extern uint8_t dmarwaxi_end;

mpuRegion_t mpuRegions[] = {
#ifdef USE_ITCM_RAM
    {
        //  Mark ITCM-RAM as read-only
        // "For Cortex®-M7, TCMs memories always behave as Non-cacheable, Non-shared normal memories, irrespective of the memory type attributes defined in the MPU for a memory region containing addresses held in the TCM"
        // See AN4838
        .start      = 0x00000000,
        .end        = 0, // Size defined by "size"
        .size       = MPU_REGION_SIZE_64KB,
        .perm       = MPU_AP_PRIV_UNPRIV_RO,
        .exec       = MPU_INSTRUCTION_EXEC_PERMIT,
        .shareable  = MPU_ACCESS_NON_SHAREABLE,
        .cacheable  = MPU_ACCESS_NON_CACHEABLE,
        .bufferable = MPU_ACCESS_BUFFERABLE,
    },
#endif
#ifdef USE_DMA_RAM
    {
        // DMA transmit buffer in D2 SRAM1
        // Reading needs cache coherence operation
        .start      = (uint32_t)&dmaram_start,
        .end        = (uint32_t)&dmaram_end,
        .size       = 0,  // Size determined by ".end"
        .perm       = MPU_AP_FULL_ACCESS,
        .exec       = MPU_INSTRUCTION_EXEC_PERMIT,
        .shareable  = MPU_ACCESS_SHAREABLE,
        .cacheable  = MPU_ACCESS_CACHEABLE,
        .bufferable = MPU_ACCESS_NON_BUFFERABLE,
    },
    {
        // A region in AXI RAM accessible from SDIO internal DMA
        .start      = (uint32_t)&dmarwaxi_start,
        .end        = (uint32_t)&dmarwaxi_end,
        .size       = 0,  // Size determined by ".end"
        .perm       = MPU_AP_FULL_ACCESS,
        .exec       = MPU_INSTRUCTION_EXEC_PERMIT,
        .shareable  = MPU_ACCESS_NON_SHAREABLE,
        .cacheable  = MPU_ACCESS_CACHEABLE,
        .bufferable = MPU_ACCESS_NON_BUFFERABLE,
    },
#endif
};

unsigned mpuRegionCount = ARRAYLEN(mpuRegions);

STATIC_ASSERT(ARRAYLEN(mpuRegions) <= MAX_MPU_REGIONS, MPU_region_count_exceeds_limit);
