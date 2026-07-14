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
#ifdef USE_DMA_RAM
    {
        // DMA transmit buffer
        // Reading needs cache coherence operation
        .start      = (uint32_t)&dmaram_start,
        .end        = (uint32_t)&dmaram_end,
        .size       = 0,  // Size determined by ".end"
        .perm       = MPU_REGION_FULL_ACCESS,
        .exec       = MPU_INSTRUCTION_ACCESS_ENABLE,
        .shareable  = MPU_ACCESS_SHAREABLE,
        .cacheable  = MPU_ACCESS_CACHEABLE,
        .bufferable = MPU_ACCESS_NOT_BUFFERABLE,
    },
    {
        // A region in AXI RAM accessible from SDIO internal DMA
        .start      = (uint32_t)&dmarwaxi_start,
        .end        = (uint32_t)&dmarwaxi_end,
        .size       = 0,  // Size determined by ".end"
        .perm       = MPU_REGION_FULL_ACCESS,
        .exec       = MPU_INSTRUCTION_ACCESS_ENABLE,
        .shareable  = MPU_ACCESS_NOT_SHAREABLE,
        .cacheable  = MPU_ACCESS_CACHEABLE,
        .bufferable = MPU_ACCESS_NOT_BUFFERABLE,
    },
#endif
};

unsigned mpuRegionCount = ARRAYLEN(mpuRegions);

STATIC_ASSERT(ARRAYLEN(mpuRegions) <= MAX_MPU_REGIONS, MPU_region_count_exceeds_limit);
