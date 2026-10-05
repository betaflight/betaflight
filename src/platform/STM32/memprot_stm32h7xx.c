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
        .perm       = MPU_REGION_PRIV_RO_URO,
        .exec       = MPU_INSTRUCTION_ACCESS_ENABLE,
        .shareable  = MPU_ACCESS_NOT_SHAREABLE,
        .cacheable  = MPU_ACCESS_NOT_CACHEABLE,
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
        .perm       = MPU_REGION_FULL_ACCESS,
        .exec       = MPU_INSTRUCTION_ACCESS_ENABLE,
        .shareable  = MPU_ACCESS_SHAREABLE,
        .cacheable  = MPU_ACCESS_CACHEABLE,
        .bufferable = MPU_ACCESS_NOT_BUFFERABLE,
    },
    {
        // A region in AXI RAM accessible from SDIO internal DMA.
        //
        // Left cached, unlike dmaram above: its users do their own cache maintenance
        // so that they can read the buffers back at cached speed. DShot bitbang in
        // particular walks its input buffer repeatedly to decode telemetry.
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

/* The part of the dmaram sections that is genuinely uncached.
 *
 * bus_spi skips the cache maintenance around a DMA transfer when the buffer is in memory
 * the processor is keeping out of the D cache. Which memory that is cannot be read off
 * the region table: rounding each region up to a power-of-two window makes overlaps
 * routine, and in an overlap the highest-numbered region's attributes win over all of it,
 * so a neighbouring region can strip the shareable bit off dmaram without either section
 * moving.
 *
 * Reading it back from the programmed MPU settles it, and costs nothing in the hot path
 * beyond two loads. A configuration that does not deliver an uncached dmaram narrows the
 * range, or empties it, and the transfers that fall outside get the maintenance they need
 * - slower, rather than wrong.
 *
 * Zero until resolved, which reads as "cached" for every address.
 */
FAST_DATA_ZERO_INIT uint32_t dmaramUncachedBase;
FAST_DATA_ZERO_INIT uint32_t dmaramUncachedLength;

void memProtResolveDmaRam(void)
{
#ifdef USE_DMA_RAM
    uint32_t uncachedStart;
    uint32_t uncachedEnd;

    memProtFindUncachedRange((uint32_t)&dmaram_start, (uint32_t)&dmaram_end, &uncachedStart, &uncachedEnd);

    dmaramUncachedBase = uncachedStart;
    dmaramUncachedLength = uncachedEnd - uncachedStart;
#endif
}
