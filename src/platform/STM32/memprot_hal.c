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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "common/utils.h"

#include "drivers/memprot.h"

static void memProtConfigError(void)
{
    for (;;) {}
}

/* Read back the region attributes the MPU will actually apply to an address.
 *
 * A region's declared attributes are not necessarily the ones it ends up with. PMSAv7
 * requires every region to be a natural power of two, so memProtConfigure() rounds each
 * one up until it covers the section it was given, which makes regions overlap routinely
 * even where the sections they describe do not touch. In an overlap the highest-numbered
 * region wins outright, taking its attributes with it.
 *
 * Returns false if no enabled region covers the address, leaving the background (default
 * memory map) attributes in force.
 */
static bool memProtGetRegionAttributes(uint32_t addr, uint32_t *rasr)
{
    bool covered = false;

    for (unsigned region = 0; region < MAX_MPU_REGIONS; region++) {
        MPU->RNR = region;
        const uint32_t regionRbar = MPU->RBAR;
        const uint32_t regionRasr = MPU->RASR;

        if (!(regionRasr & MPU_RASR_ENABLE_Msk)) {
            continue;
        }

        // RASR SIZE semantics: region bytes = 2^(SIZE + 1), based at a multiple of that size
        const uint32_t size = 1U << (((regionRasr & MPU_RASR_SIZE_Msk) >> MPU_RASR_SIZE_Pos) + 1);
        const uint32_t base = regionRbar & ~(size - 1);

        if ((addr < base) || (addr - base >= size)) {
            continue;
        }

        // Regions of 256 bytes or more are split into eighths, any of which can be
        // disabled; where one is, the region does not apply.
        if (size >= 256) {
            const uint32_t srd = (regionRasr & MPU_RASR_SRD_Msk) >> MPU_RASR_SRD_Pos;
            if (srd & (1U << ((addr - base) / (size / 8)))) {
                continue;
            }
        }

        // A later region would override this one, so record it and keep looking
        *rasr = regionRasr;
        covered = true;
    }

    return covered;
}

/* Whether the processor will keep an address out of the D cache.
 *
 * The Cortex-M7 has no cache coherency hardware, so it never allocates Normal memory
 * marked shareable into the L1 data cache. That is what keeps the DMA regions coherent
 * without explicit maintenance.
 */
static bool memProtAddrIsUncached(uint32_t addr)
{
    uint32_t rasr;

    if (!memProtGetRegionAttributes(addr, &rasr)) {
        // Background attributes, so assume the worst
        return false;
    }

    return (rasr & MPU_RASR_S_Msk) || !(rasr & MPU_RASR_C_Msk);
}

// Collect the addresses within (start, end) at which the winning region, and so the
// attributes in force, can change. Sub-region edges are not among them: memProtConfigure()
// never disables a sub-region.
static unsigned memProtCollectEdges(uint32_t start, uint32_t end, uint32_t *edges, unsigned maxEdges)
{
    unsigned count = 0;

    for (unsigned region = 0; region < MAX_MPU_REGIONS; region++) {
        MPU->RNR = region;
        const uint32_t regionRbar = MPU->RBAR;
        const uint32_t regionRasr = MPU->RASR;

        if (!(regionRasr & MPU_RASR_ENABLE_Msk)) {
            continue;
        }

        const uint32_t size = 1U << (((regionRasr & MPU_RASR_SIZE_Msk) >> MPU_RASR_SIZE_Pos) + 1);
        const uint32_t base = regionRbar & ~(size - 1);
        const uint32_t regionEdges[] = { base, base + size };

        for (unsigned i = 0; i < ARRAYLEN(regionEdges); i++) {
            const uint32_t edge = regionEdges[i];

            if ((edge <= start) || (edge >= end) || (count >= maxEdges)) {
                continue;
            }

            // Insertion sort, keeping the list ordered and free of duplicates
            unsigned pos = count;
            while ((pos > 0) && (edges[pos - 1] > edge)) {
                edges[pos] = edges[pos - 1];
                pos--;
            }
            if ((pos < count) && (edges[pos] == edge)) {
                // Already present, so undo the shift
                while (pos < count) {
                    edges[pos] = edges[pos + 1];
                    pos++;
                }
                continue;
            }
            edges[pos] = edge;
            count++;
        }
    }

    return count;
}

void memProtFindUncachedRange(uint32_t start, uint32_t end, uint32_t *uncachedStart, uint32_t *uncachedEnd)
{
    // Nothing found yet, expressed as an empty range
    *uncachedStart = start;
    *uncachedEnd = start;

    if (start >= end) {
        return;
    }

    uint32_t edges[2 * MAX_MPU_REGIONS];
    const unsigned edgeCount = memProtCollectEdges(start, end, edges, ARRAYLEN(edges));

    /* The edges cut [start, end) into spans of constant attributes, so one probe per span
     * settles it. Return the longest run of adjacent uncached spans: a single range keeps
     * the test cheap for callers, and a caller finding its buffer outside it is only
     * obliged to fall back to explicit cache maintenance.
     */
    uint32_t runStart = start;
    bool inRun = false;

    for (unsigned i = 0; i <= edgeCount; i++) {
        const uint32_t spanStart = (i == 0) ? start : edges[i - 1];
        const uint32_t spanEnd = (i == edgeCount) ? end : edges[i];

        if (memProtAddrIsUncached(spanStart)) {
            if (!inRun) {
                runStart = spanStart;
                inRun = true;
            }
            if (spanEnd - runStart > *uncachedEnd - *uncachedStart) {
                *uncachedStart = runStart;
                *uncachedEnd = spanEnd;
            }
        } else {
            inRun = false;
        }
    }
}

void memProtConfigure(mpuRegion_t *regions, unsigned regionCount)
{
    MPU_Region_InitTypeDef MPU_InitStruct;

    if (regionCount > MAX_MPU_REGIONS) {
        memProtConfigError();
    }

    HAL_MPU_Disable();

    // Setup common members
    MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;

    for (unsigned number = 0; number < regionCount; number++) {
        mpuRegion_t *region = &regions[number];

        if (region->end == 0 && region->size == 0) {
            memProtConfigError();
        }

        MPU_InitStruct.Number      = number;
        MPU_InitStruct.BaseAddress = region->start;

        if (region->size) {
            MPU_InitStruct.Size = region->size;
        } else {
            // Adjust start of the region to align with cache line size.
            uint32_t start = region->start & ~0x1F;
            uint32_t length = region->end - start;

            if (length < 32) {
                // This will also prevent flsl from returning negative (case length == 0)
                length = 32;
            }

            int msbpos = flsl(length) - 1;

            if (length != (1U << msbpos)) {
                msbpos += 1;
            }

            // PMSAv7 requires the region base to be aligned to the region size: the MPU
            // ignores base-address bits below the region size, so an unaligned base
            // silently snaps down and the tail of the region is left uncovered. E.g. a
            // 64KB region based at an unaligned dmaram_start protects only the enclosing
            // aligned 64KB window; DMA buffers linked past its end stay cacheable with no
            // cache maintenance (bus_spi skips it inside dmaram), corrupting SD writes.
            // Align the base down and grow the size until the region covers region->end.
            // RASR SIZE semantics: region bytes = 2^(Size + 1).
            uint32_t regionBytes = 1U << (msbpos + 1);
            uint32_t alignedStart = start & ~(regionBytes - 1);

            while (alignedStart + regionBytes < region->end) {
                msbpos += 1;
                regionBytes <<= 1;
                alignedStart = start & ~(regionBytes - 1);
            }

            MPU_InitStruct.BaseAddress = alignedStart;
            MPU_InitStruct.Size = msbpos;
        }

        // Copy per region attributes
        MPU_InitStruct.AccessPermission = region->perm;
        MPU_InitStruct.DisableExec      = region->exec;
        MPU_InitStruct.IsShareable      = region->shareable;
        MPU_InitStruct.IsCacheable      = region->cacheable;
        MPU_InitStruct.IsBufferable     = region->bufferable;

        HAL_MPU_ConfigRegion(&MPU_InitStruct);
    }

    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

void memProtReset(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct;

    /* Disable the MPU */
    HAL_MPU_Disable();

    // Disable existing regions

    for (uint8_t region = 0; region <= MAX_MPU_REGIONS; region++) {
        MPU_InitStruct.Enable = MPU_REGION_DISABLE;
        MPU_InitStruct.Number = region;
        HAL_MPU_ConfigRegion(&MPU_InitStruct);
    }

    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}
