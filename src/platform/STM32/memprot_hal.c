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

#include <string.h>

#include "platform.h"

#include "drivers/memprot.h"

static void memProtConfigError(void)
{
    for (;;) {}
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

            // MPU base address must be aligned to the region size.
            // Widen the region until the aligned base still covers the
            // full range from start to end.
            uint32_t regionSize = 1U << msbpos;
            uint32_t alignedStart = start & ~(regionSize - 1);

            while (alignedStart + regionSize < region->end) {
                msbpos++;
                regionSize = 1U << msbpos;
                alignedStart = start & ~(regionSize - 1);
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
