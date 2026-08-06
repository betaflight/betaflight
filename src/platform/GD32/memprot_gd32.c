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

#include <string.h>

#include "platform.h"
#include "drivers/memprot.h"

static void memProtConfigError(void)
{
    for (;;) {}
}

void memProtConfigure(mpuRegion_t *regions, unsigned regionCount)
{
#if defined(GD32H7) && (__MPU_PRESENT == 1)
    mpu_region_init_struct mpu_init_struct;

    if (regionCount > MAX_MPU_REGIONS) {
        memProtConfigError();
    }

    /* disable the MPU */
    ARM_MPU_Disable();
    ARM_MPU_SetRegion(0, 0);

    /* initialize MPU region structure with default values */
    mpu_region_struct_para_init(&mpu_init_struct);
    mpu_init_struct.subregion_disable = 0x00;
    mpu_init_struct.tex_type = MPU_TEX_TYPE0;

    for (unsigned number = 0; number < regionCount; number++) {
        mpuRegion_t *region = &regions[number];

        if (region->end == 0 && region->size == 0) {
            memProtConfigError();
        }

        mpu_init_struct.region_number = number;
        mpu_init_struct.region_base_address = region->start;

        if (region->size) {
            mpu_init_struct.region_size = region->size;
        } else {
            // Adjust start of the region to align with cache line size.
            uint32_t start = region->start & ~0x1F;
            uint32_t length = region->end - start;

            if (length < 32) {
                // This will also prevent __builtin_clzl from returning negative (case length == 0)
                length = 32;
            }

            // Find most significant bit position using GCC builtin
            int msbpos = 32 - __builtin_clzl(length) - 1;

            if (length > (1U << msbpos)) {
                msbpos += 1;
            }

            // RASR SIZE field encoding: region_bytes = 2^(SIZE+1), so SIZE = msbpos - 1.
            // msbpos is the exponent such that 2^msbpos >= length (ceil of log2).
            // Writing msbpos directly would configure a region twice as large as needed.
            mpu_init_struct.region_size = msbpos - 1;
        }

        // Copy per region attributes
        mpu_init_struct.access_permission = region->perm;
        mpu_init_struct.instruction_exec = region->exec;
        mpu_init_struct.access_shareable = region->shareable;
        mpu_init_struct.access_cacheable = region->cacheable;
        mpu_init_struct.access_bufferable = region->bufferable;

        /* configure the MPU region */
        mpu_region_config(&mpu_init_struct);
        mpu_region_enable();
    }

    /* enable the MPU with privileged default access */
    ARM_MPU_Enable(MPU_MODE_PRIV_DEFAULT);
#else
    UNUSED(regions);
    UNUSED(regionCount);
#endif
}

void memProtReset(void)
{
#if defined(GD32H7) && (__MPU_PRESENT == 1)
    mpu_region_init_struct mpu_init_struct;

    /* disable the MPU */
    ARM_MPU_Disable();
    ARM_MPU_SetRegion(0, 0);

    /* initialize MPU region structure with default values */
    mpu_region_struct_para_init(&mpu_init_struct);

    // Disable existing regions by configuring them with no access
    for (uint8_t region = 0; region < MAX_MPU_REGIONS; region++) {
        mpu_init_struct.region_number = region;
        mpu_init_struct.region_base_address = 0x0;
        mpu_init_struct.region_size = MPU_REGION_SIZE_32B;
        mpu_init_struct.access_permission = MPU_AP_NO_ACCESS;
        mpu_init_struct.instruction_exec = MPU_INSTRUCTION_EXEC_NOT_PERMIT;
        mpu_init_struct.access_shareable = MPU_ACCESS_NON_SHAREABLE;
        mpu_init_struct.access_cacheable = MPU_ACCESS_NON_CACHEABLE;
        mpu_init_struct.access_bufferable = MPU_ACCESS_NON_BUFFERABLE;
        mpu_init_struct.subregion_disable = 0x00;
        mpu_init_struct.tex_type = MPU_TEX_TYPE0;

        /* configure the region (this effectively disables it) */
        mpu_region_config(&mpu_init_struct);
        // Note: Don't enable the region to keep it disabled
    }

    /* enable the MPU with privileged default access */
    ARM_MPU_Enable(MPU_MODE_PRIV_DEFAULT);
#endif
}
