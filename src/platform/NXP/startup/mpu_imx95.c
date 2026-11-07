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
 * Higher numbered regions win. Region 0 maps everything Strongly Ordered and no execute, so the core
 * never reads ahead into another core's block, where the access may fault.
 */

#include <stdint.h>

#include "platform.h"

#include "common/utils.h"

#include "mpu_armv7.h"

#define MPU_EXEC_ALLOW  0
#define MPU_EXEC_DENY   1

#define MPU_NOT_SHARED  0
#define MPU_SHARED      1

#define MPU_NOT_CACHED  0
#define MPU_CACHED      1

#define MPU_NOT_BUFFERED 0
#define MPU_BUFFERED     1

#define MPU_NO_SUBREGIONS 0

#define MPU_TEX_STANDARD    0
#define MPU_TEX_OUTER_INNER 1

typedef struct {
    uint32_t baseAddress;
    uint32_t attributes;        // a complete RASR word
} mpuRegion_t;

static const mpuRegion_t mpuRegions[] = {
    {
        0x00000000u,
        ARM_MPU_RASR(MPU_EXEC_DENY, ARM_MPU_AP_FULL, MPU_TEX_STANDARD,
                     MPU_NOT_SHARED, MPU_NOT_CACHED, MPU_NOT_BUFFERED,
                     MPU_NO_SUBREGIONS, ARM_MPU_REGION_SIZE_4GB)
    },
    // Peripherals including the SCMI MU shared memory, Device memory.
    {
        0x40000000u,
        ARM_MPU_RASR(MPU_EXEC_DENY, ARM_MPU_AP_FULL, MPU_TEX_STANDARD,
                     MPU_SHARED, MPU_NOT_CACHED, MPU_BUFFERED,
                     MPU_NO_SUBREGIONS, ARM_MPU_REGION_SIZE_512MB)
    },
    // The 64 MB NOR on FlexSPI1, mcuboot included. Read only, write through.
    {
        0x28000000u,
        ARM_MPU_RASR(MPU_EXEC_ALLOW, ARM_MPU_AP_RO, MPU_TEX_STANDARD,
                     MPU_NOT_SHARED, MPU_CACHED, MPU_NOT_BUFFERED,
                     MPU_NO_SUBREGIONS, ARM_MPU_REGION_SIZE_64MB)
    },
    // ITCM and DTCM, not cached; ITCM is filled before the MPU is enabled, so read only.
    {
        0x00000000u,
        ARM_MPU_RASR(MPU_EXEC_ALLOW, ARM_MPU_AP_RO, MPU_TEX_OUTER_INNER,
                     MPU_NOT_SHARED, MPU_NOT_CACHED, MPU_NOT_BUFFERED,
                     MPU_NO_SUBREGIONS, ARM_MPU_REGION_SIZE_256KB)
    },
    {
        0x20000000u,
        ARM_MPU_RASR(MPU_EXEC_DENY, ARM_MPU_AP_FULL, MPU_TEX_OUTER_INNER,
                     MPU_NOT_SHARED, MPU_NOT_CACHED, MPU_NOT_BUFFERED,
                     MPU_NO_SUBREGIONS, ARM_MPU_REGION_SIZE_256KB)
    },
    // OCRAM, write back with write allocate.
    {
        0x20480000u,
        ARM_MPU_RASR(MPU_EXEC_DENY, ARM_MPU_AP_FULL, MPU_TEX_OUTER_INNER,
                     MPU_NOT_SHARED, MPU_CACHED, MPU_BUFFERED,
                     MPU_NO_SUBREGIONS, ARM_MPU_REGION_SIZE_256KB)
    },
    // OCRAM_CODE in the bottom 8 KB of OCRAM, read only like ITCM.
    {
        0x20480000u,
        ARM_MPU_RASR(MPU_EXEC_ALLOW, ARM_MPU_AP_RO, MPU_TEX_OUTER_INNER,
                     MPU_NOT_SHARED, MPU_CACHED, MPU_BUFFERED,
                     MPU_NO_SUBREGIONS, ARM_MPU_REGION_SIZE_8KB)
    },
    // Persistent block in the top 8 KB of OCRAM, not cached so a reset cannot drop a write.
    {
        0x204be000u,
        ARM_MPU_RASR(MPU_EXEC_DENY, ARM_MPU_AP_FULL, MPU_TEX_OUTER_INNER,
                     MPU_NOT_SHARED, MPU_NOT_CACHED, MPU_NOT_BUFFERED,
                     MPU_NO_SUBREGIONS, ARM_MPU_REGION_SIZE_8KB)
    },
    // rpmsg DDR shared with Linux, not cached so neither core needs cache maintenance.
    {
        0x88000000u,
        ARM_MPU_RASR(MPU_EXEC_DENY, ARM_MPU_AP_FULL, MPU_TEX_OUTER_INNER,
                     MPU_SHARED, MPU_NOT_CACHED, MPU_NOT_BUFFERED,
                     MPU_NO_SUBREGIONS, ARM_MPU_REGION_SIZE_4MB)
    },
    // DDR reserved for this core, write back.
    {
        0x90000000u,
        ARM_MPU_RASR(MPU_EXEC_ALLOW, ARM_MPU_AP_FULL, MPU_TEX_OUTER_INNER,
                     MPU_NOT_SHARED, MPU_CACHED, MPU_BUFFERED,
                     MPU_NO_SUBREGIONS, ARM_MPU_REGION_SIZE_4MB)
    },
};

STATIC_ASSERT(ARRAYLEN(mpuRegions) <= 16, cortex_m7_mpu_has_16_regions);

void nxpMpuInit(void)
{
    // Called from the reset handler with the caches off; attributes change only with the MPU disabled.
    ARM_MPU_Disable();

    for (unsigned region = 0; region < ARRAYLEN(mpuRegions); region++) {
        ARM_MPU_SetRegionEx(region,
                            ARM_MPU_RBAR(region, mpuRegions[region].baseAddress),
                            mpuRegions[region].attributes);
    }

    // Clear regions a previous image set above this table.
    const unsigned implemented = (MPU->TYPE & MPU_TYPE_DREGION_Msk) >> MPU_TYPE_DREGION_Pos;
    for (unsigned region = ARRAYLEN(mpuRegions); region < implemented; region++) {
        ARM_MPU_ClrRegion(region);
    }

    ARM_MPU_Enable(0);

    // Both enables invalidate without a clean.
    SCB_EnableICache();
    SCB_EnableDCache();
}
