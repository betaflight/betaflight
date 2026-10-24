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
 * MPU regions and cache setup for the i.MXRT 11xx, run before the caches are enabled.
 * Later regions win. Region 0 maps everything Strongly Ordered and no execute, so the core
 * never speculates into a clock gated block.
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

// TEX 0 with the C and B bits selects the standard device, write through and
// write back encodings; TEX 1 selects the separate outer and inner attributes.
#define MPU_TEX_STANDARD    0
#define MPU_TEX_OUTER_INNER 1

// FlexSPI1 at 0x30000000 and FlexSPI2 at 0x60000000; the size must match the linker script.
#if IMXRT_FLEXSPI_NOR_INSTANCE == 0
#define IMXRT_XIP_BASE      FlexSPI1_AMBA_BASE
#else
#define IMXRT_XIP_BASE      FlexSPI2_AMBA_BASE
#endif

#define IMXRT_XIP_SIZE      (TARGET_FLASH_SIZE * 1024u)

// The RASR size field is log2(size in bytes) - 1, which is all that
// ARM_MPU_REGION_SIZE_4MB and friends are.
#define IMXRT_XIP_SIZE_CODE (__builtin_ctz(IMXRT_XIP_SIZE) - 1u)

STATIC_ASSERT((IMXRT_XIP_SIZE & (IMXRT_XIP_SIZE - 1u)) == 0,
              flash_size_must_be_a_power_of_two);
STATIC_ASSERT((IMXRT_XIP_BASE & (IMXRT_XIP_SIZE - 1u)) == 0,
              flash_window_must_be_aligned_on_its_size);

// OCRAM1 and OCRAM2 at 0x20240000 are not 1 MB aligned, so the region is the 2 MB block
// from 0x20200000 with only their four 256 KB subregions enabled.
#define IMXRT_OCRAM_REGION_BASE 0x20200000u
#define IMXRT_OCRAM_SUBREGIONS  0xe1u   // subregions 1 to 4 on, 0 and 5 to 7 off

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
    // Peripherals, all four AIPS slices, as Device memory.
    {
        0x40000000u,
        ARM_MPU_RASR(MPU_EXEC_DENY, ARM_MPU_AP_FULL, MPU_TEX_STANDARD,
                     MPU_SHARED, MPU_NOT_CACHED, MPU_BUFFERED,
                     MPU_NO_SUBREGIONS, ARM_MPU_REGION_SIZE_32MB)
    },
    // Boot ROM, executable for the FlexSPI NOR driver that saves the configuration.
    {
        0x00200000u,
        ARM_MPU_RASR(MPU_EXEC_ALLOW, ARM_MPU_AP_RO, MPU_TEX_STANDARD,
                     MPU_NOT_SHARED, MPU_CACHED, MPU_NOT_BUFFERED,
                     MPU_NO_SUBREGIONS, ARM_MPU_REGION_SIZE_256KB)
    },
    // The boot flash. Read only: a store here would reach the FlexSPI write
    // path, and flash is written through the boot ROM API instead.
    {
        IMXRT_XIP_BASE,
        ARM_MPU_RASR(MPU_EXEC_ALLOW, ARM_MPU_AP_RO, MPU_TEX_STANDARD,
                     MPU_NOT_SHARED, MPU_CACHED, MPU_NOT_BUFFERED,
                     MPU_NO_SUBREGIONS, IMXRT_XIP_SIZE_CODE)
    },
    // OCRAM1 and OCRAM2, the two dedicated 512 KB blocks.
    {
        IMXRT_OCRAM_REGION_BASE,
        ARM_MPU_RASR(MPU_EXEC_DENY, ARM_MPU_AP_FULL, MPU_TEX_OUTER_INNER,
                     MPU_NOT_SHARED, MPU_CACHED, MPU_BUFFERED,
                     IMXRT_OCRAM_SUBREGIONS, ARM_MPU_REGION_SIZE_2MB)
    },
    // ITCM and DTCM as TEX 1 normal non cacheable; TEX 0 would be Strongly Ordered and fault
    // unaligned accesses. ITCM is read only once the reset handler has copied the code in.
    {
        0x00000000u,
        ARM_MPU_RASR(MPU_EXEC_ALLOW, ARM_MPU_AP_RO, MPU_TEX_OUTER_INNER,
                     MPU_NOT_SHARED, MPU_NOT_CACHED, MPU_NOT_BUFFERED,
                     MPU_NO_SUBREGIONS, ARM_MPU_REGION_SIZE_256KB)
    },
    // Every DMA buffer lives here.
    {
        0x20000000u,
        ARM_MPU_RASR(MPU_EXEC_DENY, ARM_MPU_AP_FULL, MPU_TEX_OUTER_INNER,
                     MPU_NOT_SHARED, MPU_NOT_CACHED, MPU_NOT_BUFFERED,
                     MPU_NO_SUBREGIONS, ARM_MPU_REGION_SIZE_256KB)
    },
};

STATIC_ASSERT(ARRAYLEN(mpuRegions) <= 16, "the Cortex-M7 MPU has only 16 regions");

void nxpMpuInit(void)
{
    // Attributes may only change with the caches off and the MPU disabled.
    SCB_DisableDCache();
    SCB_DisableICache();
    ARM_MPU_Disable();

    for (unsigned region = 0; region < ARRAYLEN(mpuRegions); region++) {
        ARM_MPU_SetRegionEx(region,
                            ARM_MPU_RBAR(region, mpuRegions[region].baseAddress),
                            mpuRegions[region].attributes);
    }

    // No privileged default map: region 0 already covers the whole address
    // space, so an access outside every region above is a fault.
    ARM_MPU_Enable(0);

    SCB_EnableICache();
    SCB_EnableDCache();
}
