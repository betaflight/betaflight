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

// Memory attribute index for non-cacheable normal memory
#define MEMPROT_ATTR_IDX_NON_CACHEABLE  0

// Access permission encoding for mpuRegion_t.perm field.
// Stores the combined AP value: (RO << 1) | NP, matching the
// ARMv8-M MPU RBAR AP field layout used by ARM_MPU_AP_().
#define MEMPROT_PERM_RW_NP  ARM_MPU_AP_(ARM_MPU_AP_RW, ARM_MPU_AP_NP)  // 0x01: RW, non-privileged
#define MEMPROT_PERM_RW_PO  ARM_MPU_AP_(ARM_MPU_AP_RW, ARM_MPU_AP_PO)  // 0x00: RW, privileged only
#define MEMPROT_PERM_RO_NP  ARM_MPU_AP_(ARM_MPU_AP_RO, ARM_MPU_AP_NP)  // 0x03: RO, non-privileged
#define MEMPROT_PERM_RO_PO  ARM_MPU_AP_(ARM_MPU_AP_RO, ARM_MPU_AP_PO)  // 0x02: RO, privileged only

mpuRegion_t mpuRegions[] = {
#ifdef USE_DMA_RAM
    {
        // DMA RAM region -- normal memory, non-cacheable, shareable
        // STM32C5 (Cortex-M33) has no data cache, so this ensures
        // DMA coherency without requiring cache maintenance operations.
        .start      = (uint32_t)&dmaram_start,
        .end        = (uint32_t)&dmaram_end,
        .size       = 0,               // Size determined by ".end"
        .perm       = MEMPROT_PERM_RW_NP,
        .exec       = ARM_MPU_EX,      // Execute permitted
        .shareable  = ARM_MPU_SH_INNER,
        .cacheable  = 0,
        .bufferable = 0,
    },
#endif
};

unsigned mpuRegionCount = ARRAYLEN(mpuRegions);

STATIC_ASSERT(ARRAYLEN(mpuRegions) <= MAX_MPU_REGIONS, MPU_region_count_exceeds_limit);

// STM32C5 uses Cortex-M33 (ARMv8-M) MPU which has a different register
// model from the ARMv7-M MPU used by STM32F4/F7/H7/G4.
// This implementation uses the CMSIS ARMv8-M MPU API directly
// (armv8m_mpu.h) instead of the HAL MPU wrappers.

void memProtReset(void)
{
    ARM_MPU_Disable();

    for (uint32_t region = 0; region < MAX_MPU_REGIONS; region++) {
        ARM_MPU_ClrRegion(region);
    }

    ARM_MPU_Enable(MPU_CTRL_PRIVDEFENA_Msk);
}

void memProtConfigure(mpuRegion_t *regions, unsigned regionCount)
{
    if (regionCount > MAX_MPU_REGIONS) {
        for (;;) {}
    }

    ARM_MPU_Disable();

    // Configure memory attribute 0: non-cacheable normal memory
    ARM_MPU_SetMemAttr(MEMPROT_ATTR_IDX_NON_CACHEABLE,
        ARM_MPU_ATTR(ARM_MPU_ATTR_NON_CACHEABLE, ARM_MPU_ATTR_NON_CACHEABLE));

    for (unsigned i = 0; i < regionCount; i++) {
        mpuRegion_t *region = &regions[i];

        // ARMv8-M MPU requires 32-byte aligned base and limit addresses
        uint32_t baseAddr = region->start & ~0x1FU;
        uint32_t limitAddr;

        if (region->end != 0) {
            limitAddr = (region->end + 0x1FU) & ~0x1FU;
        } else {
            limitAddr = baseAddr + (1U << region->size);
        }

        // Limit address must be aligned to 32 bytes and is inclusive of last byte
        if (limitAddr > 0) {
            limitAddr -= 1;
        }

        // Decode perm field: bit 1 = RO, bit 0 = NP (non-privileged)
        uint32_t ro = (region->perm >> 1) & 1U;
        uint32_t np = region->perm & 1U;

        ARM_MPU_SetRegion(i,
            ARM_MPU_RBAR(baseAddr, region->shareable, ro, np, region->exec),
            ARM_MPU_RLAR(limitAddr, MEMPROT_ATTR_IDX_NON_CACHEABLE));
    }

    ARM_MPU_Enable(MPU_CTRL_PRIVDEFENA_Msk);
}
