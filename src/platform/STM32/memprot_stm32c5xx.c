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

mpuRegion_t mpuRegions[] = {
#ifdef USE_DMA_RAM
    {
        // DMA RAM region -- normal memory, non-cacheable, shareable
        // STM32C5 (Cortex-M33) has no data cache, so this ensures
        // DMA coherency without requiring cache maintenance operations.
        .start      = (uint32_t)&dmaram_start,
        .end        = (uint32_t)&dmaram_end,
        .size       = 0,  // Size determined by ".end"
        .perm       = MPU_REGION_ALL_RW,
        .exec       = MPU_INSTRUCTION_ACCESS_ENABLE,
        .shareable  = MPU_ACCESS_INNER_SHAREABLE,
        .cacheable  = 0,
        .bufferable = 0,
    },
#endif
};

unsigned mpuRegionCount = ARRAYLEN(mpuRegions);

STATIC_ASSERT(ARRAYLEN(mpuRegions) <= MAX_MPU_REGIONS, MPU_region_count_exceeds_limit);

// STM32C5 uses Cortex-M33 (ARMv8-M) MPU which has a different register
// model from the ARMv7-M MPU used by STM32F4/F7/H7/G4.

void memProtReset(void)
{
    HAL_MPU_Disable();

    for (uint32_t region = 0; region < MAX_MPU_REGIONS; region++) {
        HAL_MPU_DisableRegion(region);
    }

    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

void memProtConfigure(mpuRegion_t *regions, unsigned regionCount)
{
    if (regionCount > MAX_MPU_REGIONS) {
        for (;;) {}
    }

    HAL_MPU_Disable();

    // Configure memory attribute for non-cacheable normal memory
    MPU_Attributes_InitTypeDef attrInit;
    attrInit.Number = MPU_ATTRIBUTES_NUMBER0;
    attrInit.Attributes = INNER_OUTER(MPU_NOT_CACHEABLE);
    HAL_MPU_ConfigMemoryAttributes(&attrInit);

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

        MPU_Region_InitTypeDef regionInit;
        regionInit.Enable = MPU_REGION_ENABLE;
        regionInit.Number = i;
        regionInit.BaseAddress = baseAddr;
        regionInit.LimitAddress = limitAddr;
        regionInit.AttributesIndex = MPU_ATTRIBUTES_NUMBER0;
        regionInit.AccessPermission = region->perm;
        regionInit.DisableExec = region->exec;
        regionInit.IsShareable = region->shareable;

        HAL_MPU_ConfigRegion(&regionInit);
    }

    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}
