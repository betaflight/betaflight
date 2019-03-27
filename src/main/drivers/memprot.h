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

#pragma once

typedef struct mpuRegion_s {
    uint32_t start;
    uint32_t end;        // Zero if determined by size member (MPU_REGION_SIZE_xxx)
    uint8_t  size;       // Zero if determined from linker symbols
    uint8_t  perm;
    uint8_t  exec;
    uint8_t  shareable;
    uint8_t  cacheable;
    uint8_t  bufferable;
} mpuRegion_t;

#if defined(STM32H7)
/*
 * Upper 8 regions are reserved for a boot loader in EXST environment,
 * and of 3 regions are actually used to write protect the firmware
 * image transferred from an external storage.
 *
 * There were arguments on the boot loader reserving something
 * not obvious from firmware point of view, but left as is for now.
 *
 * If we want to remove this invisible "feature", we would completely
 * reset the MPU regions and write protect the firmware image,
 * using (most likely) multiple regions (e.g., 448K = 256K + 128K + 64K).
 */
#ifdef EXST
#define MAX_MPU_REGIONS 8
#else
#define MAX_MPU_REGIONS 16
#endif
#else
#error Unknown MCU family/type
#endif

extern mpuRegion_t mpuRegions[];
extern unsigned mpuRegionCount;

void memProtReset(void);
void memProtConfigure(mpuRegion_t *mpuRegions, unsigned regionCount);
