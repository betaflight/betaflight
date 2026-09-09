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

#include <stdbool.h>
#include <stdint.h>

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

extern mpuRegion_t mpuRegions[];
extern unsigned mpuRegionCount;

void memProtReset(void);
void memProtConfigure(mpuRegion_t *mpuRegions, unsigned regionCount);

// Narrow [start, end) to the longest span within it that the processor will keep out of
// the D cache, judged from the regions the MPU actually holds rather than from the table
// it was configured with. Returns an empty range if there is no such span. Available
// where memProtConfigure() programs a PMSAv7 MPU.
void memProtFindUncachedRange(uint32_t start, uint32_t end, uint32_t *uncachedStart, uint32_t *uncachedEnd);

// Work out which part of the DMA sections is genuinely uncached, for the bus drivers that
// skip cache maintenance there. Implemented by the platforms that need it.
void memProtResolveDmaRam(void);
