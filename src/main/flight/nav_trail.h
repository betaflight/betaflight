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

#pragma once

#include <stdint.h>

#include "common/time.h"

#ifdef USE_OSD_NAV_MAP

// Build-time cap for trail storage. 240 points * 4 bytes = 960 bytes of RAM.
#define NAV_TRAIL_CAPACITY 240

// Trail points are stored in whole metres East/North of home: at map scales a
// character cell is metres wide, so centimetre resolution buys nothing.
typedef struct navTrailPoint_s {
    int16_t eastM;               // metres East of home
    int16_t northM;              // metres North of home
} navTrailPoint_t;

// Called periodically from the main loop; rate-limits internally and appends
// a point once the craft has flown far enough from the last stored one.
void navTrailUpdate(timeUs_t currentTimeUs);
void navTrailReset(void);

// trail access, index 0 = oldest point
unsigned navTrailCount(void);
const navTrailPoint_t *navTrailPointAt(unsigned index);

#endif // USE_OSD_NAV_MAP
