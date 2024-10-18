/*
 * This file is part of Betaflight and INAV
 *
 * Betaflight and INAV are free software. You can
 * redistribute this software and/or modify this software under
 * the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * Betaflight and INAV are distributed in the hope that
 * they will be useful, but WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "drivers/rangefinder/rangefinder.h"
// the following values are from the MTF-01P Lidar
// TODO: the parameters should be configrable to support other MSP Lidars
#define RANGEFINDER_VIRTUAL_MAX_RANGE_CM    1200
#define RANGEFINDER_VIRTUAL_TASK_PERIOD_MS  100

// this value is taken from INAV, I tried to use 15 (according the dataset of MTTF-01P) but no readings were received 
#define RANGEFINDER_VIRTUAL_DETECTION_CONE_DECIDEGREES  900 

typedef struct virtualRangefinderVTable_s {
    bool (*detect)(void);
    void (*init)(void);
    void (*update)(void);
    int32_t (*read)(void);
} virtualRangefinderVTable_t;

bool virtualRangefinderDetect(rangefinderDev_t * dev, const virtualRangefinderVTable_t * vtable);
