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
#include "sensors/rangefinder.h"

#define RANGEFINDER_MT_DETECTION_CONE_DECIDEGREES  900 

bool mtRangefinderDetect(rangefinderDev_t * dev, uint8_t mtRangefinderToUse);
void mtRangefinderReceiveNewData(uint8_t * bufferPtr);
uint8_t getDeviceTableIdx(uint8_t mtRangefinderToUse);

typedef struct {
    uint8_t  deviceId;
    uint8_t  delayMs;
    uint16_t maxRangeCm;
} MTRangefinderConfig;

// Initialize the table with values for each rangefinder type
static const MTRangefinderConfig rangefinderConfigs[] = {
    [0] = { .deviceId = RANGEFINDER_MT01P, .delayMs = 20, .maxRangeCm = 1000 },
    [1] = { .deviceId = RANGEFINDER_MT01P, .delayMs = 10, .maxRangeCm = 800  },
    [2] = { .deviceId = RANGEFINDER_MT01P, .delayMs = 20, .maxRangeCm = 250  },
    [3] = { .deviceId = RANGEFINDER_MT01P, .delayMs = 10, .maxRangeCm = 1200 },
    [4] = { .deviceId = RANGEFINDER_MT01P, .delayMs = 20, .maxRangeCm = 600  },
};
