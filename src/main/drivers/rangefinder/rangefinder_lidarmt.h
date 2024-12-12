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
#include "drivers/opticalflow/opticalflow.h"
#include "sensors/rangefinder.h"

#define RANGEFINDER_MT_DETECTION_CONE_DECIDEGREES  900

typedef struct {
    rangefinderType_e deviceType;
    uint8_t delayMs;
    uint16_t maxRangeCm;
} MTRangefinderConfig;

typedef struct {
    uint32_t timestampUs;
    int32_t distanceMm;
} mtRangefinderData_t;

bool mtRangefinderDetect(rangefinderDev_t * dev, rangefinderType_e mtRangefinderToUse);
void mtRangefinderReceiveNewData(const uint8_t * bufferPtr);
const MTRangefinderConfig* getMTRangefinderDeviceConf(rangefinderType_e mtRangefinderToUse);

bool mtOpticalflowDetect(opticalflowDev_t * dev, rangefinderType_e mtRangefinderToUse);
void mtOpticalflowReceiveNewData(const uint8_t * bufferPtr);
