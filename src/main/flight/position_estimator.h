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

#include "common/axis.h"
#include "common/vector.h"
#include "common/time.h"

typedef enum {
    POSITION_SOURCE_NONE = 0,
    POSITION_SOURCE_GPS,
    POSITION_SOURCE_OPTICALFLOW
} positionSource_e;

typedef struct {
    // The position and targetPosition are expressed in earth frame (X=East, Y=North)
    // for GPS position, and in body frame for optical flow (X=Roll, Y=Pitch)
    // The autopilot must therefore rotate these values into body frame according
    // to yaw heading in GPS position hold mode, but not in optical flow mode.
    vector2_t position;      // Estimated position in cm
    vector2_t velocity;      // Estimated velocity in cm/s
    vector2_t targetPosition; // Target position in cm
    uint32_t lastUpdateUs;
    bool isValid;
    float trust;            // 0.0-1.0, based on quality
} positionEstimate_t;

#ifdef USE_OPTICALFLOW

void positionEstimatorInit(void);
void updateOpticalFlowPosition(void);
void resetOpticalFlowPosition(const vector2_t *currentPos);
positionEstimate_t* getOpticalFlowPosition(void);
positionSource_e getActivePositionSource(void);
void setActivePositionSource(positionSource_e source);
bool isOpticalFlowPositionValid(void);
void setOpticalFlowTarget(const vector2_t *target);
void updateOpticalFlowTargetByAxis(axis_e axis, float value);

#endif // USE_OPTICALFLOW
