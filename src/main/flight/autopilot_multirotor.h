/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/axis.h"
#include "common/vector.h"

#ifndef USE_WING

#include "io/gps.h"

extern float autopilotAngle[RP_AXIS_COUNT]; // NOTE: ANGLES ARE IN CENTIDEGREES

void autopilotInit(void);
void resetAltitudeControl(void);
void setSticksActiveStatus(bool areSticksActive);
void resetPositionControl(const gpsLocation_t *initialTargetLocation, unsigned taskRateHz);
void posControlSetMissionTarget(const gpsLocation_t *location, float sanityCheckDistanceCm);
void positionControlRelax(void); // decay autopilotAngle to wings-level while position control is not running
void posControlOutput(void);
bool positionControl(void);
void altitudeControl(float targetAltitudeCm, float taskIntervalS, float targetAltitudeStep);

bool isBelowLandingAltitude(void);
float getAutopilotThrottle(void);
bool isAutopilotInControl(void);
bool autopilotGetTargetPositionEfCm(vector2_t *targetEfCm); // read-only hold anchor for status displays

#endif // !USE_WING
