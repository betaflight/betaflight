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

#include "io/gps.h"

extern float autopilotAngle[RP_AXIS_COUNT]; // NOTE: ANGLES ARE IN CENTIDEGREES

void autopilotInit(void);
void resetAltitudeControl(void);
void setSticksActiveStatus(bool areSticksActive);
void resetPositionControl(gpsLocation_t *initialTargetLocation);
void moveTargetLocation(int32_t latStep, int32_t lonStep);
void posControlOnNewGpsData(void);
void posControlOutput(void);
bool positionControl(void);
void altitudeControl(float targetAltitudeCm, float taskIntervalS, float targetAltitudeStep);

bool isBelowLandingAltitude(void);
float getAutopilotThrottle(void);
bool isAutopilotActive(void);
