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

#ifndef USE_WING

#include "io/gps.h"

extern float autopilotAngle[RP_AXIS_COUNT]; // NOTE: ANGLES ARE IN CENTIDEGREES

void resetPositionControl(const gpsLocation_t *initialTargetLocation, unsigned taskRateHz);
void autopilotInit(void);

void resetAltitudeControl(void);
void altitudeControl(float targetAltitudeCm, float taskIntervalS, float targetAltitudeStep);

void setSticksActiveStatus(bool areSticksActive);
void moveTargetLocation(vector2_t lonLatSteps);
void posControlOnNewGpsData(void);
void posControlOutput(void);
bool positionControl(void);

bool isBelowLandingAltitude(void);
float getAutopilotThrottle(void);
bool isAutopilotInControl(void);

#endif // !USE_WING
