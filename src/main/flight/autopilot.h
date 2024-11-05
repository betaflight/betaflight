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

#include "pg/autopilot.h"
#include "flight/pid.h"
#include "io/gps.h"

extern float autopilotAngle[2]; // NOTE: ANGLES ARE IN CENTIDEGREES

void autopilotInit(const autopilotConfig_t *config);
void resetAltitudeControl(void);
void setSticksActiveStatus(bool areSticksActive);
void resetPositionControl(gpsLocation_t initialTargetLocation);
void setTargetLocation(gpsLocation_t newTargetLocation);

void altitudeControl(float targetAltitudeCm, float taskIntervalS, float verticalVelocity, float targetAltitudeStep);
bool positionControl();

bool isBelowLandingAltitude(void);
float getAutopilotThrottle(void);
bool isAutopilotActive(void);
