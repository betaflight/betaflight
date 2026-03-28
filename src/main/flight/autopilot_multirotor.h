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

#ifndef USE_WING

#define AP_HOVER_THROTTLE_DEFAULT 1275U

extern float autopilotAngle[RP_AXIS_COUNT]; // NOTE: ANGLES ARE IN CENTIDEGREES

void autopilotInit(void);
void resetAltitudeControl(void);
void setSticksActiveStatus(bool areSticksActive);
void resetPositionControl(unsigned taskRateHz);
void posControlOutput(void);
bool positionControl(void);
// targetAltitudeVelCmS: climb-rate feedforward (cm/s) — stick in alt hold, rescue ascent/descent rates.
// velLimitCmS: |vertical velocity command| cap (cm/s); use 0 for default (1500 cm/s).
void altitudeControl(float targetAltitudeCm, float taskIntervalS, float targetAltitudeVelCmS, float velLimitCmS);

uint16_t autopilotGetEffectiveHoverThrottlePwm(void);
void autopilotCaptureHoverThrottleForAltHold(void);
void autopilotClearAltHoldHoverThrottle(void);

uint16_t autopilotGetEffectiveHoverThrottlePwm(void);
void autopilotCaptureHoverThrottleForAltHold(void);
void autopilotClearAltHoldHoverThrottle(void);

bool isBelowLandingAltitude(void);
float getAutopilotThrottle(void);
bool isAutopilotInControl(void);

#endif // !USE_WING
