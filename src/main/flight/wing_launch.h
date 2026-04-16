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

#ifdef USE_WING_LAUNCH

#include "common/time.h"

typedef enum {
    WING_LAUNCH_IDLE = 0,
    WING_LAUNCH_DETECTED,
    WING_LAUNCH_MOTOR_DELAY,
    WING_LAUNCH_MOTOR_RAMP,
    WING_LAUNCH_CLIMBING,
    WING_LAUNCH_TRANSITION,
    WING_LAUNCH_COMPLETE,
    WING_LAUNCH_ABORT,
    WING_LAUNCH_STATE_COUNT
} wingLaunchState_e;

struct pidProfile_s;

void wingLaunchInit(const struct pidProfile_s *pidProfile);
void wingLaunchUpdate(timeUs_t currentTimeUs);
void wingLaunchReset(void);

bool isWingLaunchActive(void);
bool isWingLaunchInProgress(void);

float wingLaunchGetThrottle(void);
float wingLaunchGetPitchAngle(void);
float wingLaunchGetTransitionFactor(void);
int32_t wingLaunchGetClimbTimeRemainingMs(void);
wingLaunchState_e wingLaunchGetState(void);
bool wingLaunchIsThrottleGatePassed(void);

#endif // USE_WING_LAUNCH
