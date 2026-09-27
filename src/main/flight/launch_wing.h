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

#include "common/time.h"

#if defined(USE_WING) && defined(USE_LAUNCH_WING)

typedef enum {
    LAUNCH_WING_IDLE = 0,
    LAUNCH_WING_WAIT_THROTTLE,
    LAUNCH_WING_MOTOR_IDLE,
    LAUNCH_WING_WAIT_DETECTION,
    LAUNCH_WING_MOTOR_DELAY,
    LAUNCH_WING_SPINUP,
    LAUNCH_WING_IN_PROGRESS,
    LAUNCH_WING_FINISH,
    LAUNCH_WING_FLYING,
    LAUNCH_WING_ABORTED,
} launchWingState_e;

void launchWingInit(void);
void launchWingArm(void);
void launchWingDisarm(void);
void launchWingUpdate(timeUs_t currentTimeUs);
void launchWingSwitchOff(void);

bool launchWingLatched(void);
bool launchWingIsActive(void);
bool launchWingIsTerminal(void);
bool launchWingThrottleValid(void);
bool launchWingHoldsIterm(void);
float launchWingGetThrottle(void);
float launchWingHandoverFactor(void);
launchWingState_e launchWingGetState(void);

#else

static inline void launchWingArm(void) { }
static inline void launchWingDisarm(void) { }
static inline bool launchWingHoldsIterm(void) { return false; }
static inline float launchWingHandoverFactor(void) { return 0.0f; }

#endif // USE_WING && USE_LAUNCH_WING
