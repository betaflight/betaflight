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

#include "platform.h"

#ifndef USE_WING

#include "math.h"

#ifdef USE_ALTITUDE_HOLD

#include "build/debug.h"
#include "common/maths.h"
#include "config/config.h"

#include "fc/rc.h"
#include "fc/runtime_config.h"

#include "flight/autopilot.h"
#include "flight/failsafe.h"
#include "flight/position.h"

#include "rx/rx.h"
#include "pg/autopilot.h"

#include "alt_hold.h"

static const float taskIntervalSeconds = HZ_TO_INTERVAL(ALTHOLD_TASK_RATE_HZ); // i.e. 0.01 s

typedef struct {
    bool isActive;
    float targetAltitudeCm;
    float maxVelocity;
    float targetVelocity;
    float deadband;
    bool allowStickAdjustment;
} altHoldState_t;

altHoldState_t altHold;

void altHoldReset(void)
{
    resetAltitudeControl();
    altHold.targetAltitudeCm = getAltitudeCm();
    altHold.targetVelocity = 0.0f;
}

void altHoldInit(void)
{
    altHold.isActive = false;
    altHold.deadband = altHoldConfig()->alt_hold_deadband / 100.0f;
    altHold.allowStickAdjustment = altHoldConfig()->alt_hold_deadband;
    altHold.maxVelocity = altHoldConfig()->alt_hold_climb_rate * 10.0f; // 50 in CLI means 500cm/s
    altHoldReset();
}

void altHoldProcessTransitions(void) {

    if (FLIGHT_MODE(ALT_HOLD_MODE)) {
        if (!altHold.isActive) {
            altHoldReset();
            altHold.isActive = true;
        }
    } else {
        altHold.isActive = false;
    }

    // ** the transition out of alt hold (exiting altHold) may be rough.  Some notes... **
    // The original PR had a gradual transition from hold throttle to pilot control throttle
    // using !(altHoldRequested && altHold.isActive) to run an exit function
    // a cross-fade factor was sent to mixer.c based on time since the flight mode request was terminated
    // it was removed primarily to simplify this PR

    // hence in this PR's the user's throttle needs to be close to the hover throttle value on exiting altHold
    // its not so bad because the 'target adjustment' by throttle requires that
    // user throttle must be not more than half way out from hover for a stable hold
}

void altHoldUpdateTargetAltitude(void)
{
    // User can adjust the target altitude with throttle, but only when
    // - throttle is outside deadband, and
    // - throttle is not low (zero), and
    // - deadband is not configured to zero

    float stickFactor = 0.0f;

    if (altHold.allowStickAdjustment && calculateThrottleStatus() != THROTTLE_LOW) {
        const float rcThrottle = rcCommand[THROTTLE];
        const float lowThreshold = autopilotConfig()->ap_hover_throttle - altHold.deadband * (autopilotConfig()->ap_hover_throttle - PWM_RANGE_MIN);
        const float highThreshold = autopilotConfig()->ap_hover_throttle + altHold.deadband * (PWM_RANGE_MAX - autopilotConfig()->ap_hover_throttle);

        if (rcThrottle < lowThreshold) {
            stickFactor = scaleRangef(rcThrottle, PWM_RANGE_MIN, lowThreshold, -1.0f, 0.0f);
        } else if (rcThrottle > highThreshold) {
            stickFactor = scaleRangef(rcThrottle, highThreshold, PWM_RANGE_MAX, 0.0f, 1.0f);
        }
    }

    // if failsafe is active, and we get here, we are in failsafe landing mode, it controls throttle
    if (failsafeIsActive()) {
        // descend at up to 10 times faster when high
        // default landing timeout is now 60s; must to get the quad down within this limit
        // need a rapid descent when initiated high, and must slow down closer to ground
        // this code doubles descent rate at 20m, to max 10x (10m/s on defaults) at 200m
        // the deceleration may be a bit rocky if it starts very high up
        // constant (set) deceleration target in the last 2m
        stickFactor = -(0.9f + constrainf(getAltitudeCm() / 2000.0f, 0.1f, 9.0f));
    }
    altHold.targetVelocity = stickFactor * altHold.maxVelocity;

    // prevent stick input from moving target altitude too far away from current altitude
    // otherwise it can be difficult to bring target altitude close to current altitude in a reasonable time
    // using maxVelocity means the stick can bring altitude target to current within 1s
    // this constrains the P and I response to user target changes, but not D of F responses
    // Range is compared to distance that might be traveled in one second
    if (fabsf(getAltitudeCm() - altHold.targetAltitudeCm) < altHold.maxVelocity * 1.0f /* s */) {
        altHold.targetAltitudeCm += altHold.targetVelocity * taskIntervalSeconds;
    }
}

void altHoldUpdate(void)
{
    // check if the user has changed the target altitude using sticks
    if (altHoldConfig()->alt_hold_climb_rate) {
        altHoldUpdateTargetAltitude();
    }
    altitudeControl(altHold.targetAltitudeCm, taskIntervalSeconds, altHold.targetVelocity);
}

void updateAltHold(timeUs_t currentTimeUs) {
    UNUSED(currentTimeUs);

    // check for enabling Alt Hold, otherwise do as little as possible while inactive
    altHoldProcessTransitions();

    if (altHold.isActive) {
        altHoldUpdate();
    }
}

bool isAltHoldActive(void) {
    return altHold.isActive;
}
#endif

#endif // !USE_WING
