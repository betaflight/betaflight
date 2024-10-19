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
#include "math.h"

#ifdef USE_ALT_HOLD_MODE

#include "build/debug.h"
#include "common/maths.h"
#include "config/config.h"
#include "fc/rc.h"
#include "fc/runtime_config.h"
#include "flight/autopilot.h"
#include "flight/failsafe.h"
#include "flight/position.h"
#include "rx/rx.h"

#include "alt_hold.h"

static const float taskIntervalSeconds = HZ_TO_INTERVAL(ALTHOLD_TASK_RATE_HZ); // i.e. 0.01 s

altHoldState_t altHoldState;

void controlAltitude(void)
{
    // boost D by 'increasing apparent velocity' when vertical velocity exceeds 5 m/s ( D of 75 on defaults)
    // usually we don't see fast ascend/descend rates if the altitude hold starts under stable conditions
    // this is important primarily to arrest pre-existing fast drops or climbs at the start;
    float verticalVelocity = getAltitudeDerivative(); // cm/s altitude derivative is always available
    const float kinkPoint = 500.0f; // velocity at which D should start to increase
    const float kinkPointAdjustment = kinkPoint * 2.0f; // Precompute constant
    const float sign = (verticalVelocity > 0) ? 1.0f : -1.0f;
    if (fabsf(verticalVelocity) > kinkPoint) {
        verticalVelocity = verticalVelocity * 3.0f - sign * kinkPointAdjustment;
    }

    //run the function in autopilot.c that calculates the PIDs and drives the motors
    altitudeControl(altHoldState.targetAltitudeCm, taskIntervalSeconds, verticalVelocity, altHoldState.targetAltitudeAdjustRate);
}

void altHoldReset(void)
{
    resetAltitudeControl();
    altHoldState.targetAltitudeCm = getAltitudeCm();
    altHoldState.targetAltitudeAdjustRate = 0.0f;
}

void altHoldInit(void)
{
    altHoldState.isAltHoldActive = false;
    altHoldReset();
}

void altHoldProcessTransitions(void) {

    if (FLIGHT_MODE(ALT_HOLD_MODE)) {
        if (!altHoldState.isAltHoldActive) {
            altHoldReset();
            altHoldState.isAltHoldActive = true;
        }
    } else {
        altHoldState.isAltHoldActive = false;
    }

    // ** the transition out of alt hold (exiting altHold) may be rough.  Some notes... **
    // The original PR had a gradual transition from hold throttle to pilot control throttle
    // using !(altHoldRequested && altHoldState.isAltHoldActive) to run an exit function
    // a cross-fade factor was sent to mixer.c based on time since the flight mode request was terminated
    // it was removed primarily to simplify this PR

    // hence in this PR's the user's throttle needs to be close to the hover throttle value on exiting altHold
    // its not so bad because the 'target adjustment' by throttle requires that
    // user throttle must be not more than half way out from hover for a stable hold
}

void altHoldUpdateTargetAltitude(void)
{
    // The user can raise or lower the target altitude with throttle up;  there is a big deadband.
    // Max rate for climb/descend is 1m/s by default (up to 2.5 is allowed but overshoots a fair bit)
    // If set to zero, the throttle has no effect.

    // Some people may not like throttle being able to change the target altitude, because:
    // - with throttle adjustment, hitting the switch won't always hold altitude if throttle is bumped
    // - eg if the throttle is bumped low accidentally, quad will start descending.
    // On the plus side:
    // - the pilot has control nice control over altitude, and the deadband is wide
    // - Slow controlled descents are possible, eg for landing
    // - fine-tuning height is possible, eg if there is slow sensor drift
    // - to keep the craft stable, throttle must be in the deadband, making exits smoother

    const float rcThrottle = rcCommand[THROTTLE];

    const float lowThreshold = 0.5f * (autopilotConfig()->hover_throttle + PWM_RANGE_MIN); // halfway between hover and MIN, e.g. 1150 if hover is 1300
    const float highThreshold = 0.5f * (autopilotConfig()->hover_throttle + PWM_RANGE_MAX); // halfway between hover and MAX, e.g. 1650 if hover is 1300
    
    float throttleAdjustmentFactor = 0.0f;
    if (rcThrottle < lowThreshold) {
        throttleAdjustmentFactor = scaleRangef(rcThrottle, PWM_RANGE_MIN, lowThreshold, -1.0f, 0.0f);
    } else if (rcThrottle > highThreshold) {
        throttleAdjustmentFactor = scaleRangef(rcThrottle, highThreshold, PWM_RANGE_MAX, 0.0f, 1.0f);
    }
    // if failsafe is active, and we get here, we are in failsafe landing mode
    if (failsafeIsActive()) {
        // descend at up to 10 times faster when high
        // default landing time is now 60s; need to get the quad down in this time from reasonable height
        // need a rapid descent if high to avoid that timeout, and must slow down closer to ground
        // this code doubles descent rate at 20m, to max 10x (10m/s on defaults) at 200m
        // user should be able to descend within 60s from around 150m high without disarming, on defaults
        // the deceleration may be a bit rocky if it starts very high up
        // constant (set) deceleration target in the last 2m
        throttleAdjustmentFactor = -(0.9f + constrainf(getAltitudeCm() / 2000.0f, 0.1f, 9.0f));
    }
    altHoldState.targetAltitudeAdjustRate = throttleAdjustmentFactor * altholdConfig()->alt_hold_target_adjust_rate;

    // if taskRate is 100Hz, default adjustRate of 100 adds/subtracts 1m every second, or 1cm per task run, at full stick position
    altHoldState.targetAltitudeCm += altHoldState.targetAltitudeAdjustRate  * taskIntervalSeconds;
}

void altHoldUpdate(void)
{
    // check if the user has changed the target altitude using sticks
    if (altholdConfig()->alt_hold_target_adjust_rate) {
        altHoldUpdateTargetAltitude();
    }

    controlAltitude();
}

void updateAltHoldState(timeUs_t currentTimeUs) {
    UNUSED(currentTimeUs); 

    // check for enabling Alt Hold, otherwise do as little as possible while inactive
    altHoldProcessTransitions();

    if (altHoldState.isAltHoldActive) {
        altHoldUpdate();
    }
}

int16_t getAltHoldTargetAltitudeCm(void) {
    return lrintf(altHoldState.targetAltitudeCm);
}
bool isAltHoldActive(void) {
    return altHoldState.isAltHoldActive;
}
#endif
