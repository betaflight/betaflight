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

#ifdef USE_ALT_HOLD_MODE

#include "math.h"
#include "build/debug.h"

#include "config/config.h"
#include "fc/runtime_config.h"
#include "fc/rc.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/position.h"
#include "flight/position_control.h"
#include "sensors/acceleration.h"
#include "rx/rx.h"

#include "alt_hold.h"

static float pidIntegral = 0.0f;
static const float taskIntervalSeconds = HZ_TO_INTERVAL(ALTHOLD_TASK_RATE_HZ); // i.e. 0.01 s

altHoldState_t altHoldState;

float altitudePidCalculate(void)
{
    // * introductory notes *
    // this is a classical PID controller with heuristic D boost and iTerm relax
    // the basic parameters provide good control when initiated in stable situations
    
    // tuning:
    // -reduce P I and D by 1/3 or until it doesn't oscillate but has sloppy / slow control
    // increase P until there is definite oscillation, then back off until barely noticeable
    // increase D until there is definite oscillation (will be faster than P), then back off until barely noticeable
    // try to add a little more P, then try to add a little more D, but not to oscillation
    // iTerm isn't very important if hover throttle is set carefully and sag compensation is used

    // The altitude D lowpass filter is very important.
    // The only way to get enough D is to filter the oscillations out.
    // More D filtering is needed with Baro than with GPS, since GPS is smoother and slower.

    // A major problem is the lag time for motors to arrest pre-existing drops or climbs,
    // compounded by the lag time from filtering.
    // If the quad is dropping fast, the motors have to be high for a long time to arrest the drop
    // this is very difficult for a 'simple' PID controller;
    // if the PIDs are high enough to arrest a fast drop, they will oscillate under normal conditions
    // Hence we:
    // - Enhance D when the absolute velocity is high, ie when we need to strongly oppose a fast drop,
    //   even though it may cause throttle oscillations while dropping quickly - the average D is what we need
    // - Prevent excessive iTerm growth when error is impossibly large for iTerm to resolve

    const pidCoefficient_t *pidCoefs =  getAltitudePidCoeffs();

    const float altitudeErrorCm = altHoldState.targetAltitudeCm - getAltitudeCm();

    // P
    const float pOut = pidCoefs->Kp * altitudeErrorCm;

    // I
    // input limit iTerm so that it doesn't grow fast with large errors
    // very important at the start if there are massive initial errors to prevent iTerm windup

    // much less iTerm change for errors greater than 2m, otherwise it winds up badly
    const float itermNormalRange = 200.0f; // 2m
    const float itermRelax = (fabsf(altitudeErrorCm) < itermNormalRange) ? 1.0f : 0.1f;
    pidIntegral += altitudeErrorCm * pidCoefs->Ki * itermRelax * taskIntervalSeconds;
    // arbitrary limit on iTerm, same as for gps_rescue, +/-20% of full throttle range
    // ** might not be needed with input limiting **
    pidIntegral = constrainf(pidIntegral, -200.0f, 200.0f); 
    const float iOut = pidIntegral;

    // D
    // boost D by 'increasing apparent velocity' when vertical velocity exceeds 5 m/s ( D of 75 on defaults)
    // usually we don't see fast ascend/descend rates if the altitude hold starts under stable conditions
    // this is important primarily to arrest pre-existing fast drops or climbs at the start;

    float vel = getAltitudeDerivative(); // cm/s altitude derivative is always available
    const float kinkPoint = 500.0f; // velocity at which D should start to increase
    const float kinkPointAdjustment = kinkPoint * 2.0f; // Precompute constant
    const float sign = (vel > 0) ? 1.0f : -1.0f;
    if (fabsf(vel) > kinkPoint) {
        vel = vel * 3.0f - sign * kinkPointAdjustment;
    }

    const float dOut = pidCoefs->Kd * vel;

    // F
    // if error is used, we get a 'free kick' in derivative from changes in the target value
    // but this is delayed by the smoothing, leading to lag and overshoot.
    // calculating feedforward separately avoids the filter lag.
    // Use user's D gain for the feedforward gain factor, works OK with a scaling factor of 0.01
    // A commanded drop at 100cm/s will return feedforward of the user's D value. or 15 on defaults
    const float fOut = pidCoefs->Kf * altHoldState.targetAltitudeAdjustRate;
    
    // to further improve performance...
    // adding a high-pass filtered amount of FF would give a boost when altitude changes were requested
    // this would offset motor lag and kick off some initial vertical acceleration.
    // this would be exactly what an experienced pilot would do.

    const float output = pOut + iOut - dOut + fOut;
    DEBUG_SET(DEBUG_ALTHOLD, 4, lrintf(pOut));
    DEBUG_SET(DEBUG_ALTHOLD, 5, lrintf(iOut));
    DEBUG_SET(DEBUG_ALTHOLD, 6, lrintf(dOut));
    DEBUG_SET(DEBUG_ALTHOLD, 7, lrintf(fOut));

    return output; // motor units, eg 100 means 10% of available throttle 
}

void altHoldReset(void)
{
    pidIntegral = 0.0f;
    altHoldState.targetAltitudeCm = getAltitudeCm();
    altHoldState.targetAltitudeAdjustRate = 0.0f;
}

void altHoldInit(void)
{
    altHoldState.hoverThrottle = positionControlConfig()->hover_throttle - PWM_RANGE_MIN;
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

    const float lowThreshold = 0.5f * (positionControlConfig()->hover_throttle + PWM_RANGE_MIN); // halfway between hover and MIN, e.g. 1150 if hover is 1300
    const float highThreshold = 0.5f * (positionControlConfig()->hover_throttle + PWM_RANGE_MAX); // halfway between hover and MAX, e.g. 1650 if hover is 1300
    
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

    // use PIDs to return the throttle adjustment value, add it to the hover value, and constrain
    const float throttleAdjustment = altitudePidCalculate();

    const float tiltMultiplier = 2.0f - fmaxf(getCosTiltAngle(), 0.5f);
    // 1 = flat, 1.24 at 40 degrees, max 1.5 around 60 degrees, the default limit of Angle Mode
    // 2 - cos(x) is between 1/cos(x) and 1/sqrt(cos(x)) in this range
    const float newThrottle = PWM_RANGE_MIN + (altHoldState.hoverThrottle + throttleAdjustment) * tiltMultiplier;
    altHoldState.throttleOut = constrainf(newThrottle, altholdConfig()->alt_hold_throttle_min, altholdConfig()->alt_hold_throttle_max);

    DEBUG_SET(DEBUG_ALTHOLD, 0, lrintf(altHoldState.targetAltitudeCm));
    DEBUG_SET(DEBUG_ALTHOLD, 2, lrintf(newThrottle)); // normal range 1000-2000, but is beforeconstraint 
    DEBUG_SET(DEBUG_ALTHOLD, 3, lrintf(tiltMultiplier * 100));
}

void updateAltHoldState(timeUs_t currentTimeUs) {
    UNUSED(currentTimeUs); 

    // check for enabling Alt Hold, otherwise do as little as possible while inactive
    altHoldProcessTransitions();

    if (altHoldState.isAltHoldActive) {
        altHoldUpdate();
    }
}

float altHoldGetThrottle(void) {
    // see notes in gpsRescueGetThrottle() about mincheck
    float commandedThrottle = scaleRangef(altHoldState.throttleOut, MAX(rxConfig()->mincheck, PWM_RANGE_MIN), PWM_RANGE_MAX, 0.0f, 1.0f);
    // with high values for mincheck, we could sclae to negative throttle values.
    commandedThrottle = constrainf(commandedThrottle, 0.0f, 1.0f);
    return commandedThrottle;
}

#endif
