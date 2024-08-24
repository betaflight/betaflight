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
#include "flight/position.h"
#include "sensors/acceleration.h"
#include "rx/rx.h"

#include "alt_hold.h"

typedef struct {
    float kp;
    float kd;
    float ki;
    float previousAltitude;
    float integral;
} simplePid_t;

simplePid_t simplePid;

altHoldState_t altHoldState;

#define ALT_HOLD_PID_P_SCALE  0.01f
#define ALT_HOLD_PID_I_SCALE  0.003f
#define ALT_HOLD_PID_D_SCALE  0.01f

static pt2Filter_t altHoldDeltaLpf;
static const float taskIntervalSeconds = 1.0f / ALTHOLD_TASK_RATE_HZ; // i.e. 0.01 s

float altitudePidCalculate(void)
{
    const float altErrorCm = altHoldState.targetAltitudeCm - altHoldState.measuredAltitudeCm;

    // P
    const float pOut = simplePid.kp * altErrorCm;

    // I
    simplePid.integral += altErrorCm * taskIntervalSeconds * simplePid.ki; // cm * seconds * ki
    // arbitrary limit on iTerm, same as for gps_rescue, +/-20% throttle
    simplePid.integral = constrainf(simplePid.integral, -200.0f, 200.0f); 
    const float iOut = simplePid.integral;

    // D
    float dOut = simplePid.kd * altHoldState.smoothedAltitudeDelta;

    // F
    const float fOut = altholdConfig()->alt_hold_pid_d * altHoldState.targetAltitudeAdjustRate;
    // if error is used, we get a 'free kick' in derivative from changes in the target value
    // but this is delayed by the smoothing, leading to lag and overshoot.
    // calculating feedforward separately avoids the filter lag.

    const float output = pOut + iOut + dOut + fOut;

    DEBUG_SET(DEBUG_ALTHOLD, 4, lrintf(pOut));
    DEBUG_SET(DEBUG_ALTHOLD, 5, lrintf(iOut));
    DEBUG_SET(DEBUG_ALTHOLD, 6, lrintf(dOut));
    DEBUG_SET(DEBUG_ALTHOLD, 7, lrintf(fOut));

    return output;
}

void simplePidInit(float kp, float ki, float kd)
{
    simplePid.kp = kp;
    simplePid.ki = ki;
    simplePid.kd = kd;
    simplePid.previousAltitude = 0.0f;
    simplePid.integral = 0.0f;
}

void altHoldReset(void)
{
    altHoldState.targetAltitudeCm = altHoldState.measuredAltitudeCm;
    simplePid.integral = 0.0f;
    altHoldState.targetAltitudeAdjustRate = 0.0f;
}

void altHoldInit(void)
{
    simplePidInit(
        ALT_HOLD_PID_P_SCALE * altholdConfig()->alt_hold_pid_p,
        ALT_HOLD_PID_I_SCALE * altholdConfig()->alt_hold_pid_i,
        ALT_HOLD_PID_D_SCALE * altholdConfig()->alt_hold_pid_d);
        // the multipliers are base scale factors
        // iTerm is relatively weak, intended to be slow moving to adjust baseline errors
        // the Hover value is important otherwise takes time for iTerm to correct
        // High P will wobble readily
        // with these scalers, the same numbers as for GPS Rescue work OK for altHold
        // the goal is to share these gain factors, if practical for all altitude controllers

    //setup altitude D filter
    const float cutoffHz = 0.01f * positionConfig()->altitude_d_lpf; // default 1Hz, time constant about 160ms
    const float gain = pt2FilterGain(cutoffHz, taskIntervalSeconds);
    pt2FilterInit(&altHoldDeltaLpf, gain);

    altHoldState.hover = positionConfig()->hover_throttle - PWM_RANGE_MIN;
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

    const float lowThreshold = 0.5f * (positionConfig()->hover_throttle + PWM_RANGE_MIN); // halfway between hover and MIN, e.g. 1150 if hover is 1300
    const float highThreshold = 0.5f * (positionConfig()->hover_throttle + PWM_RANGE_MAX); // halfway between hover and MAX, e.g. 1650 if hover is 1300
    
    float throttleAdjustmentFactor = 0.0f;
    if (rcThrottle < lowThreshold) {
        throttleAdjustmentFactor = scaleRangef(rcThrottle, PWM_RANGE_MIN, lowThreshold, -1.0f, 0.0f);
    } else if (rcThrottle > highThreshold) {
        throttleAdjustmentFactor = scaleRangef(rcThrottle, highThreshold, PWM_RANGE_MAX, 0.0f, 1.0f);
    }

    // if failsafe is active, and we get here, we are in failsafe landing mode
    if (failsafeIsActive()) {
        // descend at twice the configured descent rate if higher up, to minimise the time taken to reach ground
        // default landing time is now 60s so won't time out, in defaults, unless height at start is above about 110m
        // max allowed landing time is 250s ie up to about 500m high is possible without timeout disarm on default descent rate
        // or 1000m on double the default descent rate
        // very high altitude initiations are unsafe fue to drift potential
        throttleAdjustmentFactor = isAltitudeLow() ? -1.0f : -2.0f;
    }

    altHoldState.targetAltitudeAdjustRate = throttleAdjustmentFactor * altholdConfig()->alt_hold_target_adjust_rate * taskIntervalSeconds;
    // if taskRate is 100Hz, default adjustRate of 100 adds/subtracts 1m every second, or 1cm per task run, at full stick position
    altHoldState.targetAltitudeCm += altHoldState.targetAltitudeAdjustRate;
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
    const float newThrottle = PWM_RANGE_MIN + (altHoldState.hover + throttleAdjustment) * tiltMultiplier;
    altHoldState.throttleOut = constrainf(newThrottle, altholdConfig()->alt_hold_throttle_min, altholdConfig()->alt_hold_throttle_max);

    DEBUG_SET(DEBUG_ALTHOLD, 0, lrintf(altHoldState.targetAltitudeCm));
    DEBUG_SET(DEBUG_ALTHOLD, 2, lrintf(throttleAdjustment));
}

void updateAltHoldState(timeUs_t currentTimeUs) {
    UNUSED(currentTimeUs);

    // things that always happen
    // calculate smoothed altitude Delta always, for effective value on 1st pass
    altHoldState.measuredAltitudeCm = getAltitude();
    float derivative = (simplePid.previousAltitude - altHoldState.measuredAltitudeCm) / taskIntervalSeconds; // cm/s
    simplePid.previousAltitude = altHoldState.measuredAltitudeCm;

    // smooth altitude delta here to always have a current value, without delay due to filter lag
    // this way we immediately have useful D on initialising the hold
    altHoldState.smoothedAltitudeDelta = pt2FilterApply(&altHoldDeltaLpf, derivative);

    DEBUG_SET(DEBUG_ALTHOLD, 1, lrintf(altHoldState.measuredAltitudeCm));

    altHoldProcessTransitions();

    if (altHoldState.isAltHoldActive) {
        altHoldUpdate();
    }
}

float altHoldGetThrottle(void) {
    return scaleRangef(altHoldState.throttleOut, MAX(rxConfig()->mincheck, PWM_RANGE_MIN), PWM_RANGE_MAX, 0.0f, 1.0f);
}

#endif
