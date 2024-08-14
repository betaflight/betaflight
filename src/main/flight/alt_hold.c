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

#ifdef USE_ALTHOLD_MODE

#include "math.h"
#include "build/debug.h"

#include "config/config.h"
#include "fc/runtime_config.h"
#include "fc/rc.h"
#include "flight/position.h"
#include "sensors/acceleration.h"
#include "rx/rx.h"

#include "alt_hold.h"

typedef struct {
    float kp;
    float kd;
    float ki;
    float lastAltitude;
    float integral;
} simplePid_t;

simplePid_t simplePid;

altHoldState_s altHoldState;

#define ALT_HOLD_PID_P_GAIN  0.01f
#define ALT_HOLD_PID_I_GAIN  0.003f
#define ALT_HOLD_PID_D_GAIN  0.01f

PG_REGISTER_WITH_RESET_TEMPLATE(altholdConfig_t, altholdConfig, PG_ALTHOLD_CONFIG, 3);

PG_RESET_TEMPLATE(altholdConfig_t, altholdConfig,
    .altHoldPidP = 15,
    .altHoldPidI = 15,
    .altHoldPidD = 15,
    .altHoldThrottleMin = 1100,
    .altHoldThrottleMax = 1700,
    .altHoldTargetAdjustRate = 100, // max rate of change of altitude target using sticks in mm/s
);

static pt2Filter_t altHoldDeltaLpf;
static pt2Filter_t altHoldAccVerticalLpf;
const float taskIntervalSeconds = 1.0f / ALTHOLD_TASK_RATE_HZ; // i.e. 0.01 s

float altitudePidCalculate(void)
{
    const float altErrorCm = altHoldState.targetAltitudeCm - altHoldState.measuredAltitudeCm;

    // P
    const float pOut = simplePid.kp * altErrorCm;

    // I
    simplePid.integral += altErrorCm * taskIntervalSeconds; // cm * seconds
    float iOut = simplePid.ki * simplePid.integral;
    // arbitrary limit on iTerm, same as for gps_rescue, +/-20% throttle
    iOut = constrainf(iOut, -200.0f, 200.0f); 

    // D
    const float derivative = (simplePid.lastAltitude - altHoldState.measuredAltitudeCm) / taskIntervalSeconds; // cm/s
    simplePid.lastAltitude = altHoldState.measuredAltitudeCm;
    float dOut = simplePid.kd * derivative;
    // PT2 filtering at altitude_d_lpf / 100 cutoff, default 1s, the same as for GPS Rescue 
    dOut = pt2FilterApply(&altHoldDeltaLpf, dOut);

    // F
    const float fOut = altholdConfig()->altHoldPidD * altHoldState.targetAltitudeDelta;
    // if error is used, we get a 'free kick' in derivative from changes in the target value
    // but this is delayed by the smoothing, leading to lag and overshoot.
    // calculating feedforward separately avoids the filter lag.

    const float output = pOut + iOut + dOut + fOut;

    DEBUG_SET(DEBUG_ALTHOLD, 3, (lrintf)(output));
    DEBUG_SET(DEBUG_ALTHOLD, 4, (lrintf)(pOut));
    DEBUG_SET(DEBUG_ALTHOLD, 5, (lrintf)(iOut));
    DEBUG_SET(DEBUG_ALTHOLD, 6, (lrintf)(dOut));
    DEBUG_SET(DEBUG_ALTHOLD, 7, (lrintf)(fOut));

    return output;
}

void simplePidInit(float kp, float ki, float kd)
{
    simplePid.kp = kp;
    simplePid.ki = ki;
    simplePid.kd = kd;
    simplePid.lastAltitude = 0.0f;
    simplePid.integral = 0.0f;
}

void altHoldReset(void)
{
    altHoldState.targetAltitudeCm = getAltitude(); // current altitude in cm
    simplePid.lastAltitude = altHoldState.measuredAltitudeCm;
    simplePid.integral = 0.0f;
    altHoldState.throttleOut = positionConfig()->hover_throttle;
    altHoldState.targetAltitudeDelta = 0.0f;
}

void altHoldInit(void)
{
    simplePidInit(
        ALT_HOLD_PID_P_GAIN * altholdConfig()->altHoldPidP,
        ALT_HOLD_PID_I_GAIN * altholdConfig()->altHoldPidI,
        ALT_HOLD_PID_D_GAIN * altholdConfig()->altHoldPidD);
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

    // similar PT2 on accelerometer to remove noise
    const float gainAcc = pt2FilterGain((float)accelerometerConfig()->acc_lpf_hz, taskIntervalSeconds);
    pt2FilterInit(&altHoldAccVerticalLpf, gainAcc);

    altHoldState.isAltHoldActive = false;
    altHoldReset();
}

void altHoldProcessTransitions(void) {
    bool altHoldRequested = FLIGHT_MODE(ALTHOLD_MODE);

    if (altHoldRequested && !altHoldState.isAltHoldActive) {
        altHoldReset();
    }
    altHoldState.isAltHoldActive = altHoldRequested;

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
    // The user van raise or lower the target altitude with throttle up;  there is a big deadband.
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

    const float lowThreshold = 0.5f * (positionConfig()->hover_throttle + PWM_RANGE_MIN); // around 1150
    const float highThreshold = 0.5f * (positionConfig()->hover_throttle + PWM_RANGE_MAX); // around 1500
    
    float throttleAdjustmentFactor = 0.0f;
    if (rcThrottle < lowThreshold) {
        throttleAdjustmentFactor = scaleRangef(rcThrottle, PWM_RANGE_MIN, lowThreshold, -1.0f, 0.0f);
    } else if (rcThrottle > highThreshold) {
        throttleAdjustmentFactor = scaleRangef(rcThrottle, highThreshold, PWM_RANGE_MAX, 0.0f, 1.0f);
    }

    altHoldState.targetAltitudeDelta = throttleAdjustmentFactor * altholdConfig()->altHoldTargetAdjustRate * taskIntervalSeconds;
    // if taskRate is 100Hz, default adjustRate of 100 adds/subtracts 1m every second, or 1cm per task run, at full stick position
    altHoldState.targetAltitudeCm += altHoldState.targetAltitudeDelta;
}

void altHoldUpdate(void)
{
    altHoldProcessTransitions(); // initialise values at the start

    // things that always need to happen in the background should go here
    altHoldState.measuredAltitudeCm = getAltitude();
    DEBUG_SET(DEBUG_ALTHOLD, 1, (lrintf)(altHoldState.measuredAltitudeCm));

    // exit unless altHold is active
    if (!altHoldState.isAltHoldActive) {
        DEBUG_SET(DEBUG_ALTHOLD, 0, 0);
        return;
    }

    // check if the user has changed the target altitude using sticks
    if (altholdConfig()->altHoldTargetAdjustRate) {
        altHoldUpdateTargetAltitude();
    }

    // use PIDs to return the throttle adjustment value, add it to the hover value, and constrain
    const float throttleAdjustment = altitudePidCalculate();
    float newThrottle = positionConfig()->hover_throttle + throttleAdjustment;
    altHoldState.throttleOut = constrainf(newThrottle, altholdConfig()->altHoldThrottleMin, altholdConfig()->altHoldThrottleMax);

    DEBUG_SET(DEBUG_ALTHOLD, 0, (lrintf)(altHoldState.targetAltitudeCm));
    DEBUG_SET(DEBUG_ALTHOLD, 2, (lrintf)(throttleAdjustment));
}

void updateAltHoldState(timeUs_t currentTimeUs) {
    altHoldUpdate();
    UNUSED(currentTimeUs);
}

float altHoldGetThrottle(void) {
    return scaleRangef(altHoldState.throttleOut, MAX(rxConfig()->mincheck, PWM_RANGE_MIN), PWM_RANGE_MAX, 0.0f, 1.0f);
}

bool altHoldIsActive(void){
    return altHoldState.isAltHoldActive;
}
#endif
