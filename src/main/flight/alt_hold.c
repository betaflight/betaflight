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
#include "alt_hold.h"

#ifdef USE_ALTHOLD_MODE

#include "drivers/time.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/position.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "config/config.h"
#include "rx/rx.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "fc/rc.h"
#include "osd/osd.h"
#include "common/printf.h"
#include "common/maths.h"
#include "math.h"
#include "build/debug.h"

typedef struct {
    float kp;
    float kd;
    float ki;
    float lastAltError;
    float integral;
} simplePid_s;

simplePid_s simplePid;

altHoldState_s altHoldState;

#define ALT_HOLD_PID_P_GAIN  0.01f
#define ALT_HOLD_PID_I_GAIN  0.001f
#define ALT_HOLD_PID_D_GAIN  0.01f

PG_REGISTER_WITH_RESET_TEMPLATE(altholdConfig_t, altholdConfig, PG_ALTHOLD_CONFIG, 3);

PG_RESET_TEMPLATE(altholdConfig_t, altholdConfig,
    .altHoldPidP = 15,
    .altHoldPidI = 15,
    .altHoldPidD = 15,

    .altHoldThrottleMin = 1100,
    .altHoldThrottleMax = 1700,
    .altHoldThrottleHover = 1275,

    .altHoldTargetAdjustRate = 100, // max rate of change of altitude target using sticks in mm/s
);

static pt2Filter_t altHoldDeltaLpf;
static pt2Filter_t altHoldAccVerticalLpf;
const float taskIntervalSeconds = 1.0f / ALTHOLD_TASK_PERIOD; // i.e. 0.01 s

float altitudePidCalculate(void)
{
    const float altErrorCm = altHoldState.targetAltitudeCm - altHoldState.measuredAltitudeCm;
    const float pOut = simplePid.kp * altErrorCm;

    simplePid.integral += altErrorCm * taskIntervalSeconds; // cm * seconds
    float iOut = simplePid.ki * simplePid.integral;
    // arbitrary limit on iTerm, same as for gps_rescue, +/-20% throttle
    iOut = constrainf(iOut, -200.0f, 200.0f); 

    const float derivative = (altErrorCm - simplePid.lastAltError) / taskIntervalSeconds; // cm/s
    simplePid.lastAltError = altErrorCm;
    float dOut = simplePid.kd * derivative;
    // PT2 filtering like for GPS Rescue
    dOut = pt2FilterApply(&altHoldDeltaLpf, dOut);
    // error is used as source so that we get a 'kick' in derivative from changes in target altitude

    const float output = pOut + iOut + dOut;

    DEBUG_SET(DEBUG_ALTHOLD, 5, (lrintf)(pOut));
    DEBUG_SET(DEBUG_ALTHOLD, 6, (lrintf)(iOut));
    DEBUG_SET(DEBUG_ALTHOLD, 7, (lrintf)(dOut));

    return output;
}

void simplePidInit(float kp, float ki, float kd)
{
    simplePid.kp = kp;
    simplePid.ki = ki;
    simplePid.kd = kd;
    simplePid.lastAltError = 0.0f;
    simplePid.integral = 0.0f;
}

void altHoldReset(void)
{
    altHoldState.targetAltitudeCm = getAltitude(); // current altitude in cm
    simplePid.lastAltError = 0.0f;
    simplePid.integral = 0.0f;
    altHoldState.velocityFromAcc = 0.0f; // reset the integral accumulator to zero
    altHoldState.throttleOut = altholdConfig()->altHoldThrottleHover;
}

void altHoldInit(void)
{
    simplePidInit(
        ALT_HOLD_PID_P_GAIN * altholdConfig()->altHoldPidP,
        ALT_HOLD_PID_I_GAIN * altholdConfig()->altHoldPidI,
        ALT_HOLD_PID_D_GAIN * altholdConfig()->altHoldPidD);
        // the multipliers are base scale factors
        // iTerm is relatively weak, intended to be slow moving to adjust baseline errors
        // hence Hover value is important otherwise takes time for iTerm to correct
        // and high P will wobble readily
        // with these scalers, the same numbers as for GPS Rescue work OK for altHold
        // the goal is to share these gain factors, if practical for all altitude controllers

    //setup altitude D filter
    const float cutoffHz = 0.01f * positionConfig()->altitude_d_lpf; // default 1Hz, time constant about 160ms
    const float gain = pt2FilterGain(cutoffHz, taskIntervalSeconds);
    pt2FilterInit(&altHoldDeltaLpf, gain); 

    // similar PT2 on accelerometer to remove noise
    const float gainAcc = pt2FilterGain((float)accelerometerConfig()->acc_lpf_hz, taskIntervalSeconds);
    pt2FilterInit(&altHoldAccVerticalLpf, gainAcc);

    altHoldState.altHoldActive = false;
    altHoldReset();
}


void altHoldProcessTransitions(void) {
    bool altHoldRequested = FLIGHT_MODE(ALTHOLD_MODE);

    if (altHoldRequested && !altHoldState.altHoldActive) {
        altHoldReset();
    }
    altHoldState.altHoldActive = altHoldRequested;

    // ** the transition out of alt hold (exiting altHold) may be rough.  Some notes... **
    // The original PR had a gradual transition from hold throttle to pilot control throttle
    // using !(altHoldRequested && altHoldState.altHoldActive) to run an exit function
    // a cross-fade factor was sent to mixer.c based on time since the flight mode request was terminated
    // it was removed primarily to simplify this PR

    // hence in this PR's the user's throttle needs to be close to the hover throttle value on exiting altHold
    // its not so bad because the 'target adjustment' by throttle requires that
    // user throttle must be not more than half way out from hover for a stable hold
}

void altHoldUpdateTargetAltitude(void)
{
    // this was part of the oritinal code, modifed a bit to be either side of hover point.
    // it works very well.
    // The user van raise or lower the target altitude with throttle up;  there is a big deadband
    // Max rate for climb/descend is 1m/s by default (up to 2.5 is allowed but overshoots a fair bit)

    // some people may not like making the target altitude adustable, because 
    // with throttle adjustment, hitting the switch won't always hold altitude if throttle is bumped
    // eg if the throttle is bumped low accidentally, quad will start descending.
    // on the plus side, the pilot has control nice control over altitude, and the deadband is wide
    // if the craft is stable, the throttle can't be full up or full down
    // this is helpful when exiting the hold, avoiding bad sudden climbs or falls

    const float rcThrottle = rcCommand[THROTTLE];

    const float lowThreshold = 0.5f * (altholdConfig()->altHoldThrottleHover + PWM_RANGE_MIN); // around 1150
    const float highThreshold = 0.5f * (altholdConfig()->altHoldThrottleHover + PWM_RANGE_MAX); // around 1500
    
    float scaler = 0.0f;
    if (rcThrottle < lowThreshold) {
        scaler = scaleRangef(rcThrottle, PWM_RANGE_MIN, lowThreshold, -1.0f, 0.0f);
    } else if (rcThrottle > highThreshold) {
        scaler = scaleRangef(rcThrottle, highThreshold, PWM_RANGE_MAX, 0.0f, 1.0f);
    }

    const float altitudeTargetDelta = scaler * altholdConfig()->altHoldTargetAdjustRate * 0.01f;
    // task is 100Hz, default adjustRate of 100 adds/subtracts 1m every second at full stick position
    const float newTargetAltitude = altHoldState.targetAltitudeCm + altitudeTargetDelta;
    altHoldState.targetAltitudeCm = newTargetAltitude;
}

void altHoldUpdate(void)
{
    altHoldProcessTransitions(); // and initialise values if we start

    // things that always need to happen in the background should go here
    altHoldState.measuredAltitudeCm = getAltitude();
    DEBUG_SET(DEBUG_ALTHOLD, 1, (lrintf)(altHoldState.measuredAltitudeCm));

    // exit unless altHold is active
    if (!altHoldState.altHoldActive) {
        DEBUG_SET(DEBUG_ALTHOLD, 0, 0);
        return;
    }


    // ** for testing purposes... **

    // estimate the vertical velocity from vertical accelerometer
    // not used at present, but logged.
    // my testing doesn't support using Accelerometer data for altitude control
    // simple Z axis value might be OK when altHold is active mode since, unless windy, quad is close to level
    t_fp_vector accelerationVector = {{
        acc.accADC[X],
        acc.accADC[Y],
        acc.accADC[Z]
    }};
    imuTransformVectorBodyToEarth(&accelerationVector);
    float measuredAccel = 9.8f * (accelerationVector.V.Z - acc.dev.acc_1G) / acc.dev.acc_1G; // m/s^2

    // integrate into velocityFromAcc
    // note that on commencing we may need to set the initial integral value to something derived from baro/GPS
    altHoldState.velocityFromAcc += measuredAccel * taskIntervalSeconds; // m/s

    // apply a leaky integrator or lowpass of time constant approx 10s (from original code *0.999)
    // modified to leak with time constant approx 1s at 100Hz - can be detailed better later.
    altHoldState.velocityFromAcc *= 0.99f;

    // smooth the accelerometer values with its PT2
    measuredAccel = pt2FilterApply(&altHoldAccVerticalLpf, measuredAccel);

    // log at 100x for resolution
    DEBUG_SET(DEBUG_ALTHOLD, 3, (lrintf)(measuredAccel * 100.0f)); // cm/s^2
    DEBUG_SET(DEBUG_ALTHOLD, 4, (lrintf)(altHoldState.velocityFromAcc * 100.0f)); // cm/s * 10 ??

    // ** end vertical acceleration stuff **


    // check if the user has modified the target altitude using sticks while hovering
    altHoldUpdateTargetAltitude();

    // use PIDs to return the throttle adjustment value, and add it to the throttle value to use
    const float throttleAdjustment = altitudePidCalculate();

    float newThrottle = altholdConfig()->altHoldThrottleHover + throttleAdjustment;
    altHoldState.throttleOut = constrainf(newThrottle, altholdConfig()->altHoldThrottleMin, altholdConfig()->altHoldThrottleMax);

    DEBUG_SET(DEBUG_ALTHOLD, 0, (lrintf)(altHoldState.targetAltitudeCm));
    DEBUG_SET(DEBUG_ALTHOLD, 2, (lrintf)(throttleAdjustment));
}

void initAltHoldState(void) {
    altHoldInit();
}

void updateAltHoldState(timeUs_t currentTimeUs) {
    altHoldUpdate();
    UNUSED(currentTimeUs);
}

float altHoldGetThrottle(void) {
    return scaleRangef(altHoldState.throttleOut, MAX(rxConfig()->mincheck, PWM_RANGE_MIN), PWM_RANGE_MAX, 0.0f, 1.0f);
}

bool altHoldIsActive(void){
    return altHoldState.altHoldActive;
}
#endif
