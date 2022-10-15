/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <math.h>
#include "platform.h"

#ifdef USE_FEEDFORWARD

#include "build/debug.h"

#include "common/maths.h"

#include "fc/rc.h"

#include "flight/pid.h"

#include "feedforward.h"

static float setpointDelta[XYZ_AXIS_COUNT];
static float prevSetpoint[XYZ_AXIS_COUNT];
static float prevSetpointSpeed[XYZ_AXIS_COUNT];
static float prevAcceleration[XYZ_AXIS_COUNT];
static uint8_t duplicateCount[XYZ_AXIS_COUNT];
static uint8_t averagingCount;
static float feedforwardMaxRateLimit[XYZ_AXIS_COUNT];
static float feedforwardMaxRate[XYZ_AXIS_COUNT];

typedef struct laggedMovingAverageCombined_s {
     laggedMovingAverage_t filter;
     float buf[4];
} laggedMovingAverageCombined_t;
laggedMovingAverageCombined_t  setpointDeltaAvg[XYZ_AXIS_COUNT];

void feedforwardInit(const pidProfile_t *pidProfile)
{
    const float feedforwardMaxRateScale = pidProfile->feedforward_max_rate_limit * 0.01f;
    averagingCount = pidProfile->feedforward_averaging + 1;
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        feedforwardMaxRate[i] = applyCurve(i, 1.0f);
        feedforwardMaxRateLimit[i] = feedforwardMaxRate[i] * feedforwardMaxRateScale;
        laggedMovingAverageInit(&setpointDeltaAvg[i].filter, averagingCount, (float *)&setpointDeltaAvg[i].buf[0]);
    }
}

FAST_CODE_NOINLINE float feedforwardApply(int axis, bool newRcFrame, feedforwardAveraging_t feedforwardAveraging)
{

    if (newRcFrame) {

        const float feedforwardTransitionFactor = pidGetFeedforwardTransitionFactor();
        const float feedforwardSmoothFactor = pidGetFeedforwardSmoothFactor();
                    // good values : 25 for 111hz FrSky, 30 for 150hz, 50 for 250hz, 65 for 500hz links
        const float feedforwardJitterFactor = pidGetFeedforwardJitterFactor();
                    // 7 is default, 5 for faster links with smaller steps and for racing, 10-12 for 150hz freestyle
        const float feedforwardBoostFactor = pidGetFeedforwardBoostFactor();

        const float rxInterval = getCurrentRxRefreshRate() * 1e-6f; // 0.0066 for 150hz RC Link.
        const float rxRate = 1.0f / rxInterval; // eg 150 for a 150Hz RC link

        const float setpoint = getRawSetpoint(axis);
        const float absSetpointPercent = fabsf(setpoint) / feedforwardMaxRate[axis];

        float rcCommandDelta = getRcCommandDelta(axis);

        if (axis == FD_ROLL) {
            DEBUG_SET(DEBUG_FEEDFORWARD, 3, lrintf(rcCommandDelta * 100.0f));
            // rcCommand packet difference = value of 100 if 1000 RC steps
            DEBUG_SET(DEBUG_FEEDFORWARD, 0, lrintf(setpoint));
            // un-smoothed in blackbox
        }

        // calculate setpoint speed
        float setpointSpeed = (setpoint - prevSetpoint[axis]) * rxRate;
        float absSetpointSpeed = fabsf(setpointSpeed); // unsmoothed for kick prevention
        float absPrevSetpointSpeed = fabsf(prevSetpointSpeed[axis]);

        float setpointAcceleration = 0.0f;

        rcCommandDelta = fabsf(rcCommandDelta);

        if (rcCommandDelta) {
            // we have movement and should calculate feedforward

            // jitter attenuator falls below 1 when rcCommandDelta falls below jitter threshold
            float jitterAttenuator = 1.0f;
            if (feedforwardJitterFactor) {
                if (rcCommandDelta < feedforwardJitterFactor) {
                    jitterAttenuator = MAX(1.0f - (rcCommandDelta / feedforwardJitterFactor), 0.0f);
                    jitterAttenuator = 1.0f - jitterAttenuator * jitterAttenuator;
                }
            }

            // duplicateCount indicates number of prior duplicate/s, 1 means one only duplicate prior to this packet
            // reduce setpoint speed by half after a single duplicate or a third after two. Any more are forced to zero.
            // needed because while sticks are moving, the next valid step up will be proportionally bigger
            // and stops excessive feedforward where steps are at intervals, eg when the OpenTx ADC filter is active
            // downside is that for truly held sticks, the first feedforward step won't be as big as it should be
            if (duplicateCount[axis]) {
                setpointSpeed /= duplicateCount[axis] + 1;
            }

            // first order type smoothing for setpoint speed noise reduction
            setpointSpeed = prevSetpointSpeed[axis] + feedforwardSmoothFactor * (setpointSpeed - prevSetpointSpeed[axis]);

            // calculate acceleration from smoothed setpoint speed
            setpointAcceleration = setpointSpeed - prevSetpointSpeed[axis];

            // use rxRate to normalise acceleration to nominal RC packet interval of 100hz
            // without this, we would get less boost than we should at higher Rx rates
            // note rxRate updates with every new packet (though not every time data changes), hence
            // if no Rx packets are received for a period, boost amount is correctly attenuated in proportion to the delay
            setpointAcceleration *= rxRate * 0.01f;

            // first order acceleration smoothing (with smoothed input this is effectively second order all up)
            setpointAcceleration = prevAcceleration[axis] + feedforwardSmoothFactor * (setpointAcceleration - prevAcceleration[axis]);

            // jitter reduction to reduce acceleration spikes at low rcCommandDelta values
            // no effect for rcCommandDelta values above jitter threshold (zero delay)
            // does not attenuate the basic feedforward amount, but this is small anyway at centre due to expo
            setpointAcceleration *= jitterAttenuator;

            if (absSetpointPercent > 0.95f && absSetpointSpeed < 3.0f * absPrevSetpointSpeed) {
                // approaching max stick position so zero out feedforward to minimise overshoot
                setpointSpeed = 0.0f;
                setpointAcceleration = 0.0f;
            }

            prevSetpointSpeed[axis] = setpointSpeed;
            prevAcceleration[axis] = setpointAcceleration;

            setpointAcceleration *= feedforwardBoostFactor;

            // add attenuated boost to base feedforward and apply jitter attenuation
            setpointDelta[axis] = (setpointSpeed + setpointAcceleration) * pidGetDT() * jitterAttenuator;

            //reset counter
            duplicateCount[axis] = 0;

        } else {
            // no movement
            if (duplicateCount[axis]) {
                // increment duplicate count to max of 2
                duplicateCount[axis] += (duplicateCount[axis] < 2) ? 1 : 0;
                // second or subsequent duplicate, or duplicate when held at max stick or centre position.
                // force feedforward to zero
                setpointDelta[axis] = 0.0f;
                // zero speed and acceleration for correct smoothing of next good packet
                setpointSpeed = 0.0f;
                prevSetpointSpeed[axis] = 0.0f;
                prevAcceleration[axis] = 0.0f;
            } else {
                // first duplicate; hold feedforward and previous static values, as if we just never got anything
                duplicateCount[axis] = 1;
            }
        }


        if (axis == FD_ROLL) {
            DEBUG_SET(DEBUG_FEEDFORWARD, 1, lrintf(setpointSpeed * pidGetDT() * 100.0f)); // setpoint speed after smoothing
            DEBUG_SET(DEBUG_FEEDFORWARD, 2, lrintf(setpointAcceleration * pidGetDT() * 100.0f)); // boost amount after smoothing
            // debug 0 is interpolated setpoint, above
            // debug 3 is rcCommand delta, above
        }

        prevSetpoint[axis] = setpoint;

        // apply averaging, if enabled - include zero values in averaging
        if (feedforwardAveraging) {
            setpointDelta[axis] = laggedMovingAverageUpdate(&setpointDeltaAvg[axis].filter, setpointDelta[axis]);
        }

        // apply feedforward transition
        setpointDelta[axis] *= feedforwardTransitionFactor > 0 ? MIN(1.0f, getRcDeflectionAbs(axis) * feedforwardTransitionFactor) : 1.0f;

    }
    return setpointDelta[axis]; // the value used by the PID code
}

FAST_CODE_NOINLINE float applyFeedforwardLimit(int axis, float value, float Kp, float currentPidSetpoint)
{
    switch (axis) {
    case FD_ROLL:
        DEBUG_SET(DEBUG_FEEDFORWARD_LIMIT, 0, lrintf(value));
        break;
    case FD_PITCH:
        DEBUG_SET(DEBUG_FEEDFORWARD_LIMIT, 1, lrintf(value));
        break;
    }

    if (value * currentPidSetpoint > 0.0f) {
        if (fabsf(currentPidSetpoint) <= feedforwardMaxRateLimit[axis]) {
            value = constrainf(value, (-feedforwardMaxRateLimit[axis] - currentPidSetpoint) * Kp, (feedforwardMaxRateLimit[axis] - currentPidSetpoint) * Kp);
        } else {
            value = 0;
        }
    }

    if (axis == FD_ROLL) {
        DEBUG_SET(DEBUG_FEEDFORWARD_LIMIT, 2, lrintf(value));
    }

    return value;
}

bool shouldApplyFeedforwardLimits(int axis)
{
    return axis < FD_YAW && feedforwardMaxRateLimit[axis] != 0.0f;
}
#endif
