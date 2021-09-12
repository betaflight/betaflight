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

typedef struct laggedMovingAverageCombined_s {
     laggedMovingAverage_t filter;
     float buf[4];
} laggedMovingAverageCombined_t;

laggedMovingAverageCombined_t  setpointDeltaAvg[XYZ_AXIS_COUNT];

static float prevSetpoint[XYZ_AXIS_COUNT];
static float prevSetpointSpeed[XYZ_AXIS_COUNT];
static float prevAcceleration[XYZ_AXIS_COUNT];
static float prevRcCommandDelta[XYZ_AXIS_COUNT];
static uint8_t goodPacketCount[XYZ_AXIS_COUNT];
static uint8_t averagingCount;
static float feedforwardMaxRateLimit[XYZ_AXIS_COUNT];
static float feedforwardMaxRate[XYZ_AXIS_COUNT];
static bool interpolateDuplicates;

void feedforwardInit(const pidProfile_t *pidProfile) {
    const float feedforwardMaxRateScale = pidProfile->feedforward_max_rate_limit * 0.01f;
    averagingCount = pidProfile->feedforward_averaging + 1;
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        feedforwardMaxRate[i] = applyCurve(i, 1.0f);
        feedforwardMaxRateLimit[i] = feedforwardMaxRate[i] * feedforwardMaxRateScale;
        laggedMovingAverageInit(&setpointDeltaAvg[i].filter, averagingCount, (float *)&setpointDeltaAvg[i].buf[0]);
    }
    interpolateDuplicates = pidProfile->feedforward_interpolate_dups;
}

FAST_CODE_NOINLINE float feedforwardApply(int axis, bool newRcFrame, feedforwardAveraging_t feedforwardAveraging) {

    if (newRcFrame) {
        float rcCommandDelta = getRcCommandDelta(axis);
        float setpoint = getRawSetpoint(axis);
        const float rxInterval = getCurrentRxRefreshRate() * 1e-6f;
        const float rxRate = 1.0f / rxInterval; // eg 150 for a 150Hz RC link
        float setpointSpeed = (setpoint - prevSetpoint[axis]) * rxRate;
        float absPrevSetpointSpeed = fabsf(prevSetpointSpeed[axis]);
        float setpointAcceleration = 0.0f;
        const float feedforwardTransitionFactor = pidGetFeedforwardTransitionFactor();
        const float feedforwardSmoothFactor = pidGetFeedforwardSmoothFactor();
        const float feedforwardJitterFactor = pidGetFeedforwardJitterFactor();
        float feedforward;

        if (axis == FD_ROLL) {
            DEBUG_SET(DEBUG_FEEDFORWARD, 3, lrintf(rcCommandDelta * 100.0f)); // rcCommand packet difference = steps of 50 mean 2000 RC steps
        }
        rcCommandDelta = fabsf(rcCommandDelta);
        // calculate the jitter attenuator from average of two most recent abs rcCommand deltas vs jitter threshold
        float jitterAttenuator = 1.0f;
        if (feedforwardJitterFactor) {
            if (rcCommandDelta < feedforwardJitterFactor) {
                jitterAttenuator = MAX(1.0f - ((rcCommandDelta + prevRcCommandDelta[axis]) / 2.0f) / feedforwardJitterFactor, 0.0f);
                jitterAttenuator = 1.0f - jitterAttenuator * jitterAttenuator;
            }
        }

        const float setpointPercent = fabsf(setpoint) / feedforwardMaxRate[axis];
        float absSetpointSpeed = fabsf(setpointSpeed); // unsmoothed for kick prevention

        if (rcCommandDelta == 0.0f) {
            // duplicate packet data on this axis
            if (interpolateDuplicates && goodPacketCount[axis] >= 2 && getRxRateValid()  && setpointPercent < 0.95f) {
                // interpolate duplicates after two previous good packets and not after long dropouts or or sticks near max
                setpoint = prevSetpoint[axis] + (prevSetpointSpeed[axis] + prevAcceleration[axis] * jitterAttenuator) * rxInterval * jitterAttenuator;
                setpointSpeed = (setpoint - prevSetpoint[axis]) * rxRate;
                // recalculate speed and acceleration based on new setpoint
            } else {
                // force to zero
                setpointSpeed = 0.0f;
                jitterAttenuator = 0.0f;
            }
            goodPacketCount[axis] = 0;
        } else {
            // we have movement
            // count the number of valid steps to decide if we permit interpolation, for Tx ADC filter situations especially
            if (goodPacketCount[axis] < 2) {
                goodPacketCount[axis] += 1;
            }
        }

        prevSetpoint[axis] = setpoint;

        if (axis == FD_ROLL) {
            DEBUG_SET(DEBUG_FEEDFORWARD, 0, lrintf(setpoint)); // setpoint after interpolations
        }

        // first order type smoothing for derivative
        setpointSpeed = prevSetpointSpeed[axis] + feedforwardSmoothFactor * (setpointSpeed - prevSetpointSpeed[axis]);

        // second order smoothing for for acceleration by calculating it after smoothing setpointSpeed
        setpointAcceleration = setpointSpeed - prevSetpointSpeed[axis];
        setpointAcceleration *= jitterAttenuator * rxRate * 0.01f; // adjust boost for RC packet interval, including dropped packets
        setpointAcceleration = prevAcceleration[axis] + feedforwardSmoothFactor * (setpointAcceleration - prevAcceleration[axis]);

        prevSetpointSpeed[axis] = setpointSpeed;
        prevAcceleration[axis] = setpointAcceleration;
        prevRcCommandDelta[axis] = rcCommandDelta;

        setpointAcceleration *= pidGetDT();
        feedforward = setpointSpeed * pidGetDT();

        // calculate boost and prevent kick-back spike at max deflection
        const float feedforwardBoostFactor = pidGetFeedforwardBoostFactor();
        float boostAmount = 0.0f;
        if (feedforwardBoostFactor) {
            if (setpointPercent < 0.95f || absSetpointSpeed > 3.0f * absPrevSetpointSpeed) {
                // allow boost when returning from max, but not when hitting max on the way up
                boostAmount = feedforwardBoostFactor * setpointAcceleration;
            }
        }

        if (axis == FD_ROLL) {
            DEBUG_SET(DEBUG_FEEDFORWARD, 1, lrintf(feedforward * 100.0f)); // delta after interpolating duplicates and smoothing
            DEBUG_SET(DEBUG_FEEDFORWARD, 2, lrintf(boostAmount * 100.0f)); // boost amount after jitter reduction and smoothing
            // debug 0 is interpolated setpoint, above
            // debug 3 is rcCommand delta, above
        }

        // add attenuated boost to base feedforward
        feedforward += boostAmount;

        // apply averaging, if enabled
        if (feedforwardAveraging) {
            setpointDelta[axis] = laggedMovingAverageUpdate(&setpointDeltaAvg[axis].filter, feedforward);
        } else {
            setpointDelta[axis] = feedforward;
        }

        // apply feedforward transition
        setpointDelta[axis] *= feedforwardTransitionFactor > 0 ? MIN(1.0f, getRcDeflectionAbs(axis) * feedforwardTransitionFactor) : 1.0f;

    }
    return setpointDelta[axis];
}

FAST_CODE_NOINLINE float applyFeedforwardLimit(int axis, float value, float Kp, float currentPidSetpoint) {
    switch (axis) {
    case FD_ROLL:
        DEBUG_SET(DEBUG_FEEDFORWARD_LIMIT, 0, value);
        break;
    case FD_PITCH:
        DEBUG_SET(DEBUG_FEEDFORWARD_LIMIT, 1, value);
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
        DEBUG_SET(DEBUG_FEEDFORWARD_LIMIT, 2, value);
    }

    return value;
}

bool shouldApplyFeedforwardLimits(int axis)
{
    return feedforwardMaxRateLimit[axis] != 0.0f && axis < FD_YAW;
}
#endif
