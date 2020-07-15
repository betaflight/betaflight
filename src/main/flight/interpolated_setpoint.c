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

#ifdef USE_INTERPOLATED_SP

#include "build/debug.h"

#include "common/maths.h"

#include "fc/rc.h"

#include "flight/pid.h"

#include "interpolated_setpoint.h"

static float setpointDeltaImpl[XYZ_AXIS_COUNT];
static float setpointDelta[XYZ_AXIS_COUNT];
static uint8_t holdCount[XYZ_AXIS_COUNT];

typedef struct laggedMovingAverageCombined_s {
     laggedMovingAverage_t filter;
     float buf[4];
} laggedMovingAverageCombined_t;

laggedMovingAverageCombined_t  setpointDeltaAvg[XYZ_AXIS_COUNT];

static float prevSetpointSpeed[XYZ_AXIS_COUNT];
static float prevAcceleration[XYZ_AXIS_COUNT];
static float prevRawSetpoint[XYZ_AXIS_COUNT];
static float prevDeltaImpl[XYZ_AXIS_COUNT];
static bool bigStep[XYZ_AXIS_COUNT];
static uint8_t averagingCount;

// Configuration
static float ffMaxRateLimit[XYZ_AXIS_COUNT];
static float ffMaxRate[XYZ_AXIS_COUNT];

void interpolatedSpInit(const pidProfile_t *pidProfile) {
    const float ffMaxRateScale = pidProfile->ff_max_rate_limit * 0.01f;
    averagingCount = pidProfile->ff_interpolate_sp;
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        ffMaxRate[i] = applyCurve(i, 1.0f);
        ffMaxRateLimit[i] = ffMaxRate[i] * ffMaxRateScale;
        laggedMovingAverageInit(&setpointDeltaAvg[i].filter, averagingCount, (float *)&setpointDeltaAvg[i].buf[0]);
    }
}

FAST_CODE_NOINLINE float interpolatedSpApply(int axis, bool newRcFrame, ffInterpolationType_t type) {

    if (newRcFrame) {
        float rawSetpoint = getRawSetpoint(axis);

        const float rxInterval = getCurrentRxRefreshRate() * 1e-6f;
        const float rxRate = 1.0f / rxInterval;
        float setpointSpeed = (rawSetpoint - prevRawSetpoint[axis]) * rxRate;
        float setpointAcceleration = setpointSpeed - prevSetpointSpeed[axis];
        float setpointSpeedModified = setpointSpeed;
        float setpointAccelerationModified = setpointAcceleration;

        // Glitch reduction code for identical packets
        if (fabsf(setpointAcceleration) > 3.0f * fabsf(prevAcceleration[axis])) {
            bigStep[axis] = true;
        } else {
            bigStep[axis] = false;
        }

        if (setpointSpeed == 0 && fabsf(rawSetpoint) < 0.98f * ffMaxRate[axis]) {
            // identical packet detected, not at full deflection.
            // first packet on leaving full deflection always gets full FF
            if (holdCount[axis] == 0) {
                // previous packet had movement
                if (bigStep[axis]) {
                    // type 1 = interpolate forward where acceleration change is large
                    setpointSpeedModified = prevSetpointSpeed[axis];
                    setpointAccelerationModified = prevAcceleration[axis];
                    holdCount[axis] = 1;
                } else {
                    // type 2 = small change, no interpolation needed
                    setpointSpeedModified = 0.0f;
                    setpointSpeed = setpointSpeed / 2.0f;
                    holdCount[axis] = 2;
                }
            } else {
                // it is an unchanged packet after previous unchanged packet
                // speed and acceleration will be zero, no need to change anything
                holdCount[axis] = 3;
            }
        } else {
            // we're moving, or sticks are at max
            if (holdCount[axis] != 0) {
                // previous step was a duplicate, handle each type differently
                if (holdCount[axis] == 1) {
                    // interpolation was applied
                    // raw setpoint speed of next 'good' packet is twice what it should be
                    setpointSpeedModified = setpointSpeed / 2.0f;
                    setpointSpeed = setpointSpeedModified;
                    // empirically this works best
                    setpointAccelerationModified = (prevAcceleration[axis] + setpointAcceleration) / 2.0f;
                } else if (holdCount[axis] == 2) {
                    // interpolation was not applied
                } else if (holdCount[axis] == 3) {
                    // after persistent flat period, no boost
                    // reduces jitter from boost when flying smooth lines
                    // but only when no ff_averaging is active, eg hard core race setups
                    // WARNING: this means no boost if ADC is active on FrSky radios
                    if (averagingCount > 1) {
                        setpointAccelerationModified /= averagingCount;
                    }
                }
                holdCount[axis] = 0;
            }
        }

        // smooth deadband type suppression of FF jitter when sticks are at or returning to centre
        // only when ff_averaging is 3 or more, for HD or cinematic flying
        if (averagingCount > 2) {
            const float rawSetpointCentred = fabsf(rawSetpoint) / averagingCount;
            if (rawSetpointCentred < 1.0f) {
                setpointSpeedModified *= rawSetpointCentred;
                setpointAccelerationModified *= rawSetpointCentred;
                holdCount[axis] = 4;
            }
        }

        setpointDeltaImpl[axis] = setpointSpeedModified * pidGetDT();
        prevAcceleration[axis] = setpointAcceleration;

        setpointAcceleration *= pidGetDT();
        setpointAccelerationModified *= pidGetDT();

        const float ffBoostFactor = pidGetFfBoostFactor();
        float clip = 1.0f;
        float boostAmount = 0.0f;
        if (ffBoostFactor != 0.0f) {
            //calculate clip factor to reduce boost on big spikes
            if (pidGetSpikeLimitInverse()) {
                clip = 1 / (1 + (setpointAcceleration * setpointAcceleration * pidGetSpikeLimitInverse()));
                clip *= clip;
            }
            // don't clip first step inwards from max deflection
            if (fabsf(prevRawSetpoint[axis]) > 0.95f * ffMaxRate[axis] && fabsf(setpointSpeed) > 3.0f * fabsf(prevSetpointSpeed[axis])) {
                clip = 1.0f;
            }
            // calculate boost and prevent kick-back spike at max deflection
            if (fabsf(rawSetpoint) < 0.95f * ffMaxRate[axis] || fabsf(setpointSpeed) > 3.0f * fabsf(prevSetpointSpeed[axis])) {
                boostAmount = ffBoostFactor * setpointAccelerationModified;
            }
        }

        prevSetpointSpeed[axis] = setpointSpeed;
        prevRawSetpoint[axis] = rawSetpoint;

        if (axis == FD_ROLL) {
            DEBUG_SET(DEBUG_FF_INTERPOLATED, 0, lrintf(setpointDeltaImpl[axis] * 100));
            DEBUG_SET(DEBUG_FF_INTERPOLATED, 1, lrintf(setpointAccelerationModified * 100));
            DEBUG_SET(DEBUG_FF_INTERPOLATED, 2, lrintf(setpointAcceleration * 100));
            DEBUG_SET(DEBUG_FF_INTERPOLATED, 3, holdCount[axis]);
        }

        setpointDeltaImpl[axis] += boostAmount * clip;

        // first order (kind of) smoothing of FF
        const float ffSmoothFactor = pidGetFfSmoothFactor();
        setpointDeltaImpl[axis] = prevDeltaImpl[axis] + ffSmoothFactor * (setpointDeltaImpl[axis] - prevDeltaImpl[axis]);
        prevDeltaImpl[axis] = setpointDeltaImpl[axis];

        if (type == FF_INTERPOLATE_ON) {
            setpointDelta[axis] = setpointDeltaImpl[axis];
        } else {
            setpointDelta[axis] = laggedMovingAverageUpdate(&setpointDeltaAvg[axis].filter, setpointDeltaImpl[axis]);
        }
    }
    return setpointDelta[axis];
}

FAST_CODE_NOINLINE float applyFfLimit(int axis, float value, float Kp, float currentPidSetpoint) {
    switch (axis) {
    case FD_ROLL:
        DEBUG_SET(DEBUG_FF_LIMIT, 0, value);

        break;
    case FD_PITCH:
        DEBUG_SET(DEBUG_FF_LIMIT, 1, value);

        break;
    }

    if (fabsf(currentPidSetpoint) <= ffMaxRateLimit[axis]) {
        value = constrainf(value, (-ffMaxRateLimit[axis] - currentPidSetpoint) * Kp, (ffMaxRateLimit[axis] - currentPidSetpoint) * Kp);
    } else {
        value = 0;
    }

    if (axis == FD_ROLL) {
        DEBUG_SET(DEBUG_FF_LIMIT, 2, value);
    }

    return value;
}

bool shouldApplyFfLimits(int axis)
{
    return ffMaxRateLimit[axis] != 0.0f && axis < FD_YAW;
}
#endif
