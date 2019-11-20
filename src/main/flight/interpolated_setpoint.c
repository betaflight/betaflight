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
#include "flight/interpolated_setpoint.h"

#define PREV_BIG_STEP 1000.0f //threshold for size of jump of packet before the identical data packet

static float setpointDeltaImpl[XYZ_AXIS_COUNT];
static float setpointDelta[XYZ_AXIS_COUNT];
static uint8_t holdCount[XYZ_AXIS_COUNT];

typedef struct laggedMovingAverageCombined_u {
     laggedMovingAverage_t filter;
     float buf[4];
} laggedMovingAverageCombined_t;

laggedMovingAverageCombined_t  setpointDeltaAvg[XYZ_AXIS_COUNT];
 
static float prevSetpointSpeed[XYZ_AXIS_COUNT];
static float prevAcceleration[XYZ_AXIS_COUNT];
static float prevRawSetpoint[XYZ_AXIS_COUNT];
static float prevDeltaImpl[XYZ_AXIS_COUNT];
static bool bigStep[XYZ_AXIS_COUNT];

// Configuration
static float ffMaxRateLimit[XYZ_AXIS_COUNT];
static float ffMaxRate[XYZ_AXIS_COUNT];

void interpolatedSpInit(const pidProfile_t *pidProfile) {
    const float ffMaxRateScale = pidProfile->ff_max_rate_limit * 0.01f;
    uint8_t j = pidProfile->ff_interpolate_sp;
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        ffMaxRate[i] = applyCurve(i, 1.0f);
        ffMaxRateLimit[i] = ffMaxRate[i] * ffMaxRateScale;
        laggedMovingAverageInit(&setpointDeltaAvg[i].filter, j, (float *)&setpointDeltaAvg[i].buf[0]);
    }
}

FAST_CODE_NOINLINE float interpolatedSpApply(int axis, bool newRcFrame, ffInterpolationType_t type) {

    if (newRcFrame) {
        float rawSetpoint = getRawSetpoint(axis); 
        
        const float rxInterval = currentRxRefreshRate * 1e-6f;
        const float rxRate = 1.0f / rxInterval;

        float setpointSpeed = (rawSetpoint - prevRawSetpoint[axis]) * rxRate;
        float setpointAcceleration = setpointSpeed - prevSetpointSpeed[axis];
        const uint8_t holdSteps = 2;

        // Glitch reduction code for identical packets
        if (setpointSpeed == 0 && fabsf(rawSetpoint) < 0.98f * ffMaxRate[axis]) {
            // identical packets, not at full deflection
            if (holdCount[axis] < holdSteps && fabsf(rawSetpoint) > 2.0f && !bigStep[axis]) {
                // holding the entire previous speed is best for missed packets, but bad for early packets
                setpointSpeed = prevSetpointSpeed[axis] + prevAcceleration[axis];
                setpointAcceleration = prevAcceleration[axis];
                holdCount[axis] += 1;
            } else {
                // identical packets for more than hold steps, or prev big step
                // lock acceleration to zero and don't interpolate forward until sticks move again
                holdCount[axis] = holdSteps + 1;
                setpointAcceleration = 0.0f;
            }
        } else {
            // we're moving, or sticks are at max
            if (holdCount[axis] == 2) {
                // we are after an identical packet, and the one before was a normal step up,
                // so raw step speed and acceleration of next 'good' packet is twice what it should be
                setpointSpeed /= 2.0f;
                setpointAcceleration /= 2.0f;
            }
            if (holdCount[axis] == 3) {
                // we are starting to move after a gap after a big step up, or persistent flat period, so accelerate gently
                setpointAcceleration = 0.0f;
            }
            holdCount[axis] = 1;
            if (fabsf(setpointAcceleration - prevAcceleration[axis]) > PREV_BIG_STEP) {
                bigStep[axis] = true;
            } else {
                bigStep[axis] = false;
            }
        }

        prevAcceleration[axis] = setpointAcceleration;
        setpointAcceleration *= pidGetDT();
        setpointDeltaImpl[axis] = setpointSpeed * pidGetDT();

        const float ffBoostFactor = pidGetFfBoostFactor();
        float clip = 1.0f;
        float boostAmount = 0.0f;
        if (axis != FD_YAW && ffBoostFactor != 0.0f) {
            if (pidGetSpikeLimitInverse()) {
                clip = 1 / (1 + (setpointAcceleration * setpointAcceleration * pidGetSpikeLimitInverse()));
                clip *= clip;
            }
            // prevent kick-back spike at max deflection
            if (fabsf(rawSetpoint) < 0.95f * ffMaxRate[axis] || fabsf(setpointSpeed) > 3.0f * fabsf(prevSetpointSpeed[axis])) {
                boostAmount = ffBoostFactor * setpointAcceleration;
            }
            // no clip for first step inwards from max deflection
            if (fabsf(prevRawSetpoint[axis]) > 0.95f * ffMaxRate[axis] && fabsf(setpointSpeed) > 3.0f * fabsf(prevSetpointSpeed[axis])) {
                clip = 1.0f;
            }
        }

        prevSetpointSpeed[axis] = setpointSpeed;
        prevRawSetpoint[axis] = rawSetpoint;

        if (axis == FD_ROLL) {
            DEBUG_SET(DEBUG_FF_INTERPOLATED, 0, setpointDeltaImpl[axis] * 1000);
            DEBUG_SET(DEBUG_FF_INTERPOLATED, 1, boostAmount * 1000);
            DEBUG_SET(DEBUG_FF_INTERPOLATED, 2, bigStep[axis]);
            DEBUG_SET(DEBUG_FF_INTERPOLATED, 3, holdCount[axis]);
        }

        setpointDeltaImpl[axis] += boostAmount * clip;
        // first order filter FF
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
