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

static float projectedSetpoint[XYZ_AXIS_COUNT];
static float prevSetpointSpeed[XYZ_AXIS_COUNT];
static float prevRawSetpoint[XYZ_AXIS_COUNT];
static float prevRawDeflection[XYZ_AXIS_COUNT];
static uint16_t interpolationSteps[XYZ_AXIS_COUNT];
static float setpointChangePerIteration[XYZ_AXIS_COUNT];
static float deflectionChangePerIteration[XYZ_AXIS_COUNT];
static float setpointReservoir[XYZ_AXIS_COUNT];
static float deflectionReservoir[XYZ_AXIS_COUNT];

// Configuration
static float ffLookaheadLimit;
static float ffSpread;
static float ffMaxRateLimit[XYZ_AXIS_COUNT];
static float ffMaxRate[XYZ_AXIS_COUNT];

void interpolatedSpInit(const pidProfile_t *pidProfile) {
    const float ffMaxRateScale = pidProfile->ff_max_rate_limit * 0.01f;
    ffLookaheadLimit = pidProfile->ff_lookahead_limit * 0.0001f;
    ffSpread = pidProfile->ff_spread;
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        ffMaxRate[i] = applyCurve(i, 1.0f);
        ffMaxRateLimit[i] = ffMaxRate[i] * ffMaxRateScale;
    }
}

FAST_CODE_NOINLINE float interpolatedSpApply(int axis, float pidFrequency, bool newRcFrame) {
    const float rawSetpoint = getRawSetpoint(axis);
    const float rawDeflection = getRawDeflection(axis);

    float pidSetpointDelta = 0.0f;
    static int iterationsSinceLastUpdate[XYZ_AXIS_COUNT];
    if (newRcFrame) {

        setpointReservoir[axis] -= iterationsSinceLastUpdate[axis] * setpointChangePerIteration[axis];
        deflectionReservoir[axis] -= iterationsSinceLastUpdate[axis] * deflectionChangePerIteration[axis];
        iterationsSinceLastUpdate[axis] = 0;

        // get the number of interpolation steps either dynamically based on RX refresh rate
        // or manually based on ffSpread configuration property
        if (ffSpread) {
            interpolationSteps[axis] = (uint16_t) ((ffSpread + 1.0f) * 0.001f * pidFrequency);
        } else {
            interpolationSteps[axis] = (uint16_t) ((currentRxRefreshRate + 1000) * pidFrequency * 1e-6f + 0.5f);
        }

        // interpolate stick deflection
        deflectionReservoir[axis] += rawDeflection - prevRawDeflection[axis];
        deflectionChangePerIteration[axis] = deflectionReservoir[axis] / interpolationSteps[axis];
        const float projectedStickPos =
            rawDeflection + deflectionChangePerIteration[axis] * pidFrequency * ffLookaheadLimit;
        projectedSetpoint[axis] = applyCurve(axis, projectedStickPos);
        prevRawDeflection[axis] = rawDeflection;

        // apply linear interpolation on setpoint
        setpointReservoir[axis] += rawSetpoint - prevRawSetpoint[axis];
        const float ffBoostFactor = pidGetFfBoostFactor();
        if (ffBoostFactor != 0.0f) {
            const float speed = rawSetpoint - prevRawSetpoint[axis];
            if (fabsf(rawSetpoint) < 0.95f * ffMaxRate[axis] || fabsf(3.0f * speed) > fabsf(prevSetpointSpeed[axis])) {
                const float setpointAcc = speed - prevSetpointSpeed[axis];
                setpointReservoir[axis] += ffBoostFactor * setpointAcc;
            }
            prevSetpointSpeed[axis] = speed;
        }

        setpointChangePerIteration[axis] = setpointReservoir[axis] / interpolationSteps[axis];

        prevRawSetpoint[axis] = rawSetpoint;
        
        if (axis == FD_ROLL) {
            DEBUG_SET(DEBUG_FF_INTERPOLATED, 0, rawDeflection * 100);
            DEBUG_SET(DEBUG_FF_INTERPOLATED, 1, projectedStickPos * 100);
            DEBUG_SET(DEBUG_FF_INTERPOLATED, 2, projectedSetpoint[axis]);
        }
    }

    if (iterationsSinceLastUpdate[axis] < interpolationSteps[axis]) {
        iterationsSinceLastUpdate[axis]++;
        pidSetpointDelta = setpointChangePerIteration[axis];
    }

    return pidSetpointDelta;
}

FAST_CODE_NOINLINE float applyFfLimit(int axis, float value, float Kp, float currentPidSetpoint) {
    if (axis == FD_ROLL) {
        DEBUG_SET(DEBUG_FF_LIMIT, 0, value);
    }

    if (ffLookaheadLimit) {
        const float limit = fabsf((projectedSetpoint[axis] - prevRawSetpoint[axis]) * Kp);
        value = constrainf(value, -limit, limit);
        if (axis == FD_ROLL) {
            DEBUG_SET(DEBUG_FF_INTERPOLATED, 3, projectedSetpoint[axis]);
        }
    }
    if (axis == FD_ROLL) {
        DEBUG_SET(DEBUG_FF_LIMIT, 1, value);
    }

    if (ffMaxRateLimit[axis]) {
        if (fabsf(currentPidSetpoint) <= ffMaxRateLimit[axis]) {
            value = constrainf(value, (-ffMaxRateLimit[axis] - currentPidSetpoint) * Kp, (ffMaxRateLimit[axis] - currentPidSetpoint) * Kp);
        } else {
            value = 0;
        }
    }
    if (axis == FD_ROLL) {
        DEBUG_SET(DEBUG_FF_LIMIT, 2, value);
    }
    return value;
}

bool shouldApplyFfLimits(int axis)
{
    return ffLookaheadLimit != 0.0f || ffMaxRateLimit[axis] != 0.0f;
}


#endif
