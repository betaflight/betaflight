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

#include "platform.h"
#include "interpolated_setpoint.h"
#include "build/debug.h"
#include "fc/rc.h"
#include "common/maths.h"
#include "math.h"

#ifdef USE_INTERPOLATED_SP
static float projectedSetpoint[XYZ_AXIS_COUNT];
static float prevRawSetpoint[XYZ_AXIS_COUNT];
static float prevRawDeflection[XYZ_AXIS_COUNT];
static uint16_t interpolationSteps[XYZ_AXIS_COUNT];
static float setpointChangePerIteration[XYZ_AXIS_COUNT];
static float deflectionChangePerIteration[XYZ_AXIS_COUNT];
static float setpointReservoir[XYZ_AXIS_COUNT];
static float deflectionReservoir[XYZ_AXIS_COUNT];

// Configuration
static float ffMaxRateScale;
static float ffStickSpeedLimit;
static float ffMinSpread;

void interpolatedSpInit(const pidProfile_t *pidProfile) {
    ffMaxRateScale = pidProfile->ff_max_rate_limit * 0.01f;
    ffStickSpeedLimit = pidProfile->ff_stick_speed_limit * 0.0001f;
    ffMinSpread = pidProfile->ff_min_spread;
}

FAST_CODE_NOINLINE float interpolatedSpApply(int axis, float pidFrequency) {
    const float rawSetpoint = getRawSetpoint(axis);
    const float rawDeflection = getRawDeflection(axis);

    float pidSetpointDelta = 0.0f;
    if (rawDeflection != prevRawDeflection[axis]) {
        // get the number of interpolation steps either dynamically based on RX refresh rate
        // or manually based on ffMinSpread configuration property
        if (ffMinSpread) {
            interpolationSteps[axis] = (uint16_t) ((ffMinSpread + 1.0f) * 0.001f * pidFrequency);
        } else {
            interpolationSteps[axis] = (uint16_t) ((currentRxRefreshRate + 1000) * pidFrequency * 1e-6f + 0.5f);
        }

        // interpolate stick deflection
        deflectionReservoir[axis] += rawDeflection - prevRawDeflection[axis];
        deflectionChangePerIteration[axis] = deflectionReservoir[axis] / interpolationSteps[axis];
        const float projectedStickPos =
                rawDeflection + deflectionChangePerIteration[axis] * pidFrequency * ffStickSpeedLimit;
        projectedSetpoint[axis] = applyCurve(axis, projectedStickPos);
        prevRawDeflection[axis] = rawDeflection;

        // apply linear interpolation on setpoint
        setpointReservoir[axis] += rawSetpoint - prevRawSetpoint[axis];
        setpointChangePerIteration[axis] = setpointReservoir[axis] / interpolationSteps[axis];
        prevRawSetpoint[axis] = rawSetpoint;

        if (axis == FD_ROLL) {
            DEBUG_SET(DEBUG_FF_INTERPOLATED, 0, rawDeflection * 100);
            DEBUG_SET(DEBUG_FF_INTERPOLATED, 1, projectedStickPos * 100);
            DEBUG_SET(DEBUG_FF_INTERPOLATED, 2, projectedSetpoint[axis]);
        }
    }

    if (interpolationSteps[axis]) {
        pidSetpointDelta = setpointChangePerIteration[axis];

        interpolationSteps[axis]--;
        setpointReservoir[axis] -= setpointChangePerIteration[axis];
        deflectionReservoir[axis] -= deflectionChangePerIteration[axis];
    }

    return pidSetpointDelta;
}

FAST_CODE_NOINLINE float applyFFLimit(int axis, float value, float Kp, float currentPidSetpoint) {
    if (axis == FD_ROLL) {
        DEBUG_SET(DEBUG_FF_LIMIT, 0, value);
    }

    if (ffStickSpeedLimit) {
        const float limit = fabsf((projectedSetpoint[axis] - prevRawSetpoint[axis]) * Kp);
        value = constrainf(value, -limit, limit);
        if (axis == FD_ROLL) {
            DEBUG_SET(DEBUG_FF_INTERPOLATED, 3, projectedSetpoint[axis]);
        }
    }
    if (axis == FD_ROLL) {
        DEBUG_SET(DEBUG_FF_LIMIT, 1, value);
    }

    if (ffMaxRateScale) {
        const float maxRate = applyCurve(axis, 1.0f) * ffMaxRateScale;
        if (fabsf(currentPidSetpoint) <= maxRate) {
            const float limit = (maxRate - fabsf(currentPidSetpoint)) * Kp;
            value = constrainf(value, -limit, limit);
        } else {
            value = 0;
        }
    }
    if (axis == FD_ROLL) {
        DEBUG_SET(DEBUG_FF_LIMIT, 2, value);
    }
    return value;
}
#endif
