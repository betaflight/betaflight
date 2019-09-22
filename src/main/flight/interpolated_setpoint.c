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

static float setpointDeltaImpl[XYZ_AXIS_COUNT];
static float prevSetpointDeltaImpl[XYZ_AXIS_COUNT];
static float setpointDelta[XYZ_AXIS_COUNT];


static float prevSetpointSpeed[XYZ_AXIS_COUNT];
static float prevRawSetpoint[XYZ_AXIS_COUNT];
static float prevSetpointAcceleration[XYZ_AXIS_COUNT];


// Configuration
static float ffMaxRateLimit[XYZ_AXIS_COUNT];
static float ffMaxRate[XYZ_AXIS_COUNT];

void interpolatedSpInit(const pidProfile_t *pidProfile) {
    const float ffMaxRateScale = pidProfile->ff_max_rate_limit * 0.01f;
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        ffMaxRate[i] = applyCurve(i, 1.0f);
        ffMaxRateLimit[i] = ffMaxRate[i] * ffMaxRateScale;
    }
}

FAST_CODE_NOINLINE float interpolatedSpApply(int axis, bool newRcFrame, ffInterpolationType_t type) {

    if (newRcFrame) {
        float rawSetpoint = getRawSetpoint(axis); 
        
        const float rxInterval = currentRxRefreshRate * 1e-6f;
        const float rxRate = 1.0f / rxInterval;

        const float setpointSpeed = (rawSetpoint - prevRawSetpoint[axis]) * rxRate;
        const float setpointAcceleration = (setpointSpeed - prevSetpointSpeed[axis]) * pidGetDT();

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
        }
        prevSetpointSpeed[axis] = setpointSpeed;
        prevSetpointAcceleration[axis] = setpointAcceleration;
        prevRawSetpoint[axis] = rawSetpoint;
        if (axis == FD_ROLL) {
            DEBUG_SET(DEBUG_FF_INTERPOLATED, 0, setpointDeltaImpl[axis] * 1000);
            DEBUG_SET(DEBUG_FF_INTERPOLATED, 1, boostAmount * 1000);
            DEBUG_SET(DEBUG_FF_INTERPOLATED, 2, boostAmount * clip * 1000);
            DEBUG_SET(DEBUG_FF_INTERPOLATED, 3, clip * 100);
        }
        setpointDeltaImpl[axis] += boostAmount * clip;
        if (type == FF_INTERPOLATE_ON) {
            setpointDelta[axis] = setpointDeltaImpl[axis];
        } else {
            setpointDelta[axis] = 0.5f * (setpointDeltaImpl[axis] + prevSetpointDeltaImpl[axis]);
            prevSetpointDeltaImpl[axis] = setpointDeltaImpl[axis];
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
