/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/utils.h"
#include "common/filter.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "fc/fc_rc.h"
#include "fc/fc_core.h"

#include "rx/rx.h"

#include "scheduler/scheduler.h"

#include "flight/pid.h"

#include "config/config_profile.h"
#include "config/config_master.h"
#include "config/feature.h"

static float setpointRate[3], rcDeflection[3], rcDeflectionAbs[3];
static float throttlePIDAttenuation;

float getSetpointRate(int axis) {
    return setpointRate[axis];
}

float getRcDeflection(int axis) {
    return rcDeflection[axis];
}

float getRcDeflectionAbs(int axis) {
    return rcDeflectionAbs[axis];
}

float getThrottlePIDAttenuation(void) {
    return throttlePIDAttenuation;
}

#define THROTTLE_LOOKUP_LENGTH 12
static int16_t lookupThrottleRC[THROTTLE_LOOKUP_LENGTH];    // lookup table for expo & mid THROTTLE

void generateThrottleCurve(void)
{
    uint8_t i;

    for (i = 0; i < THROTTLE_LOOKUP_LENGTH; i++) {
        int16_t tmp = 10 * i - currentControlRateProfile->thrMid8;
        uint8_t y = 1;
        if (tmp > 0)
            y = 100 - currentControlRateProfile->thrMid8;
        if (tmp < 0)
            y = currentControlRateProfile->thrMid8;
        lookupThrottleRC[i] = 10 * currentControlRateProfile->thrMid8 + tmp * (100 - currentControlRateProfile->thrExpo8 + (int32_t) currentControlRateProfile->thrExpo8 * (tmp * tmp) / (y * y)) / 10;
        lookupThrottleRC[i] = PWM_RANGE_MIN + (PWM_RANGE_MAX - PWM_RANGE_MIN) * lookupThrottleRC[i] / 1000; // [MINTHROTTLE;MAXTHROTTLE]
    }
}

int16_t rcLookupThrottle(int32_t tmp)
{
    const int32_t tmp2 = tmp / 100;
    // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]
    return lookupThrottleRC[tmp2] + (tmp - tmp2 * 100) * (lookupThrottleRC[tmp2 + 1] - lookupThrottleRC[tmp2]) / 100;
}

#define SETPOINT_RATE_LIMIT 1998.0f
#define RC_RATE_INCREMENTAL 14.54f

static void calculateSetpointRate(int axis)
{
    uint8_t rcExpo;
    float rcRate;
    if (axis != YAW) {
        rcExpo = currentControlRateProfile->rcExpo8;
        rcRate = currentControlRateProfile->rcRate8 / 100.0f;
    } else {
        rcExpo = currentControlRateProfile->rcYawExpo8;
        rcRate = currentControlRateProfile->rcYawRate8 / 100.0f;
    }
    if (rcRate > 2.0f) {
        rcRate += RC_RATE_INCREMENTAL * (rcRate - 2.0f);
    }

    float rcCommandf = rcCommand[axis] / 500.0f;
    rcDeflection[axis] = rcCommandf;
    const float rcCommandfAbs = ABS(rcCommandf);
    rcDeflectionAbs[axis] = rcCommandfAbs;

    if (rcExpo) {
        const float expof = rcExpo / 100.0f;
        rcCommandf = rcCommandf * power3(rcCommandfAbs) * expof + rcCommandf * (1-expof);
    }

    float angleRate = 200.0f * rcRate * rcCommandf;
    if (currentControlRateProfile->rates[axis]) {
        const float rcSuperfactor = 1.0f / (constrainf(1.0f - (rcCommandfAbs * (currentControlRateProfile->rates[axis] / 100.0f)), 0.01f, 1.00f));
        angleRate *= rcSuperfactor;
    }

    DEBUG_SET(DEBUG_ANGLERATE, axis, angleRate);

    setpointRate[axis] = constrainf(angleRate, -SETPOINT_RATE_LIMIT, SETPOINT_RATE_LIMIT); // Rate limit protection (deg/sec)
}

static void scaleRcCommandToFpvCamAngle(void) {
    //recalculate sin/cos only when rxConfig()->fpvCamAngleDegrees changed
    static uint8_t lastFpvCamAngleDegrees = 0;
    static float cosFactor = 1.0;
    static float sinFactor = 0.0;

    if (lastFpvCamAngleDegrees != rxConfig()->fpvCamAngleDegrees){
        lastFpvCamAngleDegrees = rxConfig()->fpvCamAngleDegrees;
        cosFactor = cos_approx(rxConfig()->fpvCamAngleDegrees * RAD);
        sinFactor = sin_approx(rxConfig()->fpvCamAngleDegrees * RAD);
    }

    float roll = setpointRate[ROLL];
    float yaw = setpointRate[YAW];
    setpointRate[ROLL] = constrainf(roll * cosFactor -  yaw * sinFactor, -SETPOINT_RATE_LIMIT, SETPOINT_RATE_LIMIT);
    setpointRate[YAW]  = constrainf(yaw  * cosFactor + roll * sinFactor, -SETPOINT_RATE_LIMIT, SETPOINT_RATE_LIMIT);
}

#define THROTTLE_BUFFER_MAX 20
#define THROTTLE_DELTA_MS 100

 static void checkForThrottleErrorResetState(uint16_t rxRefreshRate) {
    static int index;
    static int16_t rcCommandThrottlePrevious[THROTTLE_BUFFER_MAX];
    const int rxRefreshRateMs = rxRefreshRate / 1000;
    const int indexMax = constrain(THROTTLE_DELTA_MS / rxRefreshRateMs, 1, THROTTLE_BUFFER_MAX);
    const int16_t throttleVelocityThreshold = (feature(FEATURE_3D)) ? currentProfile->pidProfile.itermThrottleThreshold / 2 : currentProfile->pidProfile.itermThrottleThreshold;

    rcCommandThrottlePrevious[index++] = rcCommand[THROTTLE];
    if (index >= indexMax)
        index = 0;

    const int16_t rcCommandSpeed = rcCommand[THROTTLE] - rcCommandThrottlePrevious[index];

    if(ABS(rcCommandSpeed) > throttleVelocityThreshold)
        pidSetItermAccelerator(currentProfile->pidProfile.itermAcceleratorGain);
    else
        pidSetItermAccelerator(1.0f);
}

void processRcCommand(void)
{
    static int16_t lastCommand[4] = { 0, 0, 0, 0 };
    static int16_t deltaRC[4] = { 0, 0, 0, 0 };
    static int16_t factor, rcInterpolationFactor;
    static uint16_t currentRxRefreshRate;
    const uint8_t interpolationChannels = rxConfig()->rcInterpolationChannels + 2;
    uint16_t rxRefreshRate;
    bool readyToCalculateRate = false;
    uint8_t readyToCalculateRateAxisCnt = 0;

    if (isRXDataNew) {
        currentRxRefreshRate = constrain(getTaskDeltaTime(TASK_RX),1000,20000);
        if (currentProfile->pidProfile.itermAcceleratorGain > 1.0f)
            checkForThrottleErrorResetState(currentRxRefreshRate);
    }

    if (rxConfig()->rcInterpolation || flightModeFlags) {
         // Set RC refresh rate for sampling and channels to filter
        switch(rxConfig()->rcInterpolation) {
            case(RC_SMOOTHING_AUTO):
                rxRefreshRate = currentRxRefreshRate + 1000; // Add slight overhead to prevent ramps
                break;
            case(RC_SMOOTHING_MANUAL):
                rxRefreshRate = 1000 * rxConfig()->rcInterpolationInterval;
                break;
            case(RC_SMOOTHING_OFF):
            case(RC_SMOOTHING_DEFAULT):
            default:
                rxRefreshRate = rxGetRefreshRate();
        }

        if (isRXDataNew) {
            rcInterpolationFactor = rxRefreshRate / targetPidLooptime + 1;

            if (debugMode == DEBUG_RC_INTERPOLATION) {
                for(int axis = 0; axis < 2; axis++) debug[axis] = rcCommand[axis];
                debug[3] = rxRefreshRate;
            }

            for (int channel=ROLL; channel < interpolationChannels; channel++) {
                deltaRC[channel] = rcCommand[channel] -  (lastCommand[channel] - deltaRC[channel] * factor / rcInterpolationFactor);
                lastCommand[channel] = rcCommand[channel];
            }

            factor = rcInterpolationFactor - 1;
        } else {
            factor--;
        }

        // Interpolate steps of rcCommand
        if (factor > 0) {
            for (int channel=ROLL; channel < interpolationChannels; channel++) {
                rcCommand[channel] = lastCommand[channel] - deltaRC[channel] * factor/rcInterpolationFactor;
                readyToCalculateRateAxisCnt = MAX(channel,FD_YAW); // throttle channel doesn't require rate calculation
                readyToCalculateRate = true;
            }
        } else {
            factor = 0;
        }
    } else {
        factor = 0; // reset factor in case of level modes flip flopping
    }

    if (readyToCalculateRate || isRXDataNew) {
        if (isRXDataNew)
            readyToCalculateRateAxisCnt = FD_YAW;

        for (int axis = 0; axis <= readyToCalculateRateAxisCnt; axis++)
            calculateSetpointRate(axis);

        // Scaling of AngleRate to camera angle (Mixing Roll and Yaw)
        if (rxConfig()->fpvCamAngleDegrees && IS_RC_MODE_ACTIVE(BOXFPVANGLEMIX) && !FLIGHT_MODE(HEADFREE_MODE))
            scaleRcCommandToFpvCamAngle();

        isRXDataNew = false;
    }
}

void updateRcCommands(void)
{
    // PITCH & ROLL only dynamic PID adjustment,  depending on throttle value
    int32_t prop;
    if (rcData[THROTTLE] < currentControlRateProfile->tpa_breakpoint) {
        prop = 100;
        throttlePIDAttenuation = 1.0f;
    } else {
        if (rcData[THROTTLE] < 2000) {
            prop = 100 - (uint16_t)currentControlRateProfile->dynThrPID * (rcData[THROTTLE] - currentControlRateProfile->tpa_breakpoint) / (2000 - currentControlRateProfile->tpa_breakpoint);
        } else {
            prop = 100 - currentControlRateProfile->dynThrPID;
        }
        throttlePIDAttenuation = prop / 100.0f;
    }

    for (int axis = 0; axis < 3; axis++) {
        // non coupled PID reduction scaler used in PID controller 1 and PID controller 2.

        int32_t tmp = MIN(ABS(rcData[axis] - rxConfig()->midrc), 500);
        if (axis == ROLL || axis == PITCH) {
            if (tmp > rcControlsConfig()->deadband) {
                tmp -= rcControlsConfig()->deadband;
            } else {
                tmp = 0;
            }
            rcCommand[axis] = tmp;
        } else {
            if (tmp > rcControlsConfig()->yaw_deadband) {
                tmp -= rcControlsConfig()->yaw_deadband;
            } else {
                tmp = 0;
            }
            rcCommand[axis] = tmp * -rcControlsConfig()->yaw_control_direction;
        }
        if (rcData[axis] < rxConfig()->midrc) {
            rcCommand[axis] = -rcCommand[axis];
        }
    }

    int32_t tmp;
    if (feature(FEATURE_3D)) {
        tmp = constrain(rcData[THROTTLE], PWM_RANGE_MIN, PWM_RANGE_MAX);
        tmp = (uint32_t)(tmp - PWM_RANGE_MIN);
    } else {
        tmp = constrain(rcData[THROTTLE], rxConfig()->mincheck, PWM_RANGE_MAX);
        tmp = (uint32_t)(tmp - rxConfig()->mincheck) * PWM_RANGE_MIN / (PWM_RANGE_MAX - rxConfig()->mincheck);
    }

    rcCommand[THROTTLE] = rcLookupThrottle(tmp);

    if (feature(FEATURE_3D) && IS_RC_MODE_ACTIVE(BOX3DDISABLESWITCH) && !failsafeIsActive()) {
        fix12_t throttleScaler = qConstruct(rcCommand[THROTTLE] - 1000, 1000);
        rcCommand[THROTTLE] = rxConfig()->midrc + qMultiply(throttleScaler, PWM_RANGE_MAX - rxConfig()->midrc);
    }

    if (FLIGHT_MODE(HEADFREE_MODE)) {
        const float radDiff = degreesToRadians(DECIDEGREES_TO_DEGREES(attitude.values.yaw) - headFreeModeHold);
        const float cosDiff = cos_approx(radDiff);
        const float sinDiff = sin_approx(radDiff);
        const int16_t rcCommand_PITCH = rcCommand[PITCH] * cosDiff + rcCommand[ROLL] * sinDiff;
        rcCommand[ROLL] = rcCommand[ROLL] * cosDiff - rcCommand[PITCH] * sinDiff;
        rcCommand[PITCH] = rcCommand_PITCH;
    }
}

void resetYawAxis(void) {
    rcCommand[YAW] = 0;
    setpointRate[YAW] = 0;
}
