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
#include "fc/runtime_config.h"
#include "fc/rc.h"
#include "osd/osd.h"
#include "common/printf.h"
#include "common/maths.h"
#include "math.h"
#include "build/debug.h"


PG_REGISTER_WITH_RESET_TEMPLATE(altholdConfig_t, altholdConfig, PG_ALTHOLD_CONFIG, 3);

PG_RESET_TEMPLATE(altholdConfig_t, altholdConfig,
    .velPidP = 30,
    .velPidD = 0,

    .altPidP = 75,
    .altPidI = 20,

    .minThrottle = 6,
    .maxThrottle = 65,
);


void simplePidInit(simplePid_s* simplePid, float min, float max, float kp, float kd, float ki)
{
    simplePid->max = max;
    simplePid->min = min;
    simplePid->kp = kp;
    simplePid->kd = kd;
    simplePid->ki = ki;
    simplePid->lastErr = 0;
    simplePid->integral = 0;
}

float simplePidCalculate(simplePid_s* simplePid, float dt, float targetValue, float measuredValue)
{
    float error = targetValue - measuredValue;

    float pOut = simplePid->kp * error;

    float iOut = simplePid->ki * simplePid->integral;

    simplePid->integral += error * dt;

    float derivative = (error - simplePid->lastErr) / dt;
    float dOut = simplePid->kd * derivative;

    float output = pOut + iOut + dOut;
    output = constrainf(output, simplePid->min, simplePid->max);

    simplePid->lastErr = error;
    return output;
}

static float getCurrentAltitude(altHoldState_s* altHoldState)
{
#ifdef USE_BARO
    if (sensors(SENSOR_BARO) && baroIsCalibrated()) {
        return 0.01f * baro.altitude;
    }
#endif
    float rawAltitude = 0.01f * getEstimatedAltitudeCm();
    if (ABS(altHoldState->smoothedAltitude) < 0.0f) {
        altHoldState->smoothedAltitude = rawAltitude;
    }
    float smoothFactor = 0.98f;
    altHoldState->smoothedAltitude = (1.0f - smoothFactor) * rawAltitude + smoothFactor * altHoldState->smoothedAltitude;
    return altHoldState->smoothedAltitude;
}

void altHoldReset(altHoldState_s* altHoldState)
{
    simplePidInit(&altHoldState->altPid, -50.0f, 50.0f,
                  0.01f * altholdConfig()->altPidP,
                  0.0f,
                  0.01f * altholdConfig()->altPidI);

    simplePidInit(&altHoldState->velPid, 0.0f, 1.0f,
                  0.01f * altholdConfig()->velPidP,
                  0.01f * altholdConfig()->velPidD,
                  0.0f);
    
    altHoldState->throttle = 0.0f;
    altHoldState->enterTime = millis();
    altHoldState->exitTime = 0;
    float externalVelocityEstimation = 0.01f * getEstimatedVario();
    altHoldState->startVelocityEstimationAccel = altHoldState->velocityEstimationAccel - externalVelocityEstimation;
    altHoldState->targetAltitude = getCurrentAltitude(altHoldState);
    altHoldState->smoothedAltitude = 0.0f;
}

void altHoldInit(altHoldState_s* altHoldState)
{
    altHoldState->altHoldEnabled = false;
    altHoldState->throttleFactor = 0.0f;
    altHoldState->velocityEstimationAccel = 0.0f;
    altHoldReset(altHoldState);
}

void altHoldProcessTransitions(altHoldState_s* altHoldState) {
    bool newAltHoldEnabled = FLIGHT_MODE(ALTHOLD_MODE);

    if (FLIGHT_MODE(GPS_RESCUE_MODE) || failsafeIsActive()) {
        newAltHoldEnabled = false;
    }

    if (newAltHoldEnabled && !altHoldState->altHoldEnabled)
    {
        altHoldReset(altHoldState);
    }
    if (!newAltHoldEnabled && altHoldState->altHoldEnabled) {
        altHoldState->exitTime = millis();
    }
    altHoldState->altHoldEnabled = newAltHoldEnabled;

    uint32_t currTime = millis();

    if (newAltHoldEnabled) {
        uint32_t timeSinceEnter = currTime - altHoldState->enterTime;
        if (timeSinceEnter < ALTHOLD_ENTER_PERIOD) {
            float delta = (float)timeSinceEnter / ALTHOLD_ENTER_PERIOD;
            altHoldState->throttleFactor = MAX(delta, altHoldState->throttleFactor);
        } else {
            altHoldState->throttleFactor = 1.0f;
        }
        return;
    }

    if (altHoldState->exitTime == 0) {
        altHoldState->throttleFactor = 0.0f;
        return;
    }

    uint32_t timeSinceExit = currTime - altHoldState->exitTime;
    if (timeSinceExit < ALTHOLD_MAX_ENTER_PERIOD) {
        float delta = (float)timeSinceExit / ALTHOLD_MAX_ENTER_PERIOD;
        altHoldState->throttleFactor = MIN(altHoldState->throttleFactor, 1.0f - delta);
        return;
    }

    altHoldState->throttleFactor = 0.0f;
}

void altHoldUpdate(altHoldState_s* altHoldState)
{
    altHoldProcessTransitions(altHoldState);

    if (altHoldState->altHoldEnabled) {
        float timeInterval = 1.0f / ALTHOLD_TASK_PERIOD;

        float measuredAltitude = getCurrentAltitude(altHoldState);

        t_fp_vector accelerationVector = {{
            acc.accADC[X],
            acc.accADC[Y],
            acc.accADC[Z]
        }};

        imuTransformVectorBodyToEarth(&accelerationVector);

        float measuredAccel = 9.8f * (accelerationVector.V.Z - acc.dev.acc_1G) / acc.dev.acc_1G;

        DEBUG_SET(DEBUG_ALTHOLD, 0, (int16_t)(measuredAccel * 100.0f));

        altHoldState->velocityEstimationAccel += measuredAccel * timeInterval;
        altHoldState->velocityEstimationAccel *= 0.999f;

        float currentVelocityEstimationAccel = altHoldState->velocityEstimationAccel - altHoldState->startVelocityEstimationAccel;
        DEBUG_SET(DEBUG_ALTHOLD, 1, (int16_t)(100.0f * currentVelocityEstimationAccel));

        altHoldState->measuredAltitude = measuredAltitude;
        altHoldState->measuredAccel = measuredAccel;

        float velocityTarget = simplePidCalculate(&altHoldState->altPid, timeInterval, altHoldState->targetAltitude, altHoldState->measuredAltitude);
        DEBUG_SET(DEBUG_ALTHOLD, 2, (int16_t)(100.0f * velocityTarget));

        float velPidForce = simplePidCalculate(&altHoldState->velPid, timeInterval, velocityTarget, currentVelocityEstimationAccel);

        float newThrottle = velPidForce;

        DEBUG_SET(DEBUG_ALTHOLD, 3, (int16_t)(100.0f * velPidForce));

        newThrottle = constrainf(newThrottle, 0.0f, 1.0f);
        newThrottle = scaleRangef(newThrottle, 0.0f, 1.0f, 0.01f * altholdConfig()->minThrottle, 0.01f * altholdConfig()->maxThrottle);

        altHoldState->throttle = newThrottle;
    }
}

altHoldState_s altHoldState;

void initAltHoldState(void) {
    altHoldInit(&altHoldState);
}

void updateAltHoldState(timeUs_t currentTimeUs) {
    altHoldUpdate(&altHoldState);

    UNUSED(currentTimeUs);
}

float getAltHoldThrottle(void) {
    return altHoldState.throttle;
}

float getAltHoldThrottleFactor(float currentThrottle) {
    if (!altHoldState.altHoldEnabled
        && altHoldState.exitTime != 0
        && (ABS(currentThrottle - altHoldState.throttle) < 0.15f)) {

        altHoldState.exitTime = 0;
    }
    return altHoldState.throttleFactor;
}

#endif
