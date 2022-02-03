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

#include "flight/position.h"
#include "flight/imu.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "config/config.h"
#include "fc/runtime_config.h"
#include "osd/osd.h"
#include "common/printf.h"
#include "common/maths.h"
#include "math.h"
#include "build/debug.h"


PG_REGISTER_WITH_RESET_TEMPLATE(altholdConfig_t, altholdConfig, PG_ALTHOLD_CONFIG, 1);

PG_RESET_TEMPLATE(altholdConfig_t, altholdConfig,
    .velPidP = 30,
    .velPidD = 0,

    .altPidP = 75,
    .altPidI = 20,

    .minThrottle = 6,
    .maxThrottle = 65,
);


typedef struct {
    float max;
    float min;
    float kp;
    float kd;
    float ki;
    float lastErr;
    float integral;
} simplePid_s;

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

typedef struct {
    simplePid_s altPid;
    simplePid_s velPid;
    float throttle;
    float targetAltitude;
    float measuredAltitude;
    float measuredAccel;
    float velocityEstimationAccel;  // based on acceleration
    float startVelocityEstimationAccel;
    bool prevAltHoldModeEnabled;
} altHoldState_s;

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
    altHoldState->startVelocityEstimationAccel = altHoldState->velocityEstimationAccel;
    altHoldState->targetAltitude = (float)(0.01f * getEstimatedAltitudeCm());
}

void altHoldInit(altHoldState_s* altHoldState)
{
    altHoldState->prevAltHoldModeEnabled = false;
    altHoldState->velocityEstimationAccel = 0.0f;
    altHoldReset(altHoldState);
}


void altHoldUpdate(altHoldState_s* altHoldState)
{
    bool altHoldModeEnabled = FLIGHT_MODE(ALTHOLD_MODE);

    if (altHoldModeEnabled && !altHoldState->prevAltHoldModeEnabled)
    {
        altHoldReset(altHoldState);
    }
    altHoldState->prevAltHoldModeEnabled = altHoldModeEnabled;


    float measuredAltitude = (float)(0.01f * getEstimatedAltitudeCm());

    t_fp_vector accelerationVector = {{
        acc.accADC[X],
        acc.accADC[Y],
        acc.accADC[Z]
    }};

    imuTransformVectorBodyToEarth(&accelerationVector);

    float measuredAccel = 9.8f * (accelerationVector.V.Z - acc.dev.acc_1G) / acc.dev.acc_1G;

    DEBUG_SET(DEBUG_ALTHOLD, 0, (int16_t)(measuredAccel * 100.0f));

    altHoldState->velocityEstimationAccel += measuredAccel * 0.01f;
    altHoldState->velocityEstimationAccel *= 0.999f;

    float currentVelocityEstimationAccel = altHoldState->velocityEstimationAccel - altHoldState->startVelocityEstimationAccel;
    DEBUG_SET(DEBUG_ALTHOLD, 1, (int16_t)(100.0f * currentVelocityEstimationAccel));

    altHoldState->measuredAltitude = measuredAltitude;
    altHoldState->measuredAccel = measuredAccel;

    float velocityTarget = simplePidCalculate(&altHoldState->altPid, 0.01f, altHoldState->targetAltitude, altHoldState->measuredAltitude);
    DEBUG_SET(DEBUG_ALTHOLD, 2, (int16_t)(100.0f * velocityTarget));

    float velPidForce = simplePidCalculate(&altHoldState->velPid, 0.01f, velocityTarget, currentVelocityEstimationAccel);
    
    float newThrottle = velPidForce;

    DEBUG_SET(DEBUG_ALTHOLD, 3, (int16_t)(100.0f * velPidForce));

    newThrottle = constrainf(newThrottle, 0.0f, 1.0f);
    newThrottle = scaleRangef(newThrottle, 0.0f, 1.0f, 0.01f * altholdConfig()->minThrottle, 0.01f * altholdConfig()->maxThrottle);

    if (!altHoldModeEnabled) {
        newThrottle = 0.0f;
    }

    altHoldState->throttle = newThrottle;
}


altHoldState_s altHoldState;

void initAltHoldState(void) {
    altHoldInit(&altHoldState);
}

void updateAltHoldState(timeUs_t currentTimeUs) {
    altHoldUpdate(&altHoldState);
    (void)currentTimeUs;
}

float getAltHoldThrottle(void) {
    return altHoldState.throttle;
}

#endif
