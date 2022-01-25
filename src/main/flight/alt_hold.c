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


PG_REGISTER_WITH_RESET_TEMPLATE(altholdConfig_t, altholdConfig, PG_ALTHOLD_CONFIG, 0);

PG_RESET_TEMPLATE(altholdConfig_t, altholdConfig,
    .velPidP = 30,
    .velPidD = 0,
    .velPidI = 0,

    .altPidP = 50,
    .altPidD = 0,
    .altPidI = 5,

    .minThrottle = 6,
    .maxThrottle = 65,
);


typedef struct {
    float dt;
    float max;
    float min;
    float kp;
    float kd;
    float ki;
    float preErr;
    float integral;
} simplePid_s;

void simplePidInit(simplePid_s* simplePid, float dt, float max, float min, float kp, float kd, float ki)
{
    simplePid->dt = dt;
    simplePid->max = max;
    simplePid->min = min;
    simplePid->kp = kp;
    simplePid->kd = kd;
    simplePid->ki = ki;
    simplePid->preErr = 0;
    simplePid->integral = 0;
}

float simplePidCalculate(simplePid_s* simplePid, float targetValue, float measuredValue)
{
    float error = targetValue - measuredValue;

    float pOut = simplePid->kp * error;

    simplePid->integral += error * simplePid->dt;

    float iOut = simplePid->ki * simplePid->integral;

    float derivative = (error - simplePid->preErr) / simplePid->dt;
    float dOut = simplePid->kd * derivative;

    float output = pOut + iOut + dOut;
    if (output > simplePid->max) {
        output = simplePid->max;
    } else if (output < simplePid->min) {
        output = simplePid->min;
    }

    simplePid->preErr = error;
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
    simplePidInit(&altHoldState->altPid, 0.1, 50.0, -50.0,
                  0.01 * altholdConfig()->altPidP,
                  0.01 * altholdConfig()->altPidD,
                  0.01 * altholdConfig()->altPidI);

    simplePidInit(&altHoldState->velPid, 0.1, 1.0, 0.0,
                  0.01 * altholdConfig()->velPidP,
                  0.01 * altholdConfig()->velPidD,
                  0.01 * altholdConfig()->velPidI);

    altHoldState->throttle = 0.0;
    altHoldState->startVelocityEstimationAccel = altHoldState->velocityEstimationAccel;
    altHoldState->targetAltitude = (float)(0.01 * getEstimatedAltitudeCm());
}

void altHoldInit(altHoldState_s* altHoldState)
{
    altHoldState->prevAltHoldModeEnabled = false;
    altHoldState->velocityEstimationAccel = 0.0;
    altHoldReset(altHoldState);
}


// Rotate a vector *v by the euler angles defined by the 3-vector *delta.
void rotateV(struct fp_vector *v, fp_angles_t *delta)
{
    struct fp_vector v_tmp = *v;

    fp_rotationMatrix_t rotationMatrix;

    buildRotationMatrix(delta, &rotationMatrix);

    applyMatrixRotation((float *)&v_tmp, &rotationMatrix);

    v->X = v_tmp.X;
    v->Y = v_tmp.Y;
    v->Z = v_tmp.Z;
}

void altHoldUpdate(altHoldState_s* altHoldState)
{
    bool altHoldModeEnabled = FLIGHT_MODE(ALTHOLD_MODE);

    if (altHoldModeEnabled && !altHoldState->prevAltHoldModeEnabled)
    {
        altHoldReset(altHoldState);
    }
    altHoldState->prevAltHoldModeEnabled = altHoldModeEnabled;


    float measuredAltitude = (float)(0.01 * getEstimatedAltitudeCm());

    t_fp_vector_def accelerationVector = {
        acc.accADC[X],
        acc.accADC[Y],
        acc.accADC[Z]
    };

    fp_angles_t attitudeAngles = {{
        DECIDEGREES_TO_RADIANS(-attitude.values.roll),
        DECIDEGREES_TO_RADIANS(-attitude.values.pitch),
        DECIDEGREES_TO_RADIANS(attitude.values.yaw)
    }};

    rotateV(&accelerationVector, &attitudeAngles);

    float measuredAccel = 0.01f * (accelerationVector.Z - acc.dev.acc_1G);

    DEBUG_SET(DEBUG_ALTHOLD, 0, (int16_t)(measuredAccel * 100.0f));

    altHoldState->velocityEstimationAccel += measuredAccel * 0.01f;
    altHoldState->velocityEstimationAccel *= 0.999f;

    float currentVelocityEstimationAccel = altHoldState->velocityEstimationAccel - altHoldState->startVelocityEstimationAccel;
    DEBUG_SET(DEBUG_ALTHOLD, 1, (int16_t)(100.0 * currentVelocityEstimationAccel));

    altHoldState->measuredAltitude = measuredAltitude;
    altHoldState->measuredAccel = measuredAccel;

    float velocityTarget = simplePidCalculate(&altHoldState->altPid, altHoldState->targetAltitude, altHoldState->measuredAltitude);
    DEBUG_SET(DEBUG_ALTHOLD, 2, (int16_t)(100.0 * velocityTarget));

    float velPidForce = simplePidCalculate(&altHoldState->velPid, velocityTarget, currentVelocityEstimationAccel);
    
    float newThrottle = velPidForce;

    DEBUG_SET(DEBUG_ALTHOLD, 3, (int16_t)(100.0 * velPidForce));

    newThrottle = constrainf(newThrottle, 0.0f, 1.0f);
    newThrottle = scaleRangef(newThrottle, 0.0f, 1.0f, 0.01 * altholdConfig()->minThrottle, 0.01 * altholdConfig()->maxThrottle);

    if (!altHoldModeEnabled) {
        newThrottle = 0.0;
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
