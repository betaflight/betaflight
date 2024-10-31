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

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#include "platform.h"
#include "build/debug.h"
#include "common/filter.h"
#include "common/maths.h"
#include "common/vector.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/pos_hold.h"
#include "flight/position.h"
#include "rx/rx.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"

#include "autopilot.h"

#define ALTITUDE_P_SCALE  0.01f
#define ALTITUDE_I_SCALE  0.003f
#define ALTITUDE_D_SCALE  0.01f
#define ALTITUDE_F_SCALE  0.01f
#define POSITION_P_SCALE  0.0012f
#define POSITION_I_SCALE  0.0001f
#define POSITION_D_SCALE  0.0015f
#define POSITION_A_SCALE  0.0015f
#define UPSAMPLING_CUTOFF 5.0f

static pidCoefficient_t altitudePidCoeffs;
static pidCoefficient_t positionPidCoeffs;

static float altitudeI = 0.0f;
static float throttleOut = 0.0f;

typedef struct {
    bool isStarting;
    float distance;
    float previousDistance;
    float previousVelocity;
    float integral;
    float pidSum;
    pt1Filter_t velocityLpf;
    pt1Filter_t accelerationLpf;
} earthFrame_t;

typedef struct {
    float gpsDataIntervalS;
    float gpsDataFreqHz;
    float sanityCheckDistance;
    float peakInitialGroundspeed;
    float lpfCutoff;
    float pt1Gain;
    bool sticksActive;
    float pidSum[2];
    pt3Filter_t upsample[2];
    earthFrame_t direction[2];
} posHoldState;

typedef enum {
    NORTH_SOUTH = 0,
    EAST_WEST
} axisEF_t;

static posHoldState posHold = {
    .gpsDataIntervalS = 0.1f,
    .gpsDataFreqHz = 10.0f,
    .sanityCheckDistance = 1000.0f,
    .peakInitialGroundspeed = 0.0f,
    .lpfCutoff = 1.0f,
    .pt1Gain = 1.0f,
    .sticksActive = false,
    .pidSum = { 0.0f, 0.0f },
    .upsample = { {0}, {0} },
    .direction = { {0} }
};

earthFrame_t northSouth;
earthFrame_t eastWest;

static gpsLocation_t currentTargetLocation = {0, 0, 0};
float autopilotAngle[ANGLE_INDEX_COUNT];

void resetPositionControlParams(earthFrame_t *latLong) {
    // at the start, and while sticks are moving
    latLong->previousDistance = 0.0f;
    latLong->previousVelocity = 0.0f;
    latLong->pidSum = 0.0f;
    // Clear accumulation in filters
    pt1FilterInit(&latLong->velocityLpf, posHold.pt1Gain);
    pt1FilterInit(&latLong->accelerationLpf, posHold.pt1Gain);
    // Initiate starting behaviour
    latLong->isStarting = true;
}

void resetPositionControl(gpsLocation_t initialTargetLocation) { // set only at the start frmo pos_hold.c
    currentTargetLocation = initialTargetLocation;
    resetPositionControlParams(&posHold.direction[NORTH_SOUTH]);
    resetPositionControlParams(&posHold.direction[EAST_WEST]);
    posHold.peakInitialGroundspeed = 0.0f;
    posHold.direction[NORTH_SOUTH].integral = 0.0f;
    posHold.direction[EAST_WEST].integral = 0.0f;
    posHold.sanityCheckDistance = gpsSol.groundSpeed > 1000 ? gpsSol.groundSpeed : 1000.0f;
}

void autopilotInit(const autopilotConfig_t *config)
{
    northSouth = posHold.direction[NORTH_SOUTH];
    eastWest = posHold.direction[EAST_WEST];
    altitudePidCoeffs.Kp = config->altitude_P * ALTITUDE_P_SCALE;
    altitudePidCoeffs.Ki = config->altitude_I * ALTITUDE_I_SCALE;
    altitudePidCoeffs.Kd = config->altitude_D * ALTITUDE_D_SCALE;
    altitudePidCoeffs.Kf = config->altitude_F * ALTITUDE_F_SCALE;
    positionPidCoeffs.Kp = config->position_P * POSITION_P_SCALE;
    positionPidCoeffs.Ki = config->position_I * POSITION_I_SCALE;
    positionPidCoeffs.Kd = config->position_D * POSITION_D_SCALE;
    positionPidCoeffs.Kf = config->position_A * POSITION_A_SCALE; // Kf used for acceleration
    // approximate filter gain
    posHold.lpfCutoff = config->position_cutoff * 0.01f;
    posHold.pt1Gain = pt1FilterGain(posHold.lpfCutoff, 0.1f); // assume 10Hz GPS connection at start
    float upsampleCutoff = pt3FilterGain(UPSAMPLING_CUTOFF, 0.01f); // 5Hz, assuming 100Hz task rate
    pt3FilterInit(&posHold.upsample[AI_ROLL], upsampleCutoff);
    pt3FilterInit(&posHold.upsample[AI_PITCH], upsampleCutoff);
    // initialise filters
// Reset parameters for both NS and EW
    resetPositionControlParams(&posHold.direction[NORTH_SOUTH]);
    resetPositionControlParams(&posHold.direction[EAST_WEST]);
}

void resetAltitudeControl (void) {
    altitudeI = 0.0f;
}

void altitudeControl(float targetAltitudeCm, float taskIntervalS, float verticalVelocity, float targetAltitudeStep) {

    const float altitudeErrorCm = targetAltitudeCm - getAltitudeCm();
    const float altitudeP = altitudeErrorCm * altitudePidCoeffs.Kp;

    // reduce the iTerm gain for errors greater than 200cm (2m), otherwise it winds up too much
    const float itermRelax = (fabsf(altitudeErrorCm) < 200.0f) ? 1.0f : 0.1f;
    altitudeI += altitudeErrorCm * altitudePidCoeffs.Ki * itermRelax * taskIntervalS;
    // limit iTerm to not more than 200 throttle units
    altitudeI = constrainf(altitudeI, -200.0f, 200.0f);

    const float altitudeD = verticalVelocity * altitudePidCoeffs.Kd;

    const float altitudeF = targetAltitudeStep * altitudePidCoeffs.Kf;

    const float hoverOffset = autopilotConfig()->hover_throttle - PWM_RANGE_MIN;
    float throttleOffset = altitudeP + altitudeI - altitudeD + altitudeF + hoverOffset;

    const float tiltMultiplier = 1.0f / fmaxf(getCosTiltAngle(), 0.5f);
    // 1 = flat, 1.3 at 40 degrees, 1.56 at 50 deg, max 2.0 at 60 degrees or higher
    // note: the default limit of Angle Mode is 60 degrees

    throttleOffset *= tiltMultiplier;

    float newThrottle = PWM_RANGE_MIN + throttleOffset;
    newThrottle = constrainf(newThrottle, autopilotConfig()->throttle_min, autopilotConfig()->throttle_max);
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 0, lrintf(newThrottle)); // normal range 1000-2000 but is before constraint

    newThrottle = scaleRangef(newThrottle, MAX(rxConfig()->mincheck, PWM_RANGE_MIN), PWM_RANGE_MAX, 0.0f, 1.0f);

    throttleOut = constrainf(newThrottle, 0.0f, 1.0f);

    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 1, lrintf(tiltMultiplier * 100));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 3, lrintf(targetAltitudeCm));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 4, lrintf(altitudeP));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 5, lrintf(altitudeI));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 6, lrintf(-altitudeD));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 7, lrintf(altitudeF));
}

void setSticksActiveStatus(bool areSticksActive)
{
    posHold.sticksActive = areSticksActive;
}

void setTargetLocation(gpsLocation_t newTargetLocation) {
    currentTargetLocation = newTargetLocation;
}

bool positionControl(void) {

    if (!STATE(GPS_FIX)) {
        return false; // cannot proceed; this could happen mid-flight, e.g. early after takeoff without a fix
    }

    if (
    #ifdef USE_MAG
        !compassIsHealthy() && // if compass is OK, don't worry about GPS; this is not likely to change in-flight
    #endif
        (!allowPosHoldWithoutMag() || !canUseGPSHeading) // no compass, check if GPS heading is OK, which changes during the flight
    ) {
        return false;
    }

    if (!wasThrottleRaised()) {
        return false;
    }

    // note: returning false dispays POS_HOLD_FAIL warning in OSD

    if (isNewGPSDataAvailable()) {
        posHold.gpsDataIntervalS = getGpsDataIntervalSeconds(); // interval for current GPS data value 0.01s to 1.0s
        posHold.gpsDataFreqHz = 1.0f / posHold.gpsDataIntervalS;
        if (posHold.sticksActive) {
            // if a Position Hold deadband is set, and sticks are outside deadband, allow pilot control in angle mode
            resetPositionControlParams(&posHold.direction[NORTH_SOUTH]);
            resetPositionControlParams(&posHold.direction[EAST_WEST]);
            posHold.pidSum[AI_ROLL] = 0.0f;
            posHold.pidSum[AI_PITCH] = 0.0f;
        } else {
            // first get xy distances from current location (gpsSol.llh) to target location
            vector2_t gpsDistance;
            GPS_distances(&gpsSol.llh, &currentTargetLocation, &gpsDistance.y, &gpsDistance.x); // Y is north, X is south

            posHold.direction[NORTH_SOUTH].distance = gpsDistance.y;
            posHold.direction[EAST_WEST].distance = gpsDistance.x;
            float distanceCm = vector2Norm(&gpsDistance);

            posHold.pt1Gain = pt1FilterGain(posHold.lpfCutoff, posHold.gpsDataIntervalS);
            const float leak = 1.0f - 0.4f * posHold.gpsDataIntervalS; // gpsDataIntervalS is not more than 1.0s

            // ** Sanity check **
            // primarily to detect flyaway from no Mag or badly oriented Mag
            // must accept some overshoot at the start, especially if entering at high speed
            if (distanceCm > posHold.sanityCheckDistance) {
                return false;
            }

            // static float prevPidDASquared = 0.0f; // if we limit DA on true vector length

            for (int i = 0; i < 2; i++) {
                earthFrame_t *latLong = &posHold.direction[i];

                // separate PID controllers for latitude (NorthSouth or NS) and longitude (EastWest or EW)

                // ** P **
                float pidP = latLong->distance * positionPidCoeffs.Kp;

                // ** I **
                if (!latLong->isStarting){
                    // only accumulate iTerm after completing the start phase
                    // perhaps need a timeout on the start phase ?
                    latLong->integral += latLong->distance * posHold.gpsDataIntervalS;
                } else {
                    // while moving sticks, slowly leak iTerm away, approx 2s time constant
                    latLong->integral *= leak;
                }
                float pidI = latLong->integral * positionPidCoeffs.Ki;

                // ** D ** //
                // Velocity derived from GPS position works better than module supplied GPS Speed and Heading information
                float velocity = (latLong->distance - latLong->previousDistance) * posHold.gpsDataFreqHz; // cm/s, minimum step 11.1 cm/s
                latLong->previousDistance = latLong->distance;
                pt1FilterUpdateCutoff(&latLong->velocityLpf, posHold.pt1Gain);
                velocity = pt1FilterApply(&latLong->velocityLpf, velocity);
                float pidD = velocity * positionPidCoeffs.Kd;

                float acceleration = (velocity - latLong->previousVelocity) * posHold.gpsDataFreqHz;
                latLong->previousVelocity = velocity;
                pt1FilterUpdateCutoff(&latLong->accelerationLpf, posHold.pt1Gain);
                acceleration = pt1FilterApply(&latLong->accelerationLpf, acceleration);
                float pidA = acceleration * positionPidCoeffs.Kd;

                // limit sum of D and A because otherwise can be too aggressive when starting at speed
                const float maxDAAngle = 35.0f; // limit in degrees; arbitrary.  20 is a bit too low, allows a lot of overshoot
                const float pidDA = constrainf(pidD + pidA, -maxDAAngle, maxDAAngle);
                // note: an angle of more than 35 degrees can still be achieved as P and I grow

                /* possible economical alternative method using true length:
                float pidDASquared = pidDA * pidDA;
                float mag = sqrtf(pidDASquared + prevPidDASquared)
                if (mag > maxDAAngle) {
                    pidDA *= (maxDAAngle / mag);
                }
                prevPidDASquared = pidDASquared
                */

                // ** PID Sum **
                float pidSum = pidP + pidI + pidDA;

                // reset the position target when pidSum crosses zero, typically when velocity is very close to zero, ie craft has stopped
                // this enhances the smoothness of the transition from stick input back to position hold because there is no sharp change in pidSum
                if (latLong->isStarting && latLong->pidSum * pidSum < 0.0f) { // pidsum ns has reversed sign
                    resetPositionControlParams(latLong);
                    if (i == 0) {
                        currentTargetLocation.lat = gpsSol.llh.lat;
                    } else {
                        currentTargetLocation.lon = gpsSol.llh.lon;
                    }
                    latLong->distance = 0.0f;
                    posHold.sanityCheckDistance = 1000.0f; // 10m, once stable
                    latLong->isStarting = false;
                }
                latLong->pidSum = pidSum;

                // Debugs... distances in cm, angles in degrees * 10, velocities cm/2
                if (gyroConfig()->gyro_filter_debug_axis == i) {
                    DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 0, lrintf(distanceCm));
                    DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 1, lrintf(latLong->distance));
                    DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 2, lrintf(latLong->pidSum * 10));
                    DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 4, lrintf(pidP * 10));
                    DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 5, lrintf(pidI * 10));
                    DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 6, lrintf(pidDA * 10));
                }
            }
        }
        // ** Rotate pid Sum to quad frame of reference, into pitch and roll **
        float headingRads = DECIDEGREES_TO_RADIANS(attitude.values.yaw);
        const float sinHeading = sin_approx(headingRads);
        const float cosHeading = cos_approx(headingRads);

        posHold.pidSum[AI_ROLL] = -sinHeading * posHold.direction[NORTH_SOUTH].pidSum + cosHeading * posHold.direction[EAST_WEST].pidSum;
        posHold.pidSum[AI_PITCH] = cosHeading * posHold.direction[NORTH_SOUTH].pidSum + sinHeading * posHold.direction[EAST_WEST].pidSum;
    }

    // ** Final output to pid.c Angle Mode at 100Hz with primitive upsampling**
    autopilotAngle[AI_ROLL] = pt3FilterApply(&posHold.upsample[AI_ROLL], posHold.pidSum[AI_ROLL]);
    autopilotAngle[AI_PITCH] = pt3FilterApply(&posHold.upsample[AI_PITCH], posHold.pidSum[AI_PITCH]);
    // note: upsampling should really be done in earth frame, to avoid 10Hz wobbles if pilot yaws and the controller is applying significant pitch or roll

    if (gyroConfig()->gyro_filter_debug_axis == FD_ROLL) {
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 3, lrintf(posHold.pidSum[AI_ROLL] * 10));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 7, lrintf(autopilotAngle[AI_ROLL] * 10));
    } else {
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 3, lrintf(posHold.pidSum[AI_PITCH] * 10));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 7, lrintf(autopilotAngle[AI_PITCH] * 10));
    }


    return true;
}

bool isBelowLandingAltitude(void)
{
    return getAltitudeCm() < 100.0f * autopilotConfig()->landing_altitude_m;
}

float getAutopilotThrottle(void)
{
    return throttleOut;
}
