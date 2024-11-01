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
#include "fc/rc.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/position.h"
#include "rx/rx.h"
#include "sensors/gyro.h"

#include "autopilot.h"

#define ALTITUDE_P_SCALE  0.01f
#define ALTITUDE_I_SCALE  0.003f
#define ALTITUDE_D_SCALE  0.01f
#define ALTITUDE_F_SCALE  0.01f
#define POSITION_P_SCALE  0.0012f
#define POSITION_I_SCALE  0.0001f
#define POSITION_D_SCALE  0.0015f
#define POSITION_A_SCALE  0.0008f
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

typedef enum {
    EW = 0,
    NS
} axisEF_t;

earthFrame_t eastWest;
earthFrame_t northSouth;

typedef struct {
    float gpsDataIntervalS;
    float gpsDataFreqHz;
    float sanityCheckDistance;
    float peakInitialGroundspeed;
    float lpfCutoff;
    float pt1Gain;
    bool sticksActive;
    float pidSumCraft[2];
    pt3Filter_t upsample[2];
    earthFrame_t efAxis[2];
} posHoldState;

static posHoldState posHold = {
    .gpsDataIntervalS = 0.1f,
    .gpsDataFreqHz = 10.0f,
    .sanityCheckDistance = 1000.0f,
    .peakInitialGroundspeed = 0.0f,
    .lpfCutoff = 1.0f,
    .pt1Gain = 1.0f,
    .sticksActive = false,
    .pidSumCraft = { 0.0f, 0.0f },
    .upsample = { {0}, {0} },
    .efAxis = { {0} }
};

static gpsLocation_t currentTargetLocation = {0, 0, 0};
float autopilotAngle[ANGLE_INDEX_COUNT];

void resetPositionControlParams(earthFrame_t *efAxis) {
    // at the start, and while sticks are moving
    // Clear accumulation in filters
    pt1FilterInit(&efAxis->velocityLpf, posHold.pt1Gain);
    pt1FilterInit(&efAxis->accelerationLpf, posHold.pt1Gain);
    // Initiate starting behaviour
    efAxis->isStarting = true;
}

void resetPositionControl(gpsLocation_t initialTargetLocation) { // set only at the start frmo pos_hold.c
    currentTargetLocation = initialTargetLocation;
    resetPositionControlParams(&posHold.efAxis[EW]);
    resetPositionControlParams(&posHold.efAxis[NS]);
    posHold.peakInitialGroundspeed = 0.0f;
    posHold.efAxis[EW].integral = 0.0f;
    posHold.efAxis[NS].integral = 0.0f;
    posHold.sticksActive = false;
    posHold.sanityCheckDistance = gpsSol.groundSpeed > 500 ? gpsSol.groundSpeed * 2.0f : 1000.0f;
}

void autopilotInit(const autopilotConfig_t *config)
{
    eastWest = posHold.efAxis[EW];
    northSouth = posHold.efAxis[NS];
    altitudePidCoeffs.Kp = config->altitude_P * ALTITUDE_P_SCALE;
    altitudePidCoeffs.Ki = config->altitude_I * ALTITUDE_I_SCALE;
    altitudePidCoeffs.Kd = config->altitude_D * ALTITUDE_D_SCALE;
    altitudePidCoeffs.Kf = config->altitude_F * ALTITUDE_F_SCALE;
    positionPidCoeffs.Kp = config->position_P * POSITION_P_SCALE;
    positionPidCoeffs.Ki = config->position_I * POSITION_I_SCALE;
    positionPidCoeffs.Kd = config->position_D * POSITION_D_SCALE;
    positionPidCoeffs.Kf = config->position_A * POSITION_A_SCALE; // Kf used for acceleration
    // initialise filters with approximate filter gain
    posHold.lpfCutoff = config->position_cutoff * 0.01f;
    posHold.pt1Gain = pt1FilterGain(posHold.lpfCutoff, 0.1f); // assume 10Hz GPS connection at start
    float upsampleCutoff = pt3FilterGain(UPSAMPLING_CUTOFF, 0.01f); // 5Hz, assuming 100Hz task rate
    pt3FilterInit(&posHold.upsample[AI_ROLL], upsampleCutoff);
    pt3FilterInit(&posHold.upsample[AI_PITCH], upsampleCutoff);
    // Reset parameters and initialise PT1 filters for both NS and EW
    resetPositionControlParams(&posHold.efAxis[EW]);
    resetPositionControlParams(&posHold.efAxis[NS]);
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

void setTargetLocation(gpsLocation_t newTargetLocation)
{
    currentTargetLocation = newTargetLocation;
    // note: will cause a large D and A spike if there is an abrupt change in distance
    // function is intended for only small changes in position
    // for example, where the step distance change reflects an intended velocity, determined by a client function
    // if we had a 'target_ground_speed' value, like in gps_rescue, we can make a function that starts and stops smoothly and targets that velocity
}

void resetLocation(axisEF_t loopAxis)
{
    if (loopAxis == EW) {
        currentTargetLocation.lon = gpsSol.llh.lon; // update East-West / / longitude position
    } else {
        currentTargetLocation.lat = gpsSol.llh.lat; // update North-South / latitude position
    }
    posHold.efAxis[loopAxis].previousDistance = 0.0f; // and reset the previous distance
}

bool positionControl(void) {

    if (isNewGPSDataAvailable()) {
        posHold.gpsDataIntervalS = getGpsDataIntervalSeconds(); // interval for current GPS data value 0.01s to 1.0s
        posHold.gpsDataFreqHz = 1.0f / posHold.gpsDataIntervalS;

        // first get xy distances from current location (gpsSol.llh) to target location
        vector2_t gpsDistance;
        GPS_distances(&gpsSol.llh, &currentTargetLocation, &gpsDistance.y, &gpsDistance.x); // Y is north, X is south

        posHold.efAxis[EW].distance = gpsDistance.x;
        posHold.efAxis[NS].distance = gpsDistance.y;
        const float distanceCm = vector2Norm(&gpsDistance);

        const float leak = 1.0f - 0.4f * posHold.gpsDataIntervalS;
        // leak iTerm while sticks are centered, 2s time constant approximately
        const float lpfGain = pt1FilterGain(posHold.lpfCutoff, posHold.gpsDataIntervalS);

        // ** Sanity check **
        // primarily to detect flyaway from no Mag or badly oriented Mag
        // must accept some overshoot at the start, especially if entering at high speed
        if (distanceCm > posHold.sanityCheckDistance) {
            return false;
        }

        // static float prevPidDASquared = 0.0f; // if we limit DA on true vector length

        for (int loopAxis = 0; loopAxis < 2; loopAxis++) {
            earthFrame_t *efAxis = &posHold.efAxis[loopAxis];
            // separate PID controllers for latitude (NorthSouth or NS) and longitude (EastWest or EW)

            // ** P **
            const float pidP = efAxis->distance * positionPidCoeffs.Kp;

            // ** I **
            efAxis->integral += efAxis->isStarting ? 0.0f : efAxis->distance * posHold.gpsDataIntervalS;
            // only add to iTerm while in hold phase
            const float pidI = efAxis->integral * positionPidCoeffs.Ki;

            // ** D ** //
            // Velocity derived from GPS position works better than module supplied GPS Speed and Heading information

            float velocity = (efAxis->distance - efAxis->previousDistance) * posHold.gpsDataFreqHz; // cm/s, minimum step 11.1 cm/s
            efAxis->previousDistance = efAxis->distance;
            pt1FilterUpdateCutoff(&efAxis->velocityLpf, lpfGain);
            const float velocityFiltered = pt1FilterApply(&efAxis->velocityLpf, velocity);
            float pidD = velocityFiltered * positionPidCoeffs.Kd;

            float acceleration = (velocity - efAxis->previousVelocity) * posHold.gpsDataFreqHz;
            efAxis->previousVelocity = velocity;
            pt1FilterUpdateCutoff(&efAxis->accelerationLpf, lpfGain);
            const float accelerationFiltered = pt1FilterApply(&efAxis->accelerationLpf, acceleration);
            const float pidA = accelerationFiltered * positionPidCoeffs.Kf;

            if (posHold.sticksActive) {
                efAxis->isStarting = true;
                // while sticks are moving, reset the location on each axis, to maintain a usable D value
                resetLocation(loopAxis);
                // slowly leak iTerm away, approx 2s time constant
                efAxis->integral *= leak;
            } else if (efAxis->isStarting) {
                // period after sticks stop, but before craft has stopped
                pidD *= 1.6f; // aribitrary D boost to stop more quickly when sticks are centered
                if (velocity * velocityFiltered < 0.0f) {
                    // when craft has nearly stopped moving, reset home and end the start phase
                    resetLocation(loopAxis);
                    efAxis->isStarting = false;
                    posHold.sanityCheckDistance = 1000.0f; // restore normal 10m limit, once stable
                }
            }

            // limit sum of D and A per axis because otherwise can be too aggressive when starting at speed
            const float maxDAAngle = 35.0f; // limit in degrees; arbitrary.  20 is a bit too low, allows a lot of overshoot
            const float pidDA = constrainf(pidD + pidA, -maxDAAngle, maxDAAngle);
            // note: an angle of more than 35 degrees can still be achieved as P and I grow, and on diagonal
            // note: angle mode limit is absolute max, as set in CLI default is 60

            /* possible alternate method using total vector length for the limiting, if needed:
            float pidDASquared = pidDA * pidDA;
            float mag = sqrtf(pidDASquared + prevPidDASquared)
            if (mag > maxDAAngle) {
                pidDA *= (maxDAAngle / mag);
            }
            prevPidDASquared = pidDASquared
            */

            // ** PID Sum **
            efAxis->pidSum = pidP + pidI + pidDA;

            // Debugs... distances in cm, angles in degrees * 10, velocities cm/2
            if (gyroConfig()->gyro_filter_debug_axis == loopAxis) {
                DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 0, lrintf(distanceCm));
                DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 4, lrintf(pidP * 10));
                DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 5, lrintf(pidI * 10));
                DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 6, lrintf(pidD * 10));
                DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 7, lrintf(pidA * 10));
            }
        }

        if (posHold.sticksActive) {
            // if a Position Hold deadband is set, and sticks are outside deadband, allow pilot control in angle mode
            posHold.pidSumCraft[AI_ROLL] = 0.0f;
            posHold.pidSumCraft[AI_PITCH] = 0.0f;
        } else {
            // ** Rotate pid Sum to quad frame of reference, into pitch and roll **
            const float headingRads = DECIDEGREES_TO_RADIANS(attitude.values.yaw);
            const float sinHeading = sin_approx(headingRads);
            const float cosHeading = cos_approx(headingRads);
            posHold.pidSumCraft[AI_ROLL] = -sinHeading * posHold.efAxis[NS].pidSum + cosHeading * posHold.efAxis[EW].pidSum;
            posHold.pidSumCraft[AI_PITCH] = cosHeading * posHold.efAxis[NS].pidSum + sinHeading * posHold.efAxis[EW].pidSum;
        }
    }

    // ** Final output to pid.c Angle Mode at 100Hz with primitive upsampling**
    autopilotAngle[AI_ROLL] = pt3FilterApply(&posHold.upsample[AI_ROLL], posHold.pidSumCraft[AI_ROLL]);
    autopilotAngle[AI_PITCH] = pt3FilterApply(&posHold.upsample[AI_PITCH], posHold.pidSumCraft[AI_PITCH]);
    // note: upsampling should really be done in earth frame, to avoid 10Hz wobbles if pilot yaws and the controller is applying significant pitch or roll

    if (gyroConfig()->gyro_filter_debug_axis == FD_ROLL) {
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 1, lrintf(posHold.efAxis[NS].distance));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 2, lrintf(posHold.efAxis[NS].pidSum * 10));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 3, lrintf(autopilotAngle[AI_ROLL] * 10));
    } else {
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 1, lrintf(posHold.efAxis[NS].distance));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 2, lrintf(posHold.efAxis[NS].pidSum * 10));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 3, lrintf(autopilotAngle[AI_PITCH] * 10));
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

bool isAutopilotActive(void)
{
    return !posHold.sticksActive;
}
