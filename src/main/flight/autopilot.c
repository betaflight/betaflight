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

#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/position.h"
#include "rx/rx.h"

#include "autopilot.h"

#define ALTITUDE_P_SCALE  0.01f
#define ALTITUDE_I_SCALE  0.003f
#define ALTITUDE_D_SCALE  0.01f
#define ALTITUDE_F_SCALE  0.01f
#define POSITION_P_SCALE  0.01f
#define POSITION_I_SCALE  0.01f
#define POSITION_D_SCALE  0.01f

static pidCoefficient_t altitudePidCoeffs;
static float altitudeI = 0.0f;
static float throttleOut = 0.0f;

static pidCoefficient_t positionPidCoeffs;
static float previousDistanceCm = 0.0f;
static float previousVelocity = 0.0f;
float posHoldAngle[ANGLE_INDEX_COUNT];
static pt1Filter_t accelerationLpf;
static float positionAccelerationCutoffHz;

void autopilotInit(const autopilotConfig_t *config)
{
    altitudePidCoeffs.Kp = config->altitude_P * ALTITUDE_P_SCALE;
    altitudePidCoeffs.Ki = config->altitude_I * ALTITUDE_I_SCALE;
    altitudePidCoeffs.Kd = config->altitude_D * ALTITUDE_D_SCALE;
    altitudePidCoeffs.Kf = config->altitude_F * ALTITUDE_F_SCALE;
    positionPidCoeffs.Kp = config->position_P * POSITION_P_SCALE;
    positionPidCoeffs.Ki = config->position_I * POSITION_I_SCALE;
    positionPidCoeffs.Kd = config->position_D * POSITION_D_SCALE;

    positionAccelerationCutoffHz = config->position_filter_hz / 100.0f;
    const float gain = pt1FilterGain(positionAccelerationCutoffHz, 0.1f); // assume 10Hz GPS connection at start
    pt1FilterInit(&accelerationLpf, gain);
}

const pidCoefficient_t *getAltitudePidCoeffs(void)
{
    return &altitudePidCoeffs;
}

const pidCoefficient_t *getPositionPidCoeffs(void)
{
    return &positionPidCoeffs;
}

void resetAltitudeControl (void) {
    altitudeI = 0.0f;
}

void resetPositionControl (void) {
    previousDistanceCm = 0.0f;
    previousVelocity = 0.0f;
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
    const float tiltMultiplier = 2.0f - fmaxf(getCosTiltAngle(), 0.5f);
    // 1 = flat, 1.24 at 40 degrees, max 1.5 around 60 degrees, the default limit of Angle Mode
    // 2 - cos(x) is between 1/cos(x) and 1/sqrt(cos(x)) in this range
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

void positionControl(gpsLocation_t targetLocation) {
    // gpsSol.llh = current gps location
    // get distance and bearing from current location to target location
    // void GPS_distance_cm_bearing(const gpsLocation_t *from, const gpsLocation_t* to, bool dist3d, uint32_t *pDist, int32_t *pBearing)
    static float rollSetpoint = 0.0f;
    static float pitchSetpoint = 0.0f;
    uint32_t distanceCm;
    int32_t bearing; // degrees * 100
    GPS_distance_cm_bearing(&gpsSol.llh, &targetLocation, false, &distanceCm, &bearing);

    const float errorAngle = (attitude.values.yaw * 10.0f - bearing) / 100.0f;
    float normalisedErrorAngle = fmodf(errorAngle + 360.0f, 360.0f);

    if (normalisedErrorAngle > 180.0f) {
        normalisedErrorAngle -= 360.0f; // Range: -180 to 180
    }

    float errorAngleRadians = normalisedErrorAngle * (M_PI / 180.0f);

    // Calculate correction factors using sine and cosine
    float rollCorrection = -sin_approx(errorAngleRadians); // + 1 when target is left, -1 when to right, of the craft
    float pitchCorrection = cos_approx(errorAngleRadians); // + 1 when target is ahead, -1 when behind, the craft

    const float gpsDataIntervalS = getGpsDataIntervalSeconds(); // 0.01s to 1.0s
    float velocity = (distanceCm - previousDistanceCm) / gpsDataIntervalS; // positive away
    previousDistanceCm = distanceCm;

    float acceleration = (velocity - previousVelocity) / gpsDataIntervalS; // positive away
    previousVelocity = velocity;
    // ** THIS WILL NEED A FILTER LIKE FOR GPS RESCUE**

    const float gain = pt1FilterGain(positionAccelerationCutoffHz, gpsDataIntervalS);
    pt1FilterUpdateCutoff(&accelerationLpf, gain);
    acceleration = pt1FilterApply(&accelerationLpf, acceleration);

    float velocityP = velocity * positionPidCoeffs.Kp;     // velocity away from intended location
    float velocityI = distanceCm * positionPidCoeffs.Ki;   // integral of velocity is distance from intended location
    float velocityD = acceleration * positionPidCoeffs.Kd; // acceleration away from intended location
    
    float pidSum = velocityP + velocityI + velocityD; // greater when position error is bad and getting worse

    rollSetpoint = rollCorrection * pidSum;
    pitchSetpoint = pitchCorrection * pidSum;

    DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 0, lrintf(normalisedErrorAngle)); //-180 to +180
    DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 1, lrintf(rollCorrection * 100));
    DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 2, lrintf(pitchCorrection * 100));
    DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 3, lrintf(distanceCm));
    DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 4, lrintf(velocityP));
    DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 5, lrintf(velocityI));
    DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 6, lrintf(velocityD));

    // send setpoints to pid.c using a method similar to that in gpsRescueAngle[axis]
    // value sent needs shoiuld be in degrees * 100
    // values will have steps at GPS rate, if too jumpy we would need to upsample smooth them
    posHoldAngle[AI_ROLL] = rollSetpoint;
    posHoldAngle[AI_PITCH] = pitchSetpoint;

    DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 7, lrintf(rollSetpoint));

    // but for now let's not really do that until we get the PIDs sorted out :-)
//    posHoldAngle[AI_PITCH] = 0.0f;
//    posHoldAngle[AI_ROLL] = 0.0f;
}

bool isBelowLandingAltitude(void)
{
    return getAltitudeCm() < 100.0f * autopilotConfig()->landing_altitude_m;
}

const pidCoefficient_t *getAltitudePidCoeffs(void)
{
    return &altitudePidCoeffs;

float getAutopilotThrottle(void)
{
    return throttleOut;
}
