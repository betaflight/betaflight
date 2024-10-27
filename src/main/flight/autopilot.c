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
#include "fc/rc.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/position.h"
#include "rx/rx.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"

#include "autopilot.h"

#define ALTITUDE_P_SCALE  0.01f
#define ALTITUDE_I_SCALE  0.003f
#define ALTITUDE_D_SCALE  0.01f
#define ALTITUDE_F_SCALE  0.01f
#define POSITION_P_SCALE  0.001f
#define POSITION_I_SCALE  0.0005f
#define POSITION_D_SCALE  0.002f
#define POSITION_A_SCALE  0.002f

static pidCoefficient_t altitudePidCoeffs;
static pidCoefficient_t positionPidCoeffs;

static float altitudeI = 0.0f;
static float throttleOut = 0.0f;

typedef struct {
    float gpsDataIntervalS;
    float gpsDataFreqHz;
    float sanityCheckDistance;
    bool isStartingNS;
    bool isStartingEW;
    float peakInitialGroundspeed;
    float lpfCutoff;
    float pt1Gain;
    bool sticksActive;
    float previousDistanceNS;
    float previousDistanceEW;
    float previousVelocityNS;
    float previousVelocityEW;
    float previousPidSumNS;
    float previousPidSumEW;
    float integralNS;
    float integralEW;
} posHoldState;

static posHoldState posHold = {
    .gpsDataIntervalS = 0.1f,
    .gpsDataFreqHz = 10.0f,
    .sanityCheckDistance = 1000.0f,
    .isStartingNS = false,
    .isStartingEW = false,
    .peakInitialGroundspeed = 0.0f,
    .lpfCutoff = 1.0f,
    .pt1Gain = 1.0f,
    .sticksActive = false,
    .previousDistanceNS = 0.0f,
    .previousDistanceEW = 0.0f,
    .previousVelocityNS = 0.0f,
    .previousVelocityEW = 0.0f,
    .previousPidSumNS = 0.0f,
    .previousPidSumEW = 0.0f,
    .integralNS = 0.0f,
    .integralEW = 0.0f,
};

static gpsLocation_t currentTargetLocation = {0, 0, 0};
float autopilotAngle[ANGLE_INDEX_COUNT];
static pt1Filter_t velocityNSLpf;
static pt1Filter_t velocityEWLpf;
static pt2Filter_t accelerationNSLpf;
static pt2Filter_t accelerationEWLpf;

void autopilotInit(const autopilotConfig_t *config)
{
    altitudePidCoeffs.Kp = config->altitude_P * ALTITUDE_P_SCALE;
    altitudePidCoeffs.Ki = config->altitude_I * ALTITUDE_I_SCALE;
    altitudePidCoeffs.Kd = config->altitude_D * ALTITUDE_D_SCALE;
    altitudePidCoeffs.Kf = config->altitude_F * ALTITUDE_F_SCALE;
    positionPidCoeffs.Kp = config->position_P * POSITION_P_SCALE;
    positionPidCoeffs.Ki = config->position_I * POSITION_I_SCALE;
    positionPidCoeffs.Kd = config->position_D * POSITION_D_SCALE;
    positionPidCoeffs.Kf = config->position_A * POSITION_A_SCALE; // Kf used for acceleration
    posHold.lpfCutoff = config->position_cutoff * 0.01f;
    posHold.pt1Gain = pt1FilterGain(posHold.lpfCutoff, 0.1f); // assume 10Hz GPS connection at start
    pt1FilterInit(&velocityNSLpf, posHold.pt1Gain);
    pt1FilterInit(&velocityEWLpf, posHold.pt1Gain);
    pt2FilterInit(&accelerationNSLpf, posHold.pt1Gain);
    pt2FilterInit(&accelerationEWLpf, posHold.pt1Gain);
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

void resetPositionControlParamsNS(void) { // at the start, and while sticks are moving
    posHold.previousDistanceNS = 0.0f;
    posHold.previousVelocityNS = 0.0f;
    posHold.previousPidSumNS = 0.0f;
    pt1FilterInit(&velocityNSLpf, posHold.pt1Gain);
    pt2FilterInit(&accelerationNSLpf, posHold.pt1Gain);
    posHold.isStartingNS = true;
}

void resetPositionControlParamsEW(void) { // at the start, and while sticks are moving
    posHold.previousDistanceEW = 0.0f;
    posHold.previousVelocityEW = 0.0f;
    posHold.previousPidSumEW = 0.0f;
    pt1FilterInit(&velocityEWLpf, posHold.pt1Gain);
    pt2FilterInit(&accelerationEWLpf, posHold.pt1Gain);
    posHold.isStartingEW = true;
}

void resetPositionControl(gpsLocation_t initialTargetLocation) { // set only at the start frmo pos_hold.c
    currentTargetLocation = initialTargetLocation;
    resetPositionControlParamsNS();
    resetPositionControlParamsEW();
    posHold.peakInitialGroundspeed = 0.0f;
    posHold.integralEW = 0.0f;
    posHold.integralNS = 0.0f;
}

void updateTargetLocation(gpsLocation_t newTargetLocation) {
    currentTargetLocation = newTargetLocation;
}

bool positionControl(void) {

    // exit if we don't have a GPS 3D fix
    if (!STATE(GPS_FIX)) {
        return false; // cannot proceed without a GPS location
    }
    // and if no valid heading from GPS, or no mag
    if (!canUseGPSHeading
#ifdef USE_MAG
        && !compassIsHealthy()
#endif
        ) {
        return false;
    }

    posHold.gpsDataIntervalS = getGpsDataIntervalSeconds(); // interval for current GPS data value 0.01s to 1.0s
    posHold.gpsDataFreqHz = 1.0f / posHold.gpsDataIntervalS;

    if (posHold.sticksActive) {
        // if a Position Hold deadband is set, and sticks are outside deadband, allow pilot control in angle mode
        resetPositionControlParamsNS();
        resetPositionControlParamsEW();
        autopilotAngle[AI_ROLL] =  0.0f;
        autopilotAngle[AI_PITCH] = 0.0f; 
    } else {
        // control position
        // first get xy distances from current location (gpsSol.llh) to target location
        float nsDistance; // cm, steps of 11.1cm, North of target is positive
        float ewDistance; // cm, steps of 11.1cm, East of target is positive
        GPS_distances(&gpsSol.llh, &currentTargetLocation, &nsDistance, &ewDistance);
        float distanceCm = sqrtf(sq(nsDistance) + sq(ewDistance));

        posHold.pt1Gain = pt1FilterGain(posHold.lpfCutoff, posHold.gpsDataIntervalS);

        // ** Sanity check **
        // larger threshold if faster at start
        if (posHold.isStartingNS || posHold.isStartingEW) {
            posHold.sanityCheckDistance = gpsSol.groundSpeed > 1000 ? gpsSol.groundSpeed : 1000.0f;
        }
        // primarily to detect flyaway from no Mag or badly oriented Mag
        // but must accept some overshoot at the start, especially if entering at high speed
        if (distanceCm > posHold.sanityCheckDistance) {
            return false; // must stay within 10m or probably flying away
            // value at this point is a 'best guess' to detect IMU failure in the event the user has no Mag
            // if entering poshold from a stable hover, we would only exceed this if IMU was disoriented
            // if entering poshold at speed, it may overshoot this value and falsely fail, if so need something more complex
        }

        // ** PIDs **
        // separate PID controllers for latitude (NorthSouth or ns) and longitude (EastWest or ew)

        // ** P **

        const float nsP = nsDistance * positionPidCoeffs.Kp;
        const float ewP = ewDistance * positionPidCoeffs.Kp;

        // ** I **

        const float leak = 1.0f - 0.4f * posHold.gpsDataIntervalS; // gpsDataIntervalS is not more than 1.0s
        if (!posHold.isStartingNS){
            // only accumulate iTerm after completing the start phase
            // perhaps need a timeout on the start phase ?
            posHold.integralNS += nsDistance * posHold.gpsDataIntervalS;
        } else {
            // while moving sticks, slowly leak iTerm away, approx 2s time constant
            posHold.integralNS *= leak;
        }
        if (!posHold.isStartingEW){
            posHold.integralEW += ewDistance * posHold.gpsDataIntervalS;
        } else {
            posHold.integralEW *= leak;
        }

        const float nsI = posHold.integralNS * positionPidCoeffs.Ki;
        const float ewI = posHold.integralEW * positionPidCoeffs.Ki;

        // ** D ** //
        // get change in distance in NS and EW directions from gps.c using the `GPS_distances` function
        // this gives cleaner velocity data than the module supplied GPS Speed and Heading information

        const float velocityNS = (nsDistance - posHold.previousDistanceNS) * posHold.gpsDataFreqHz; // cm/s, minimum step 11.1 cm/s
        const float velocityEW = (ewDistance - posHold.previousDistanceEW) * posHold.gpsDataFreqHz;
        posHold.previousDistanceNS = nsDistance;
        posHold.previousDistanceEW = ewDistance;

        float accelerationNS = (velocityNS - posHold.previousVelocityNS) * posHold.gpsDataFreqHz;
        posHold.previousVelocityNS = velocityNS;
        float accelerationEW = (velocityEW - posHold.previousVelocityEW) * posHold.gpsDataFreqHz;
        posHold.previousVelocityEW = velocityEW;

        // scale and filter - filter cutoffs vary during the startup phase
        float nsD = velocityNS * positionPidCoeffs.Kd;
        pt1FilterUpdateCutoff(&velocityNSLpf, posHold.pt1Gain);
        nsD = pt1FilterApply(&velocityNSLpf, nsD);

        float ewD = velocityEW * positionPidCoeffs.Kd;
        pt1FilterUpdateCutoff(&velocityEWLpf, posHold.pt1Gain);
        ewD = pt1FilterApply(&velocityEWLpf, ewD);

        float nsA = accelerationNS * positionPidCoeffs.Kf;
        pt2FilterUpdateCutoff(&accelerationNSLpf, posHold.pt1Gain); // using PT1 gain for stronger cutoff
        nsA = pt2FilterApply(&accelerationNSLpf, nsA);

        float ewA = accelerationEW * positionPidCoeffs.Kf;
        pt2FilterUpdateCutoff(&accelerationEWLpf, posHold.pt1Gain); // using PT1 gain for stronger cutoff
        ewA = pt2FilterApply(&accelerationEWLpf, ewA);

        // limit sum of D and A because otherwise can be too aggressive when starting at speed
        float nsDA = nsD + nsA;
        float ewDA = ewD + ewA;
        const float maxDAAngle = 35.0f; // limit in degrees; arbitrary.  20 is a bit too low, allows a lot of overshoot
        // an angle of more than 35 degrees is achieved as P and I grow
        // ** todo = should this be half of the user-configurable angle_limit?  Or fixed?
        nsDA = constrainf(nsDA, -maxDAAngle, maxDAAngle);
        ewDA = constrainf(ewDA, -maxDAAngle, maxDAAngle);

        // ** PID Sum **

        float nsPidSum = nsP + nsI + nsDA;
        float ewPidSum = ewP + ewI + ewDA;

        // terminate initial startup behaviour as pidSum crosses zero
        // this is best for smoothness of the transition from the stop to the new target location
        if (posHold.isStartingNS && posHold.previousPidSumNS * nsPidSum < 0.0f) { // pidsum ns has reversed sign
            resetPositionControlParamsNS();
            currentTargetLocation.lat = gpsSol.llh.lat;
            nsDistance = 0.0f;
            posHold.isStartingNS = false;
        }
        if (posHold.isStartingEW && posHold.previousPidSumEW * ewPidSum < 0.0f) { // pidsum ns has reversed sign
            resetPositionControlParamsEW();
            ewDistance = 0.0f;
            currentTargetLocation.lon = gpsSol.llh.lon;
            posHold.isStartingEW = false;
        }
        posHold.previousPidSumNS = nsPidSum;
        posHold.previousPidSumEW = ewPidSum;

        // ** Rotate pid Sum to quad frame of reference, into pitch and roll **
        float headingRads = (attitude.values.yaw / 10.0f) * RAD; // will be constrained to +/-pi in sin_approx()
        const float sinHeading = sin_approx(headingRads);
        const float cosHeading = cos_approx(headingRads);
        float pidSumRoll = -sinHeading * nsPidSum + cosHeading * ewPidSum;
        float pidSumPitch = cosHeading * nsPidSum + sinHeading * ewPidSum;

        // todo: fix the upsample filtering in pid.c
        // pidSum steps at GPS rate, and needs to change the pid.c upsampling filter for smoothness.

        // ** Final output to pid.c Angle Mode **
        autopilotAngle[AI_ROLL] = pidSumRoll;
        autopilotAngle[AI_PITCH] = pidSumPitch; 

        // Debugs... distances in cm, angles in degrees * 10, velocities cm/2
        if (gyroConfig()->gyro_filter_debug_axis == FD_ROLL) {
            DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 0, lrintf(distanceCm));
            DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 1, lrintf(ewDistance));
            DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 2, lrintf(ewPidSum * 10));
            DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 3, lrintf(pidSumRoll * 10));
            DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 4, lrintf(ewP * 10));
            DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 5, lrintf(ewI * 10));
            DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 6, lrintf(ewDA * 10));
            DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 7, lrintf(posHold.integralEW * 10));
        } else {
            DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 0, lrintf(distanceCm));
            DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 1, lrintf(nsDistance));
            DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 2, lrintf(nsPidSum * 10));
            DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 3, lrintf(pidSumPitch * 10));
            DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 4, lrintf(nsP * 10));
            DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 5, lrintf(nsI * 10));
            DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 6, lrintf(nsDA * 10));
            DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 7, lrintf(posHold.integralNS));
        }

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
