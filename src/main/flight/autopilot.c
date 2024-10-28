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
#include "fc/core.h"
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
    pt2Filter_t accelerationLpf;
} vectors_t;

typedef struct {
    float gpsDataIntervalS;
    float gpsDataFreqHz;
    float sanityCheckDistance;
    float peakInitialGroundspeed;
    float lpfCutoff;
    float pt1Gain;
    bool sticksActive;
    float pidSumRoll;
    float pidSumPitch;
    pt3Filter_t upsampleRollLpf;
    pt3Filter_t upsamplePitchLpf;
    vectors_t NS;
    vectors_t EW;
} posHoldState;

static posHoldState posHold = {
    .gpsDataIntervalS = 0.1f,
    .gpsDataFreqHz = 10.0f,
    .sanityCheckDistance = 1000.0f,
    .peakInitialGroundspeed = 0.0f,
    .lpfCutoff = 1.0f,
    .pt1Gain = 1.0f,
    .sticksActive = false,
    .pidSumRoll = 0.0f,
    .pidSumPitch = 0.0f,
    .NS = {
        .isStarting = false,
        .distance = 0.0f,
        .previousDistance = 0.0f,
        .previousVelocity = 0.0f,
        .integral = 0.0f,
        .pidSum = 0.0f,
    },
    .EW = {
        .isStarting = false,
        .distance = 0.0f,
        .previousDistance = 0.0f,
        .previousVelocity = 0.0f,
        .integral = 0.0f,
        .pidSum = 0.0f,
    },
};

static gpsLocation_t currentTargetLocation = {0, 0, 0};
float autopilotAngle[ANGLE_INDEX_COUNT];

void resetPositionControlParams(vectors_t *latLong) {
    // at the start, and while sticks are moving
    latLong->previousDistance = 0.0f;
    latLong->previousVelocity = 0.0f;
    latLong->pidSum = 0.0f;
    // Clear accumulation in filters
    pt1FilterInit(&latLong->velocityLpf, posHold.pt1Gain);
    pt2FilterInit(&latLong->accelerationLpf, posHold.pt1Gain);
    // Initiate starting behaviour
    latLong->isStarting = true;
}

void resetPositionControl(gpsLocation_t initialTargetLocation) { // set only at the start frmo pos_hold.c
    currentTargetLocation = initialTargetLocation;
    resetPositionControlParams(&posHold.NS);
    resetPositionControlParams(&posHold.EW);
    posHold.peakInitialGroundspeed = 0.0f;
    posHold.NS.integral = 0.0f;
    posHold.EW.integral = 0.0f;
}

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
    // approximate filter gain
    posHold.lpfCutoff = config->position_cutoff * 0.01f;
    posHold.pt1Gain = pt1FilterGain(posHold.lpfCutoff, 0.1f); // assume 10Hz GPS connection at start
    float upsampleCutoff = pt3FilterGain(UPSAMPLING_CUTOFF, 0.01f); // 5Hz, assuming 100Hz task rate
    pt3FilterInit(&posHold.upsampleRollLpf, upsampleCutoff);
    pt3FilterInit(&posHold.upsamplePitchLpf, upsampleCutoff);
    // initialise filters
// Reset parameters for both NS and EW
    resetPositionControlParams(&posHold.NS);
    resetPositionControlParams(&posHold.EW);
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
    
    if (!wasThrottleRaised()) {
        return false;
    }

    if (isNewDataForPosHold()) {
        posHold.gpsDataIntervalS = getGpsDataIntervalSeconds(); // interval for current GPS data value 0.01s to 1.0s
        posHold.gpsDataFreqHz = 1.0f / posHold.gpsDataIntervalS;
        if (posHold.sticksActive) {
            // if a Position Hold deadband is set, and sticks are outside deadband, allow pilot control in angle mode
            resetPositionControlParams(&posHold.NS);
            resetPositionControlParams(&posHold.EW);
            posHold.pidSumRoll = 0.0f;
            posHold.pidSumPitch = 0.0f;
        } else {
            // first get xy distances from current location (gpsSol.llh) to target location
            float nsDistance; // cm, steps of 11.1cm, North of target is positive
            float ewDistance; // cm, steps of 11.1cm, East of target is positive
            GPS_distances(&gpsSol.llh, &currentTargetLocation, &nsDistance, &ewDistance);
            float distanceCm = sqrtf(sq(nsDistance) + sq(ewDistance));
    
            posHold.NS.distance = nsDistance;
            posHold.EW.distance = ewDistance;
    
            posHold.pt1Gain = pt1FilterGain(posHold.lpfCutoff, posHold.gpsDataIntervalS);
            const float leak = 1.0f - 0.4f * posHold.gpsDataIntervalS; // gpsDataIntervalS is not more than 1.0s
    
            // ** Sanity check **
            // larger threshold if faster at start
            if (posHold.NS.isStarting || posHold.EW.isStarting) {
                posHold.sanityCheckDistance = gpsSol.groundSpeed > 1000 ? gpsSol.groundSpeed : 1000.0f;
                // 1s of flight at current speed or 10m, in cm
            }
            // primarily to detect flyaway from no Mag or badly oriented Mag
            // but must accept some overshoot at the start, especially if entering at high speed
            if (distanceCm > posHold.sanityCheckDistance) {
                return false; // must stay within 10m or probably flying away
                // value at this point is a 'best guess' to detect IMU failure in the event the user has no Mag
                // if entering poshold from a stable hover, we would only exceed this if IMU was disoriented
                // if entering poshold at speed, it may overshoot this value and falsely fail, if so need something more complex
            }
    
            vectors_t *vectors[] = { &posHold.NS, &posHold.EW };
            for (int i = 0; i < 2; i++) {
                vectors_t *latLong = vectors[i];
    
                // separate PID controllers for latitude (NorthSouth or ns) and longitude (EastWest or ew)
    
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
                // get change in distance in NS and EW directions from gps.c using the `GPS_distances` function
                // this gives cleaner velocity data than the module supplied GPS Speed and Heading information
                float velocity = (latLong->distance - latLong->previousDistance) * posHold.gpsDataFreqHz; // cm/s, minimum step 11.1 cm/s
                latLong->previousDistance = latLong->distance;
    
                float acceleration = (velocity - latLong->previousVelocity) * posHold.gpsDataFreqHz;
                latLong->previousVelocity = velocity;
        
                // scale and filter - filter cutoffs vary during the startup phase
                float pidD = velocity * positionPidCoeffs.Kd;
                pt1FilterUpdateCutoff(&latLong->velocityLpf, posHold.pt1Gain);
                pidD = pt1FilterApply(&latLong->velocityLpf, pidD);
    
                float pidA = acceleration * positionPidCoeffs.Kd;
                pt2FilterUpdateCutoff(&latLong->accelerationLpf, posHold.pt1Gain);
                pidA = pt2FilterApply(&latLong->accelerationLpf, pidA);
        
                // limit sum of D and A because otherwise can be too aggressive when starting at speed
                const float maxDAAngle = 35.0f; // limit in degrees; arbitrary.  20 is a bit too low, allows a lot of overshoot
                // an angle of more than 35 degrees is achieved as P and I grow
                // ** todo = should this be half of the user-configurable angle_limit?  Or fixed?
                const float pidDA = constrainf(pidD + pidA, -maxDAAngle, maxDAAngle);
    
                // ** PID Sum **
                float pidSum = pidP + pidI + pidDA;
    
                // terminate initial startup behaviour separately for latitude and longitude controllers
                // the position target is reset when pidSum crosses zero
                // this enhances the smoothness of the transition from stick input back to position hold - there is no sharp change in pidSum
                if (latLong->isStarting && latLong->pidSum * pidSum < 0.0f) { // pidsum ns has reversed sign
                    resetPositionControlParams(latLong);
                    if (i == 0) {
                        currentTargetLocation.lat = gpsSol.llh.lat;  // can we simplify this within the loop?
                    } else {
                        currentTargetLocation.lon = gpsSol.llh.lon;
                    }
                    latLong->distance = 0.0f;
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
        float headingRads = DECIDEGREES_TO_RADIANS(attitude.values.yaw); // will be constrained to +/-pi in sin_approx()
        const float sinHeading = sin_approx(headingRads);
        const float cosHeading = cos_approx(headingRads);

        posHold.pidSumRoll = -sinHeading * posHold.NS.pidSum + cosHeading * posHold.EW.pidSum;
        posHold.pidSumPitch = cosHeading * posHold.NS.pidSum + sinHeading * posHold.EW.pidSum;
    }

    // ** Final output to pid.c Angle Mode at 100Hz with primitive upsampling**
    autopilotAngle[AI_ROLL] = pt3FilterApply(&posHold.upsampleRollLpf, posHold.pidSumRoll);
    autopilotAngle[AI_PITCH] = pt3FilterApply(&posHold.upsamplePitchLpf, posHold.pidSumPitch);

    if (gyroConfig()->gyro_filter_debug_axis == FD_ROLL) {
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 3, lrintf(posHold.pidSumRoll * 10));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 7, lrintf(autopilotAngle[AI_ROLL] * 10));
    } else {
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 3, lrintf(posHold.pidSumPitch * 10));
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
