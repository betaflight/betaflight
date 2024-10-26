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
    float distanceCm;
    float previousDistanceCm;
    float sanityCheckDistance;
    bool isStarting;
    float peakInitialGroundspeed;
    float lpfCutoff;
    bool sticksActive;
    float previousVelocityEW;
    float previousVelocityNS;
    float integralEW;
    float integralNS;
} posHoldState;

static posHoldState posHold = {
    .gpsDataIntervalS = 0.1f,
    .gpsDataFreqHz = 10.0f,
    .distanceCm = 0.0f,
    .previousDistanceCm = 0.0f,
    .sanityCheckDistance = 1000.0f,
    .isStarting = false,
    .peakInitialGroundspeed = 0.0f,
    .lpfCutoff = 1.0f,
    .sticksActive = false,
    .previousVelocityEW = 0.0f,
    .previousVelocityNS = 0.0f,
    .integralEW = 0.0f,
    .integralNS = 0.0f,
};

static gpsLocation_t currentTargetLocation = {0, 0, 0};
static gpsLocation_t previousLocation = {0, 0, 0};
float autopilotAngle[ANGLE_INDEX_COUNT];
static pt1Filter_t velocityNSLpf;
static pt1Filter_t velocityEWLpf;
static pt2Filter_t accelerationRollLpf;
static pt2Filter_t accelerationPitchLpf;

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
    const float pt1Gain = pt1FilterGain(posHold.lpfCutoff, 0.1f); // assume 10Hz GPS connection at start
    pt1FilterInit(&velocityNSLpf, pt1Gain);
    pt1FilterInit(&velocityEWLpf, pt1Gain);
    pt2FilterInit(&accelerationRollLpf, pt1Gain);
    pt2FilterInit(&accelerationPitchLpf, pt1Gain);
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

void resetPositionControlParams(void) { // at the start, and while sticks are moving
    posHold.distanceCm = 0.0f;
    posHold.previousDistanceCm = 0.0f;
    posHold.sanityCheckDistance = 1000.0f;
    posHold.previousVelocityEW = 0.0f;
    posHold.previousVelocityNS = 0.0f;
    posHold.lpfCutoff = autopilotConfig()->position_cutoff * 0.003f; // slow rise at start
    const float pt1Gain = pt1FilterGain(posHold.lpfCutoff, 0.1f);
    // reset all lowpass filter accumulators to zero
    pt1FilterInit(&velocityNSLpf, pt1Gain);
    pt1FilterInit(&velocityEWLpf, pt1Gain);
    pt2FilterInit(&accelerationRollLpf, pt1Gain);
    pt2FilterInit(&accelerationPitchLpf, pt1Gain);
    posHold.isStarting = true;
}

void resetPositionControl(gpsLocation_t initialTargetLocation) { // set only at the start
    currentTargetLocation = initialTargetLocation;
    resetPositionControlParams();
    posHold.peakInitialGroundspeed = 0.0f;
    posHold.integralEW = 0.0f;
    posHold.integralNS = 0.0f;
    previousLocation = gpsSol.llh;
}

void updateTargetLocation(gpsLocation_t newTargetLocation) {
    currentTargetLocation = newTargetLocation;
    resetPositionControlParams(); // don't reset accumulated iTerm
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

    // get distance and bearing from current location (gpsSol.llh) to target location
    uint32_t distanceCm = 0;
    int32_t bearing = 0;        // degrees * 100
    GPS_distance_cm_bearing(&gpsSol.llh, &currentTargetLocation, false, &distanceCm, &bearing);
    posHold.distanceCm = (float)distanceCm;

    // ** handle startup **
    
    // at the start, if the quad was moving, or randomly, distance from the start point will increase
    // check for the distance increase to 'stop', then reset the target point
    // this looks a lot better than a huge rebound
    // D and A are primarily responsible for the immediate 'stopping' behaviour
    // so that the stop is not too harsh, these are smoothed more than normal right at the start
    if (posHold.isStarting) {
        // very first time, or after the sticks were centered and the pilot flew around a bit
        posHold.peakInitialGroundspeed = fmaxf(posHold.peakInitialGroundspeed, gpsSol.groundSpeed);
        // at first, give the filter a long time constant to soften the harshness of the stop
        if (gpsSol.groundSpeed > 0.5f * posHold.peakInitialGroundspeed) {
            posHold.lpfCutoff = autopilotConfig()->position_cutoff * 0.003f; // 3x slower than normal
        } else {
            // when close to stopping, make the time constant shorter than normal
            // so that D and A will quickly fade away as it comes to a stop
            posHold.lpfCutoff = autopilotConfig()->position_cutoff * 0.03f;
        }
        // allow a larder sanity check distance if at higher speed when coming to a stop
        // because the risk of overshoot is greater at higher speed
        posHold.sanityCheckDistance = gpsSol.groundSpeed > 1000 ? gpsSol.groundSpeed : 1000.0f;
        // when the quad stops, the start period is over
        if (posHold.distanceCm < posHold.previousDistanceCm) {
            // reset the target location to the stop point
            currentTargetLocation = gpsSol.llh;
            posHold.lpfCutoff = autopilotConfig()->position_cutoff * 0.01f; // use normal filter values
            resetPositionControlParams();
            posHold.peakInitialGroundspeed = 0.0f;
            posHold.isStarting = false;
        }
    }
    posHold.previousDistanceCm = posHold.distanceCm;
    // intentionally using PT1 gain in PT2 filters to provide a lower effective cutoff
    float pt1Gain = pt1FilterGain(posHold.lpfCutoff, posHold.gpsDataIntervalS);

    // ** Sanity check **

    // primarily to detect flyaway from no Mag or badly oriented Mag
    // but must accept some overshoot at the start, especially if entering at high speed
    if (posHold.distanceCm > posHold.sanityCheckDistance) {
        return false; // must stay within 10m or probably flying away
        // value at this point is a 'best guess' to detect IMU failure in the event the user has no Mag
        // if entering poshold from a stable hover, we would only exceed this if IMU was disoriented
        // if entering poshold at speed, it may overshoot this value and falsely fail, if so need something more complex
    }

    // ** PIDs **

    // separate PID controllers for latitude (NorthSouth or ns) and longitude (EastWest or ew)
    // divide the distance vector into ns and ew parts depending on the bearing to the target point
    float bearingRadians = (bearing / 100.0f) * RAD; // 0-360 degrees, constrained within sin_approx

    const float nsDistance = -cos_approx(bearingRadians) * posHold.distanceCm;
    // Positive when North of the target, negative when when South of target
    const float ewDistance = sin_approx(bearingRadians) * posHold.distanceCm;
    // Negative when East, and positive when West of the target

    // ** P **

    const float nsP = -nsDistance * positionPidCoeffs.Kp;
    const float ewP = ewDistance * positionPidCoeffs.Kp;

    // ** I **

    if (!posHold.isStarting){
        // only accumulate iTerm after completing the start phase
        // perhaps need a timeout on the start phase ?
        posHold.integralNS -= nsDistance * posHold.gpsDataIntervalS;
        posHold.integralEW += ewDistance * posHold.gpsDataIntervalS;
    } else {
        // while moving sticks, slowly leak iTerm away, approx 2s time constant
        const float leak = 1.0f - 0.4f * posHold.gpsDataIntervalS; // gpsDataIntervalS is not more than 1.0s
        posHold.integralNS *= leak;
        posHold.integralEW *= leak;
    }

    const float nsI = posHold.integralNS * positionPidCoeffs.Ki;
    const float ewI = posHold.integralEW * positionPidCoeffs.Ki;

    // ** D ** //

    // get change in distance in NS and EW directions from gps.c using the `GPS_distances` function
    // this gives cleaner velocity data than the module supplied GPS Speed and Heading information
    float deltaDistanceNS; // step size is 1.11cm
    float deltaDistanceEW;
    GPS_distances(&gpsSol.llh, &previousLocation, &deltaDistanceNS, &deltaDistanceEW);
    previousLocation = gpsSol.llh;

    const float velocityNS = deltaDistanceNS * posHold.gpsDataFreqHz; // cm/s, minimum step 11.1 cm/s
    const float velocityEW = deltaDistanceEW * posHold.gpsDataFreqHz;

    float accelerationNS = (velocityNS - posHold.previousVelocityNS) * posHold.gpsDataFreqHz;
    posHold.previousVelocityNS = velocityNS;
    float accelerationEW = (velocityEW - posHold.previousVelocityEW) * posHold.gpsDataFreqHz;
    posHold.previousVelocityEW = velocityEW;

    // scale and filter - filter cutoffs vary during the startup phase
    float nsD = velocityNS * positionPidCoeffs.Kd;
    pt1FilterUpdateCutoff(&velocityNSLpf, pt1Gain);
    nsD = pt1FilterApply(&velocityNSLpf, nsD);

    float ewD = velocityEW * positionPidCoeffs.Kd;
    pt1FilterUpdateCutoff(&velocityEWLpf, pt1Gain);
    ewD = pt1FilterApply(&velocityEWLpf, ewD);

    float nsA = accelerationNS * positionPidCoeffs.Kf;
    pt2FilterUpdateCutoff(&accelerationRollLpf, pt1Gain); // using PT1 gain for stronger cutoff
    nsA = pt2FilterApply(&accelerationRollLpf, nsA);

    float ewA = accelerationEW * positionPidCoeffs.Kf;
    pt2FilterUpdateCutoff(&accelerationPitchLpf, pt1Gain); // using PT1 gain for stronger cutoff
    ewA = pt2FilterApply(&accelerationPitchLpf, ewA);

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

    // ** Rotate pid Sum to quad frame of reference, into pitch and roll **
    float headingRads = (attitude.values.yaw / 10.0f) * RAD; // will be constrained to +/-pi in sin_approx()
    const float sinHeading = sin_approx(headingRads);
    const float cosHeading = cos_approx(headingRads);
    float pidSumRoll = -sinHeading * nsPidSum + cosHeading * ewPidSum;
    float pidSumPitch = cosHeading * nsPidSum + sinHeading * ewPidSum;

    // todo: fix the upsample filtering in pid.c
    // pidSum steps at GPS rate, and needs to change the pid.c upsampling filter for smoothness.

    // ** Final output to pid.c Angle Mode **

    // if a Position Hold deadband is set, and sticks are outside deadband, allow pilot control in angle mode
    autopilotAngle[AI_ROLL] = posHold.sticksActive ? 0.0f : pidSumRoll;
    autopilotAngle[AI_PITCH] = posHold.sticksActive ? 0.0f : pidSumPitch; 

    // Debugs... distances in cm, angles in degrees * 10, velocities cm/2

    if (gyroConfig()->gyro_filter_debug_axis == FD_ROLL) {
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 0, bearing / 10);
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 1, lrintf(ewDistance));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 2, lrintf(ewPidSum * 10));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 3, lrintf(pidSumRoll * 10));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 4, lrintf(ewP * 10));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 5, lrintf(ewI * 10));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 6, lrintf(ewDA * 10));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 7, lrintf(posHold.integralEW * 10));
    } else {
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 0, bearing / 10);
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 1, lrintf(nsDistance));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 2, lrintf(nsPidSum * 10));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 3, lrintf(pidSumPitch * 10));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 4, lrintf(nsP * 10));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 5, lrintf(nsI * 10));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 6, lrintf(nsDA * 10));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 7, lrintf(posHold.integralNS));
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
