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
#define POSITION_P_SCALE  0.0008f
#define POSITION_I_SCALE  0.0002f
#define POSITION_D_SCALE  0.0015f
#define POSITION_A_SCALE  0.0008f

static pidCoefficient_t altitudePidCoeffs;
static pidCoefficient_t positionPidCoeffs;

static float altitudeI = 0.0f;
static float throttleOut = 0.0f;

typedef struct {
    float distanceCm;
    float previousDistanceCm;
    float sanityCheckDistance;
    float previousVelocity;
    float initialHeadingDeg;
    float iTermRoll;
    float iTermPitch;
    bool isStarting;
    float peakInitialGroundspeed;
    float lpfCutoff;
    bool sticksActive;
    float NSIntegral;
    float EWIntegral;
} posHoldState;

static posHoldState posHold = {
    .distanceCm = 0.0f,
    .previousDistanceCm = 0.0f,
    .sanityCheckDistance = 1000.0f,
    .previousVelocity = 0.0f,
    .initialHeadingDeg = 0.0f,
    .iTermRoll = 0.0f,
    .iTermPitch = 0.0f,
    .isStarting = false,
    .peakInitialGroundspeed = 0.0f,
    .lpfCutoff = 1.0f,
    .sticksActive = false,
    .NSIntegral = 0.0f,
    .EWIntegral = 0.0f,
};

static gpsLocation_t currentTargetLocation = {0, 0, 0};
float autopilotAngle[ANGLE_INDEX_COUNT];
static pt1Filter_t velocityPitchLpf;
static pt1Filter_t velocityRollLpf;
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
    pt1FilterInit(&velocityRollLpf, pt1Gain);
    pt1FilterInit(&velocityPitchLpf, pt1Gain);
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

void setSticksActiveStatus(bool areSticksActive)
{
    posHold.sticksActive = areSticksActive;
}

void resetPositionControlParams(void) { // at the start, and while sticks are moving
    posHold.distanceCm = 0.0f;
    posHold.previousDistanceCm = 0.0f;
    posHold.sanityCheckDistance = 1000.0f;
    posHold.previousVelocity = 0.0f;
    posHold.lpfCutoff = autopilotConfig()->position_cutoff * 0.01f;
    const float pt1Gain = pt1FilterGain(posHold.lpfCutoff, 0.1f);
    // reset all lowpass filter accumulators to zero
    pt1FilterInit(&velocityRollLpf, pt1Gain);
    pt1FilterInit(&velocityPitchLpf, pt1Gain);
    pt2FilterInit(&accelerationRollLpf, pt1Gain);
    pt2FilterInit(&accelerationPitchLpf, pt1Gain);
    posHold.isStarting = true;
}

void resetPositionControl(gpsLocation_t initialTargetLocation) { // set only at the start
    currentTargetLocation = initialTargetLocation;
    resetPositionControlParams();
    posHold.iTermRoll = 0.0f;
    posHold.iTermPitch = 0.0f;
    posHold.peakInitialGroundspeed = 0.0f;
    posHold.initialHeadingDeg = attitude.values.yaw * 0.1f;
    posHold.NSIntegral = 0.0f;
    posHold.EWIntegral = 0.0f;
}

void updateTargetLocation(gpsLocation_t newTargetLocation) {
    currentTargetLocation = newTargetLocation;
    resetPositionControlParams(); // don't reset accumulated iTerm
}

bool positionControl(void) {

    // exit if we don't have suitable data
    if (!STATE(GPS_FIX)) {
        return false; // cannot proceed without a GPS location
    }

    if (!canUseGPSHeading
#ifdef USE_MAG
        && !compassIsHealthy() // if no heading info, ie none from Mag *or* GPS-derived, can't continue
#endif
        ) {
        return false;
        // If compass is healthy, we must have a Mag, and therefore are OK, even with no GPS Heading
        // If user allows posHold without a Mag, the IMU must be able to get heading from the GPS
    }

    // collect initial data values - gpsSol.llh = current gps location
    uint32_t distanceCm = 0;
    int32_t bearing = 0; // degrees * 100
    const float gpsDataIntervalS = getGpsDataIntervalSeconds(); // interval for current GPS data value 0.01s to 1.0s
    const float gpsDataFreqHz = 1.0f / gpsDataIntervalS;

    // get distance and bearing from current location (gpsSol.llh) to target location
    GPS_distance_cm_bearing(&gpsSol.llh, &currentTargetLocation, false, &distanceCm, &bearing);

    posHold.distanceCm = (float)distanceCm;
    float bearingDeg = bearing * 0.01f;
    float headingDeg = attitude.values.yaw * 0.1f;

    // at the start, if the quad was moving, it will initially show increasing distance from start point
    // once it has 'stopped' the PIDs will push back towards home, and the distance away will decrease
    // it looks a lot better if we reset the target point to the point that we 'pull up' at
    // otherwise there is a big distance to pull back if we start pos hold while carrying some speed
    if (posHold.isStarting) {
        // first time, or after sticks were centered
        posHold.peakInitialGroundspeed = fmaxf(posHold.peakInitialGroundspeed, gpsSol.groundSpeed);
        // watch for velocity to fall by half
        if (gpsSol.groundSpeed > 0.5f * posHold.peakInitialGroundspeed) {
            // to avoid sudden D and A changes, filter hard
            posHold.lpfCutoff = autopilotConfig()->position_cutoff * 0.003f;
        } else {
            // almost slowed down, get D and A to follow raw signal more closely
            posHold.lpfCutoff = autopilotConfig()->position_cutoff * 0.03f;
        }
        posHold.sanityCheckDistance = gpsSol.groundSpeed > 1000 ? gpsSol.groundSpeed : 1000.0f;
        // watch for quad to slow down, and start moving away
        if (posHold.distanceCm < posHold.previousDistanceCm) {
            // now the craft has 'stopped', reset the location, filter normally
            currentTargetLocation = gpsSol.llh;
            posHold.lpfCutoff = autopilotConfig()->position_cutoff * 0.01f;
            // reset all values but not iTerm
            resetPositionControlParams();
            posHold.initialHeadingDeg = headingDeg; // reset to current heading
            posHold.peakInitialGroundspeed = 0.0f;
            posHold.isStarting = false; // final target is set, no more messing around
        }
    }

    float pt1Gain = pt1FilterGain(posHold.lpfCutoff, gpsDataIntervalS);

    // ** simple (too simple) sanity check **
    // primarily to detect flyaway from no Mag or badly oriented Mag
    // but must accept some overshoot at the start, especially if entering at high speed
    if (posHold.distanceCm > posHold.sanityCheckDistance) {
        return false; // must stay within 10m or probably flying away
        // value at this point is a 'best guess' to detect IMU failure in the event the user has no Mag
        // if entering poshold from a stable hover, we would only exceed this if IMU was disoriented
        // if entering poshold at speed, it may overshoot this value and falsely fail, if so need something more complex
    }

    // calculate error angle and normalise range
    float errorAngle = (headingDeg - bearingDeg);
    float normalisedErrorAngle = fmodf(errorAngle + 360.0f, 360.0f);
    if (normalisedErrorAngle > 180.0f) {
        normalisedErrorAngle -= 360.0f; // Range: -180 to 180
    }

    // Calculate distance error proportions for pitch and roll
    const float errorAngleRadians = normalisedErrorAngle * RAD;
    const float rollProportion = -sin_approx(errorAngleRadians); // + 1 when target is left, -1 when to right, of the craft
    const float pitchProportion = cos_approx(errorAngleRadians); // + 1 when target is ahead, -1 when behind, the craft

    // P
    const float rollP = rollProportion * posHold.distanceCm * positionPidCoeffs.Kp;
    const float pitchP = pitchProportion * posHold.distanceCm * positionPidCoeffs.Kp;

    // derivative and acceleration
    // note: here we just want no velocity, so use gps groundspeed
    // adjust response angle based on drift angle compared to nose of quad
    // ie yaw attitude (angle of quad nose) minus groundcourse (drift direction) both in earth frame

    float errorGroundCourse = (attitude.values.yaw - gpsSol.groundCourse) * 0.1f;
    float normGCE = fmodf(errorGroundCourse + 360.0f, 360.0f);
    if (normGCE > 180.0f) {
        normGCE -= 360.0f; // Range: -180 to 180
    }
    const float normCGERadians = normGCE * RAD;
    const float rollVelProp = sin_approx(normCGERadians); // +1 when drifting rightwards, -1 when drifting leftwards 
    const float pitchVelProp = -cos_approx(normCGERadians); // +1 when drifting backwards, -1 when drifting forwards

    float velocity = gpsSol.groundSpeed;
    float acceleration = (velocity - posHold.previousVelocity) * gpsDataFreqHz; // positive when moving away
    posHold.previousVelocity = velocity;

    // include a target based D element. This is smoother and complements groundcourse measurements.
    float velocityToTarget = (posHold.distanceCm - posHold.previousDistanceCm) * gpsDataFreqHz;
    posHold.previousDistanceCm = posHold.distanceCm;

    // roll
    float velocityRoll = rollVelProp * velocity + velocityToTarget * rollProportion;
    float accelerationRoll = rollVelProp * acceleration;

    // lowpass filters
    pt1FilterUpdateCutoff(&velocityRollLpf, pt1Gain);
    velocityRoll = pt1FilterApply(&velocityRollLpf, velocityRoll);
    pt2FilterUpdateCutoff(&accelerationRollLpf, pt1Gain);
    accelerationRoll = pt2FilterApply(&accelerationRollLpf, accelerationRoll);
    float rollD = velocityRoll * positionPidCoeffs.Kd;
    float rollA = accelerationRoll * positionPidCoeffs.Kf;

    // pitch
    float velocityPitch = pitchVelProp * velocity + velocityToTarget * pitchProportion;
    float accelerationPitch = pitchVelProp * acceleration;

    // lowpass filters
    pt1FilterUpdateCutoff(&velocityPitchLpf, pt1Gain);
    velocityPitch = pt1FilterApply(&velocityPitchLpf, velocityPitch);
    pt2FilterUpdateCutoff(&accelerationPitchLpf, pt1Gain);
    accelerationPitch = pt2FilterApply(&accelerationPitchLpf, accelerationPitch);
    float pitchD = velocityPitch * positionPidCoeffs.Kd;
    float pitchA = accelerationPitch * positionPidCoeffs.Kf;

    // ** test code to handle user yaw inputs ** //
    if (autopilotConfig()->position_allow_yaw) {
        // code that we know is effective for fixing user yaw inputs (none at present lol)
        if (autopilotConfig()->position_test_yaw_fix) {

            // iTerm
            // Note: accumulated iTerm opposes wind, which is a constant earth-referenced vector
            // Hence we accumulate earth referenced iTerm
            // The sign of the iTerm input is determined in relation to the average error bearing angle
            // Finally the abs value of the accumulated iTerm is proportioned to pitch and roll
            // Based on the difference in angle between the nose of the quad and the current error bearing

            if (!posHold.isStarting){
                // only add to iTerm while not actively stopping

                float bearingRadians = bearingDeg * RAD; // 0-360, so constrain to +/- π
                if (bearingRadians > M_PIf) {
                    bearingRadians -= 2 * M_PIf;
                } else if (bearingRadians < -M_PIf) {
                    bearingRadians += 2 * M_PIf;
                }

                const float error = posHold.distanceCm * positionPidCoeffs.Ki * gpsDataIntervalS;
                // NS means NorthSouth in earth frame
                const float NSError = -cos_approx(bearingRadians) * error;
                posHold.NSIntegral += NSError; // simple non-leaky integrator

                const float EWError = sin_approx(bearingRadians) * error;
                posHold.EWIntegral += EWError;

                // averaged correction angle, radians from North, Earth frame of reference
                float EFIntegralAngleRads = atan2_approx(posHold.NSIntegral, posHold.EWIntegral);

                // heading of the quad in radians
                float headingRadians = headingDeg * RAD;

                // get the error angle between quad heading and iTerm vector
                float headingErrorRads = headingRadians - EFIntegralAngleRads;
                // ensure in range +/- π
                while (headingErrorRads > M_PIf) {
                    headingErrorRads -= 2 * M_PIf; // Wrap to the left
                }
                while (headingErrorRads < -M_PIf) {
                    headingErrorRads += 2 * M_PIf; // Wrap to the right
                }
                // get correction factors for roll and pitch based on error angle
                posHold.iTermRoll = -sin_approx(headingErrorRads) * fabsf(posHold.NSIntegral) + cos_approx(headingErrorRads) * fabsf(posHold.EWIntegral); // +1 when iTerm points left, -1 when iTerm points right 
                posHold.iTermPitch = cos_approx(headingErrorRads) * fabsf(posHold.NSIntegral) + sin_approx(headingErrorRads) * fabsf(posHold.EWIntegral); // +1 when iTerm points nose forward, -1 when iTerm should pitch back
                
            } else {
                // while moving sticks, slowly leak iTerm away, approx 3s time constant
                const float leak = 1.0f - 0.25f * gpsDataIntervalS; // assumes gpsDataIntervalS not more than 1.0s
                posHold.NSIntegral *= leak;
                posHold.EWIntegral *= leak;
            }
        }
    }

    // limit sum of D and A because otherwise too aggressive if entering at speed
    float rollDA = rollD + rollA;
    float pitchDA = pitchD + pitchA;
    const float maxDAAngle = 35.0f; // degrees; arbitrary limit.  20 is a bit too low, allows a lot of overshoot
    // to get an angle more than 35 degrees will require P and I
    // ** todo = should this be half of the user-configurable angle_limit?  Or fixed?
    rollDA = constrainf(rollDA, -maxDAAngle, maxDAAngle);
    pitchDA = constrainf(pitchDA, -maxDAAngle, maxDAAngle);

    // add up pid factors
//    const float pidSumRoll = rollP + posHold.iTermRoll + rollDA;
    const float pidSumRoll = rollP + posHold.iTermRoll + rollDA;
    const float pidSumPitch = pitchP + posHold.iTermPitch + pitchDA;

    // todo: upsample filtering
    // pidSum will have steps at GPS rate, and may require an upsampling filter for smoothness.
    // either at 100Hz by returning these values to pos_hold.c and upsampling to 100hz there,
    // or in pid.c, when angle rate is calculated
    // if done in pid.c, the same upsampler could be used for GPS and PosHold.

    // if a deadband is set, and sticks are outside deadband, allow pilot control, otherwise hold position
    autopilotAngle[AI_ROLL] = posHold.sticksActive ? 0.0f : pidSumRoll;
    autopilotAngle[AI_PITCH] = posHold.sticksActive ? 0.0f : pidSumPitch;

    // note:
    // if FLIGHT_MODE(POS_HOLD_MODE):
    // autopilotAngle[] is added to angle setpoint in pid.c, in degrees
    // stick angle setpoint forced to zero within the same deadband via rc.c

    if (gyroConfig()->gyro_filter_debug_axis == FD_ROLL) {
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 0, lrintf(bearingDeg));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 1, lrintf(-posHold.distanceCm * rollProportion));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 2, lrintf(pidSumRoll * 10));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 3, lrintf(velocityToTarget * rollProportion));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 4, lrintf(rollP * 10)); // degrees*10
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 5, lrintf(posHold.iTermRoll * 10));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 6, lrintf(rollDA * 10));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 7, lrintf(posHold.EWIntegral * 10));
    } else {
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 0, lrintf(bearingDeg));;
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 1, lrintf(-posHold.distanceCm * pitchProportion));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 2, lrintf(pidSumPitch * 10));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 3, lrintf(velocityToTarget * pitchProportion));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 4, lrintf(pitchP * 10));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 5, lrintf(posHold.iTermPitch * 10));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 6, lrintf(pitchD * 10));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 7, lrintf(pitchVelProp * velocity));
//        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 7, lrintf(posHold.NSIntegral * 10));
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
