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
#define POSITION_P_SCALE  0.003f
#define POSITION_I_SCALE  0.001f
#define POSITION_D_SCALE  0.004f
#define POSITION_J_SCALE  0.002f

static pidCoefficient_t altitudePidCoeffs;
static float altitudeI = 0.0f;
static float throttleOut = 0.0f;

static pidCoefficient_t positionPidCoeffs;

typedef struct {
    uint32_t distanceCm;
    uint32_t previousDistanceCm;
    float previousDistancePitch;
    float previousDistanceRoll;
    float previousVelocityPitch;
    float previousVelocityRoll;
    float previousHeading;
    float pitchI;
    float rollI;
    bool justStarted;
} posHoldState;

static posHoldState posHold = {
    .distanceCm = 0,
    .previousDistanceCm = 0,
    .previousDistancePitch = 0.0f,
    .previousDistanceRoll = 0.0f,
    .previousVelocityPitch = 0.0f,
    .previousVelocityRoll = 0.0f,
    .previousHeading = 0.0f,
    .pitchI = 0.0f,
    .rollI = 0.0f,
    .justStarted = true,
};

static gpsLocation_t currentTargetLocation = {0, 0, 0};
float posHoldAngle[ANGLE_INDEX_COUNT];

static pt1Filter_t velocityPitchLpf;
static pt1Filter_t accelerationPitchLpf;
static pt1Filter_t velocityRollLpf;
static pt1Filter_t accelerationRollLpf;
static float positionLpfCutoffHz;

void autopilotInit(const autopilotConfig_t *config)
{
    altitudePidCoeffs.Kp = config->altitude_P * ALTITUDE_P_SCALE;
    altitudePidCoeffs.Ki = config->altitude_I * ALTITUDE_I_SCALE;
    altitudePidCoeffs.Kd = config->altitude_D * ALTITUDE_D_SCALE;
    altitudePidCoeffs.Kf = config->altitude_F * ALTITUDE_F_SCALE;
    positionPidCoeffs.Kp = config->position_P * POSITION_P_SCALE;
    positionPidCoeffs.Ki = config->position_I * POSITION_I_SCALE;
    positionPidCoeffs.Kd = config->position_D * POSITION_D_SCALE;
    positionPidCoeffs.Kf = config->position_J * POSITION_J_SCALE; // Kf used for acceleration
    positionLpfCutoffHz = config->position_cutoff / 100.0f;
    const float gain = pt1FilterGain(positionLpfCutoffHz, 0.1f); // assume 10Hz GPS connection at start
    pt1FilterInit(&velocityPitchLpf, gain);
    pt1FilterInit(&accelerationPitchLpf, gain);
    pt1FilterInit(&velocityRollLpf, gain);
    pt1FilterInit(&accelerationRollLpf, gain);
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

void resetPositionControlParams(void) {
    posHold.distanceCm = 0.0f;
    posHold.previousDistanceCm = 0;
    posHold.previousDistanceRoll = 0.0f;
    posHold.previousVelocityRoll = 0.0f;
    posHold.previousDistancePitch = 0.0f;
    posHold.previousVelocityPitch = 0.0f;
    posHold.previousHeading = attitude.values.yaw * 0.1f;
    posHold.pitchI = 0.0f;
    posHold.rollI = 0.0f;
}

void resetPositionControl(gpsLocation_t initialTargetLocation) {
    currentTargetLocation = initialTargetLocation;
    posHold.justStarted = true;
    resetPositionControlParams();
}

bool positionControl(float deadband) {

    // exit if we don't have suitable data
    if (!STATE(GPS_FIX)) {
        return false; // cannot proceed without a GPS location
    }

    if (!canUseGPSHeading && !compassIsHealthy()) {
        return false;
        // If compass is healthy, we must have a Mag, and therefore are OK, even with no GPS Heading
        // If user allows posHold without a Mag, the IMU must be able to get heading from the GPS
    }

    // collect initial data values - gpsSol.llh = current gps location
    int32_t bearing = 0; // degrees * 100
    const float gpsDataIntervalS = getGpsDataIntervalSeconds(); // interval for current GPS data value 0.01s to 1.0s
    const float gpsDataIntervalHz = 1.0f / gpsDataIntervalS;

    // get distance and bearing from current location (gpsSol.llh) to target location
    GPS_distance_cm_bearing(&gpsSol.llh, &currentTargetLocation, false, &posHold.distanceCm, &bearing);

    // at the start, if the quad was moving, it will initially show increasing distance from start point
    // once it has 'stopped' the PIDs will push back towards home, and the distance away will decrease
    // it looks a lot better if we reset the target point to the point that we 'pull up' at
    // otherwise there is a big distance to pull back if we start pos hold while carrying some speed
    if (posHold.justStarted) {
        if (posHold.distanceCm < posHold.previousDistanceCm) {
            currentTargetLocation = gpsSol.llh;
            resetPositionControlParams();
            posHold.justStarted = false;
        } else {
            posHold.previousDistanceCm = posHold.distanceCm;
        }
    }
    const uint8_t startLogger = posHold.justStarted ? 2 : 1;;
    DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 3, startLogger);
    
    // simple (very simple) sanity check
    // primarily to detect flyaway from no Mag or badly oriented Mag
    // TODO - maybe figure how to make a better check by giving more leeway at the start?
    if (posHold.distanceCm > 1000) {
        return false; // must stay within 10m or probably flying away
        // value at this point is a 'best guess' to detect IMU failure in the event the user has no Mag
        // if entering poshold from a stable hover, we would only exceed this if IMU was disoriented
        // if entering poshold at speed, it may overshoot this value and falsely fail, if so need something more complex
    }


    // calculate error angle and normalise range
    const float errorAngle = (attitude.values.yaw * 10.0f - bearing) / 100.0f;
    float normalisedErrorAngle = fmodf(errorAngle + 360.0f, 360.0f);
    if (normalisedErrorAngle > 180.0f) {
        normalisedErrorAngle -= 360.0f; // Range: -180 to 180
    }

    // Calculate distance error proportions for pitch and roll
    const float errorAngleRadians = normalisedErrorAngle * (M_PI / 180.0f);
    const float rollProportion = -sin_approx(errorAngleRadians); // + 1 when target is left, -1 when to right, of the craft
    const float pitchProportion = cos_approx(errorAngleRadians); // + 1 when target is ahead, -1 when behind, the craft

    // set the filter gain used for D and J
    // TO DO - maybe use fixed at GPS data rate?
    const float gain = pt1FilterGain(positionLpfCutoffHz, gpsDataIntervalS);

    // ** calculate P, D and J for pitch and roll axes, independently.
    // TODO for loop by axis

    // roll
    const float distanceRoll = rollProportion * posHold.distanceCm;
    // positive distances mean nose towards target, should roll forward (positive roll)

    // we need separate velocity for roll so the filter lag isn't problematic
    float velocityRoll = (distanceRoll - posHold.previousDistanceRoll) * gpsDataIntervalHz;
    posHold.previousDistanceRoll = distanceRoll;
    // lowpass filter the velocity
    pt1FilterUpdateCutoff(&velocityRollLpf, gain);
    velocityRoll = pt1FilterApply(&velocityRollLpf, velocityRoll);

    float accelerationRoll = (velocityRoll - posHold.previousVelocityRoll) * gpsDataIntervalHz; // positive when moving away
    posHold.previousVelocityRoll = velocityRoll;
    // lowapss filter the acceleration value again, effectively PT2, it's very noisy
    pt1FilterUpdateCutoff(&accelerationRollLpf, gain);
    accelerationRoll = pt1FilterApply(&accelerationRollLpf, accelerationRoll);

    const float rollP = distanceRoll * positionPidCoeffs.Kp;
    const float rollD = velocityRoll * positionPidCoeffs.Kd;
    const float rollJ = accelerationRoll * positionPidCoeffs.Kf;

    // pitch
    const float distancePitch = pitchProportion * posHold.distanceCm;
    // positive distances mean nose towards target, should pitch forward (positive pitch)

    float velocityPitch = (distancePitch - posHold.previousDistancePitch) * gpsDataIntervalHz;
    posHold.previousDistancePitch = distancePitch;
    // lowpass filter the velocity
    pt1FilterUpdateCutoff(&velocityPitchLpf, gain);
    velocityPitch = pt1FilterApply(&velocityPitchLpf, velocityPitch);

    float accelerationPitch = (velocityPitch - posHold.previousVelocityPitch) * gpsDataIntervalHz; // positive when moving away
    posHold.previousVelocityPitch = velocityPitch;
    // lowapss filter the acceleration value again, effectively PT2, it's very noisy
    pt1FilterUpdateCutoff(&accelerationPitchLpf, gain);
    accelerationPitch = pt1FilterApply(&accelerationPitchLpf, accelerationPitch);

    const float pitchP = distancePitch * positionPidCoeffs.Kp;
    const float pitchD = velocityPitch * positionPidCoeffs.Kd;
    const float pitchJ = accelerationPitch * positionPidCoeffs.Kf;

    // ** calculate I and rotate if quad yaws

    // intent: on windy days, accumulate whatever iTerm we need, then rotate it if the quad yaws.
    // useful only on very windy days when P can't quite get there
    // needs to be attenuated towards zero when close to target to avoid overshoot and circling
    // hence cannot completely eliminate position error due to wind, will tend to end up a little bit down-wind

    posHold.rollI += distanceRoll * positionPidCoeffs.Ki * gpsDataIntervalS;
    posHold.pitchI += distancePitch * positionPidCoeffs.Ki * gpsDataIntervalS;

    // rotate iTerm if heading changes
    const float currentHeading = attitude.values.yaw * 0.1f; // from tenths of a degree to degrees
    float deltaHeading = currentHeading - posHold.previousHeading;
    posHold.previousHeading = currentHeading;

    // Normalize deltaHeading to range -180 to 180 (in case of small change around North)
    if (deltaHeading > 180.0f) {
        deltaHeading -= 360.0f; // Wrap around if greater than 180
    } else if (deltaHeading < -180.0f) {
        deltaHeading += 360.0f; // Wrap around if less than -180
    }
    float deltaHeadingRadians = deltaHeading * (M_PI / 180.0f); // Convert to radians

    float cosDeltaHeading = cos_approx(deltaHeadingRadians);
    float sinDeltaHeading = sin_approx(deltaHeadingRadians);

    // rotate pitch and roll iTerm
    const float rotatedRollI = posHold.pitchI * sinDeltaHeading + posHold.rollI * cosDeltaHeading;
    const float rotatedPitchI = posHold.pitchI * cosDeltaHeading - posHold.rollI * sinDeltaHeading;

    posHold.rollI = rotatedRollI;
    posHold.pitchI = rotatedPitchI;

    // add up pid factors
    const float pidSumRoll = rollP + posHold.rollI + rollD + rollJ;
    const float pidSumPitch = pitchP + posHold.pitchI + pitchD + pitchJ;

    // todo: upsample filtering
    // pidSum will have steps at GPS rate, and may require an upsampling filter for smoothness.
    // either at 100Hz by returning these values to pos_hold.c and upsampling to 100hz there,
    // or in pid.c, when angle rate is calculated
    // if done in pid.c, the same upsampler could be used for GPS and PosHold.

    // if sticks are centered, allow autopilot control, otherwise pilot can fly like a mavic
    posHoldAngle[AI_ROLL] = (getRcDeflectionAbs(FD_ROLL) < deadband) ? pidSumRoll : 0.0f;
    posHoldAngle[AI_PITCH] = (getRcDeflectionAbs(FD_PITCH) < deadband) ? pidSumPitch : 0.0f;

    // note:
    // if FLIGHT_MODE(POS_HOLD_MODE):
    // posHoldAngle[] is added to angle setpoint in pid.c, in degrees
    // stick angle setpoint forced to zero within the same deadband via rc.c

        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 0, lrintf(normalisedErrorAngle));
    if (gyroConfig()->gyro_filter_debug_axis == FD_ROLL) {
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 1, lrintf(-distanceRoll));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 2, lrintf(posHoldAngle[AI_ROLL] * 10));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 4, lrintf(rollP * 10)); // degrees*10
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 5, lrintf(posHold.rollI * 10));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 6, lrintf(rollD * 10));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 7, lrintf(rollJ * 10));
    } else {
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 1, lrintf(-distancePitch));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 2, lrintf(posHoldAngle[AI_PITCH] * 10));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 4, lrintf(pitchP * 10)); // degrees*10
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 5, lrintf(posHold.pitchI * 10));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 6, lrintf(pitchD * 10));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 7, lrintf(pitchJ * 10));
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
