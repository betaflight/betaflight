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

#include "flight/imu.h"
#include "flight/position.h"
#include "rx/rx.h"
#include "sensors/gyro.h"

#include "autopilot.h"

#define ALTITUDE_P_SCALE  0.01f
#define ALTITUDE_I_SCALE  0.003f
#define ALTITUDE_D_SCALE  0.01f
#define ALTITUDE_F_SCALE  0.01f
#define POSITION_P_SCALE  0.003f
#define POSITION_I_SCALE  0.0f
#define POSITION_D_SCALE  0.003f
#define POSITION_J_SCALE  0.002f

static pidCoefficient_t altitudePidCoeffs;
static float altitudeI = 0.0f;
static float throttleOut = 0.0f;

static pidCoefficient_t positionPidCoeffs;
static float previousDistancePitch = 0.0f;
static float previousDistanceRoll = 0.0f;
static float previousVelocityPitch = 0.0f;
static float previousVelocityRoll = 0.0f;
static float pitchI = 0.0f;
static float rollI = 0.0f;
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

void resetPositionControl (void) {
    // runs when position hold starts, and while either stick is within deadband
    previousDistancePitch = 0.0f;
    previousVelocityPitch = 0.0f;
    previousDistanceRoll = 0.0f;
    previousVelocityRoll = 0.0f;
    pitchI = 0.0f;
    rollI = 0.0f;
}
void positionControl(gpsLocation_t targetLocation, float deadband) {
    // gpsSol.llh = current gps location
    // get distance and bearing from current location to target location
    // void GPS_distance_cm_bearing(const gpsLocation_t *from, const gpsLocation_t* to, bool dist3d, uint32_t *pDist, int32_t *pBearing)
    uint32_t distanceCm;
    int32_t bearing; // degrees * 100
    static float previousHeading = 0.0f;
    const float gpsDataIntervalS = getGpsDataIntervalSeconds(); // interval for current GPS data value 0.01s to 1.0s
    const float gpsDataIntervalHz = 1.0f / gpsDataIntervalS;
    GPS_distance_cm_bearing(&gpsSol.llh, &targetLocation, false, &distanceCm, &bearing);

    const float errorAngle = (attitude.values.yaw * 10.0f - bearing) / 100.0f;
    float normalisedErrorAngle = fmodf(errorAngle + 360.0f, 360.0f);

    if (normalisedErrorAngle > 180.0f) {
        normalisedErrorAngle -= 360.0f; // Range: -180 to 180
    }

    // Calculate correction proportions for pitch and roll
    const float errorAngleRadians = normalisedErrorAngle * (M_PI / 180.0f);
    const float rollProportion = -sin_approx(errorAngleRadians); // + 1 when target is left, -1 when to right, of the craft
    const float pitchProportion = cos_approx(errorAngleRadians); // + 1 when target is ahead, -1 when behind, the craft

    // todo: sanity check for failure to reduce distance over some time period
    // if no mag, or bad mag, the controller may get incorrect heading data
    // this may lead to responses at the wrong angle
    // and positive feedback and persistently increasing distance, rather than reducing it
    // in wind there could also be some period of time of increasing distance until PIDs build up
    // however if the velocity is  increasing, rather than decreasing
    // then probably the heading is wrong
    // we could do a counter based method like GPS Rescue
    // and terminate the position hold, leaving the craft 'floating' in altitude hold
    // this would need some warning in OSD
    // I can probably craft a sanity check but don't know how to do OSD warnings.

    const float gain = pt1FilterGain(positionLpfCutoffHz, gpsDataIntervalS);

    // ** calculate P, D and J for pitch and roll
    // each axis separately, so that when overshooting, the filter lag doesn't cause problems
    const float distanceRoll = rollProportion * distanceCm;
    // positive distances mean nose towards target, should roll forward (positive roll)
    // we need separate velocity for roll so the filter lag isn't problematic
    float velocityRoll = (distanceRoll - previousDistanceRoll) * gpsDataIntervalHz;
    previousDistanceRoll = distanceRoll;
    // lowpass filter the velocity
    pt1FilterUpdateCutoff(&velocityRollLpf, gain);
    velocityRoll = pt1FilterApply(&velocityRollLpf, velocityRoll);

    float accelerationRoll = (velocityRoll - previousVelocityRoll) * gpsDataIntervalHz; // positive when moving away
    previousVelocityRoll = velocityRoll;
    // lowapss filter the acceleration value again, effectively PT2, it's very noisy
    pt1FilterUpdateCutoff(&accelerationRollLpf, gain);
    accelerationRoll = pt1FilterApply(&accelerationRollLpf, accelerationRoll);

    const float rollP = distanceRoll * positionPidCoeffs.Kp;
    const float rollD = velocityRoll * positionPidCoeffs.Kd;
    const float rollJ = accelerationRoll * positionPidCoeffs.Kf;

    const float distancePitch = pitchProportion * distanceCm;
    // positive distances mean nose towards target, should pitch forward (positive pitch)
    float velocityPitch = (distancePitch - previousDistancePitch) * gpsDataIntervalHz;
    previousDistancePitch = distancePitch;
    // lowpass filter the velocity
    pt1FilterUpdateCutoff(&velocityPitchLpf, gain);
    velocityPitch = pt1FilterApply(&velocityPitchLpf, velocityPitch);

    float accelerationPitch = (velocityPitch - previousVelocityPitch) * gpsDataIntervalHz; // positive when moving away
    previousVelocityPitch = velocityPitch;
    // lowapss filter the acceleration value again, effectively PT2, it's very noisy
    pt1FilterUpdateCutoff(&accelerationPitchLpf, gain);
    accelerationPitch = pt1FilterApply(&accelerationPitchLpf, accelerationPitch);

    const float pitchP = distancePitch * positionPidCoeffs.Kp;
    const float pitchD = velocityPitch * positionPidCoeffs.Kd;
    const float pitchJ = accelerationPitch * positionPidCoeffs.Kf;

    // ** calculate I and rotate if quad yawa

    rollI += distanceRoll * positionPidCoeffs.Ki * gpsDataIntervalS;
    pitchI += distancePitch * positionPidCoeffs.Ki * gpsDataIntervalS;

    // rotate iTerm if heading changes
    const float currentHeading = attitude.values.yaw * 0.1f; // from tenths of a degree to degrees
    float deltaHeading = currentHeading - previousHeading;
    previousHeading = currentHeading;

    // Normalize deltaHeading to range -180 to 180 (in case of small change around North)
    if (deltaHeading > 180.0f) {
        deltaHeading -= 360.0f; // Wrap around if greater than 180
    } else if (deltaHeading < -180.0f) {
        deltaHeading += 360.0f; // Wrap around if less than -180
    }

    float deltaHeadingRadians = deltaHeading * (M_PI / 180.0f); // Convert to radians

    float cosDeltaHeading = cos_approx(deltaHeadingRadians);
    float sinDeltaHeading = sin_approx(deltaHeadingRadians);

    // Cross-rotate pitch and roll iTerm, because
    // iTerm accumulation mostly corrects wind drag and needs to keep its earth direction if the quad yaws.
    const float newPitchI = pitchI * cosDeltaHeading - rollI * sinDeltaHeading;
    const float newRollI = pitchI * sinDeltaHeading + rollI * cosDeltaHeading;

    rollI = newRollI;
    pitchI = newPitchI;

    // add up pTerm, dTerm and jTerm

    const float pidSumRoll = rollP + rollI + rollD + rollJ;
    const float pidSumPitch = pitchP + pitchI + pitchD + pitchJ;

    DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 0, lrintf(normalisedErrorAngle)); //-180 to +180
    DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 1, lrintf(distanceCm));

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
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 2, lrintf(posHoldAngle[AI_ROLL] * 10));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 3, lrintf(posHoldAngle[AI_PITCH] * 10));

    if (gyroConfig()->gyro_filter_debug_axis == FD_ROLL) {
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 4, lrintf(rollP * 10)); // degrees*10
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 5, lrintf(rollI * 10));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 6, lrintf(rollD * 10));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 7, lrintf(rollJ * 10));
    } else {
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 4, lrintf(pitchP * 10)); // degrees*10
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 5, lrintf(pitchI * 10));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 6, lrintf(pitchD * 10));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 7, lrintf(pitchJ * 10));
    }
}

bool isBelowLandingAltitude(void)
{
    return getAltitudeCm() < 100.0f * autopilotConfig()->landing_altitude_m;
}

float getAutopilotThrottle(void)
{
    return throttleOut;
}
