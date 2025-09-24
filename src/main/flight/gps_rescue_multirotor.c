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
#include <math.h>

#include "platform.h"

#ifndef USE_WING
#ifdef USE_GPS_RESCUE

#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/config.h"
#include "drivers/time.h"

#include "fc/core.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/autopilot.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/position.h"

#include "io/gps.h"
#include "rx/rx.h"
#include "pg/autopilot.h"
#include "sensors/acceleration.h"

#include "gps_rescue.h"

typedef enum {
    RESCUE_IDLE,
    RESCUE_INITIALIZE,
    RESCUE_ATTAIN_ALT,
    RESCUE_ROTATE,
    RESCUE_FLY_HOME,
    RESCUE_DESCENT,
    RESCUE_LANDING,
    RESCUE_DO_NOTHING,
    RESCUE_ABORT
} rescuePhase_e;

typedef enum {
    RESCUE_HEALTHY,
    RESCUE_FLYAWAY,
    RESCUE_GPSLOST,
    RESCUE_LOWSATS,
    RESCUE_CRASHFLIP_DETECTED,
    RESCUE_STALLED,
    RESCUE_TOO_CLOSE,
    RESCUE_NO_HOME_POINT
} rescueFailureState_e;

typedef struct {
    float maxAltitudeCm;
    float returnAltitudeCm;
    float targetAltitudeCm;
    float targetAltitudeStepCm;
    float targetVelocityCmS;
    float pitchAngleLimitDeg;
    float rollAngleLimitDeg;
    float descentDistanceM;
    int8_t secondsFailing;
    float yawAttenuator;
    float disarmThreshold;
    float velocityITermAccumulator;
    float velocityPidCutoff;
    float velocityPidCutoffModifier;
    float velocityItermAttenuator;
    float velocityItermRelax;
} rescueIntent_s;

typedef struct {
    float distanceToHomeCm;
    float distanceToHomeM;
    uint16_t groundSpeedCmS;
    int16_t directionToHome;
    bool healthy;
    float errorAngle;
    float gpsDataIntervalSeconds;
    float velocityToHomeCmS;
    float alitutudeStepCm;
    float maxPitchStep;
    float absErrorAngle;
    float imuYawCogGain;
} rescueSensorData_s;

typedef struct {
    rescuePhase_e phase;
    rescueFailureState_e failure;
    rescueSensorData_s sensor;
    rescueIntent_s intent;
    bool isAvailable;
} rescueState_s;

#define GPS_RESCUE_MAX_YAW_RATE          180    // deg/sec max yaw rate
#define GPS_RESCUE_MAX_THROTTLE_ITERM    200    // max iterm value for throttle in degrees * 100
#define GPS_RESCUE_ALLOWED_YAW_RANGE   30.0f   // yaw error must be less than this to enter fly home phase, and to pitch during descend()

static const float taskIntervalSeconds = HZ_TO_INTERVAL(TASK_GPS_RESCUE_RATE_HZ); // i.e. 0.01 s
static float rescueYaw;
float gpsRescueAngle[RP_AXIS_COUNT] = { 0, 0 };
bool magForceDisable = false;
static pt1Filter_t velocityDLpf;
static pt3Filter_t velocityUpsampleLpf;


rescueState_s rescueState;

void gpsRescueInit(void)
{
    float cutoffHz;
    cutoffHz = gpsRescueConfig()->pitchCutoffHz / 100.0f;
    rescueState.intent.velocityPidCutoff = cutoffHz;
    rescueState.intent.velocityPidCutoffModifier = 1.0f;
    pt1FilterInitLPF(&velocityDLpf, cutoffHz, 1.0f);
    cutoffHz *= 4.0f;
    pt3FilterInitLPF(&velocityUpsampleLpf, cutoffHz, taskIntervalSeconds);
}

static void rescueStart(void)
{
    rescueState.phase = RESCUE_INITIALIZE;
}

static void rescueStop(void)
{
    rescueState.phase = RESCUE_IDLE;
}

// Things that need to run when GPS Rescue is enabled, and while armed, but while there is no Rescue in place
static void setReturnAltitude(bool newGpsData)
{
    // Hold maxAltitude at zero while disarmed, but if set_home_point_once is true, hold maxAlt until power cycled
    if (!ARMING_FLAG(ARMED) && !gpsConfig()->gps_set_home_point_once) {
        rescueState.intent.maxAltitudeCm = 0.0f;
        return;
    }

    // While armed, but not during the rescue, update the max altitude value
    rescueState.intent.maxAltitudeCm = fmaxf(getAltitudeCm(), rescueState.intent.maxAltitudeCm);

    if (newGpsData) {
        // set the target altitude to the current altitude.
        rescueState.intent.targetAltitudeCm = getAltitudeCm();

        // Intended descent distance for rescues that start outside the minStartDistM distance
        // Set this to the user's intended descent distance, but not more than half the distance to home to ensure some fly home time
        rescueState.intent.descentDistanceM = fminf(0.5f * rescueState.sensor.distanceToHomeM, gpsRescueConfig()->descentDistanceM);

        const float initialClimbCm = gpsRescueConfig()->initialClimbM * 100.0f;
        switch (gpsRescueConfig()->altitudeMode) {
            case GPS_RESCUE_ALT_MODE_FIXED:
                rescueState.intent.returnAltitudeCm = gpsRescueConfig()->returnAltitudeM * 100.0f;
                break;
            case GPS_RESCUE_ALT_MODE_CURRENT:
                // climb above current altitude, but always return at least initial height above takeoff point, in case current altitude was negative
                rescueState.intent.returnAltitudeCm = fmaxf(initialClimbCm, getAltitudeCm() + initialClimbCm);
                break;
            case GPS_RESCUE_ALT_MODE_MAX:
            default:
                rescueState.intent.returnAltitudeCm = rescueState.intent.maxAltitudeCm + initialClimbCm;
                break;
        }
    }
}

static void rescueAttainPosition(bool newGpsData)
{
    // runs at 100hz, but only updates RPYT settings when new GPS Data arrives and when not in idle phase.
    static float previousVelocityError = 0.0f;
    static float velocityI = 0.0f;
    switch (rescueState.phase) {
    case RESCUE_IDLE:
        // values to be returned when no rescue is active
        gpsRescueAngle[AI_PITCH] = 0.0f;
        gpsRescueAngle[AI_ROLL] = 0.0f;
        return;
    case RESCUE_INITIALIZE:
        // Initialize internal variables each time GPS Rescue is started
        previousVelocityError = 0.0f;
        velocityI = 0.0f;
        resetAltitudeControl();
        rescueState.intent.disarmThreshold = gpsRescueConfig()->disarmThreshold * 0.1f;
        rescueState.sensor.imuYawCogGain = 1.0f;
        return;
    case RESCUE_DO_NOTHING:
        // 20s of hover for switch induced sanity failures to allow time to recover
        gpsRescueAngle[AI_PITCH] = 0.0f;
        gpsRescueAngle[AI_ROLL] = 0.0f;
        return;
     default:
        break;
    }

    /**
        Altitude (throttle) controller
    */
    altitudeControl(rescueState.intent.targetAltitudeCm, taskIntervalSeconds, rescueState.intent.targetAltitudeStepCm);

    /**
        Heading / yaw controller
    */
    // simple yaw P controller with roll mixed in.
    // attitude.values.yaw is set by imuCalculateEstimatedAttitude() and is updated from GPS while groundspeed exceeds 2 m/s
    // below 2m/s groundspeed, the IMU uses gyro to estimate yaw attitude change from previous values
    // above 2m/s, GPS course over ground us ysed to 'correct' the IMU heading
    // if the course over ground, due to wind or pre-exiting movement, is different from the attitude of the quad, the GPS correction will be less accurate
    // the craft should not return much less than 5m/s during the rescue or the GPS corrections may be inaccurate.
    // the faster the return speed, the more accurate the IMU will be, but the consequences of IMU error at the start are greater
    // A compass (magnetometer) is vital for accurate GPS rescue at slow speeds, but must be calibrated and validated
    // WARNING:  Some GPS units give false Home values!  Always check the arrow points to home on leaving home.
    rescueYaw = rescueState.sensor.errorAngle * gpsRescueConfig()->yawP * rescueState.intent.yawAttenuator / 10.0f;
    rescueYaw = constrainf(rescueYaw, -GPS_RESCUE_MAX_YAW_RATE, GPS_RESCUE_MAX_YAW_RATE);
    // rescueYaw is the yaw rate in deg/s to correct the heading error

    // *** mix in some roll.  very important for heading tracking, since a yaw rate means the quad has drifted sideways
    const float rollMixAttenuator = constrainf(1.0f - fabsf(rescueYaw) * 0.01f, 0.0f, 1.0f);
    // less roll at higher yaw rates, no roll at 100 deg/s of yaw
    const float rollAdjustment = -rescueYaw * gpsRescueConfig()->rollMix * rollMixAttenuator;
    // if rollMix = 100, the roll:yaw ratio is 1:1 at small angles, reducing linearly to zero when the yaw rate is 100 deg/s
    // when gpsRescueConfig()->rollMix is zero, there is no roll adjustment
    // rollAdjustment is degrees * 100
    // note that the roll element has the opposite sign to the yaw element *before* GET_DIRECTION
    const float rollLimit = 100.0f * rescueState.intent.rollAngleLimitDeg;
    gpsRescueAngle[AI_ROLL] = constrainf(rollAdjustment, -rollLimit, rollLimit);
    // gpsRescueAngle is added to the normal roll Angle Mode corrections in pid.c

    rescueYaw *= GET_DIRECTION(rcControlsConfig()->yaw_control_reversed);
    // rescueYaw is the yaw rate in deg/s to correct the heading error

    DEBUG_SET(DEBUG_GPS_RESCUE_HEADING, 5, rollMixAttenuator);          // 0-1 to suppress roll adjustments at higher yaw rates
    DEBUG_SET(DEBUG_GPS_RESCUE_HEADING, 6, gpsRescueAngle[AI_ROLL]);    // roll added in degrees*100
    DEBUG_SET(DEBUG_GPS_RESCUE_HEADING, 7, rescueYaw);                  // the yaw rate in deg/s to correct a yaw error
    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 7, gpsRescueAngle[AI_ROLL]);  // roll added in degrees*100

    /**
        Pitch / velocity controller
    */
    static float pitchAdjustment = 0.0f;
    if (newGpsData) {

        const float sampleIntervalNormaliseFactor = rescueState.sensor.gpsDataIntervalSeconds * 10.0f;

        const float velocityError = rescueState.intent.targetVelocityCmS - rescueState.sensor.velocityToHomeCmS;
        // velocityError is in cm per second, positive means too slow.
        // NB positive pitch setpoint means nose down.
        // target velocity can be very negative leading to large error before the start, with overshoot

        // P component
        const float velocityP = velocityError * gpsRescueConfig()->velP;

        // I component
        velocityI += 0.01f * gpsRescueConfig()->velI * velocityError * sampleIntervalNormaliseFactor * rescueState.intent.velocityItermRelax;
        // velocityItermRelax is a time-based factor, 0->1 with time constant of 1s from when we start to fly home
        // avoids excess iTerm accumulation during the initial acceleration phase and during descent.

        velocityI *= rescueState.intent.velocityItermAttenuator;
        // used to minimise iTerm windup during IMU error states and iTerm overshoot in the descent phase
        // also, if we over-fly the home point, we need to re-accumulate iTerm from zero, not the previously accumulated value

        const float pitchAngleLimit = rescueState.intent.pitchAngleLimitDeg * 100.0f;
        const float velocityILimit = 0.5f * pitchAngleLimit;
        // I component alone cannot exceed half the max pitch angle
        velocityI = constrainf(velocityI, -velocityILimit, velocityILimit);

        // D component
        float velocityD = ((velocityError - previousVelocityError) / sampleIntervalNormaliseFactor);
        previousVelocityError = velocityError;
        velocityD *= gpsRescueConfig()->velD;
        DEBUG_SET(DEBUG_GPS_RESCUE_VELOCITY, 5, lrintf(velocityD)); // velocity D before lowpass smoothing
        // smooth the D steps
        const float cutoffHz = rescueState.intent.velocityPidCutoff * rescueState.intent.velocityPidCutoffModifier;
        // note that this cutoff is increased up to 2x as we get closer to landing point in descend()
        pt1FilterCoeffsLPF(&velocityDLpf.coeffs, cutoffHz, rescueState.sensor.gpsDataIntervalSeconds);
        velocityD = pt1FilterApply(&velocityDLpf, velocityD);

        pitchAdjustment = velocityP + velocityI + velocityD;
        // limit to maximum allowed angle
        pitchAdjustment = constrainf(pitchAdjustment, -pitchAngleLimit, pitchAngleLimit);

        // pitchAdjustment is the absolute Pitch angle adjustment value in degrees * 100
        // it gets added to the normal level mode Pitch adjustments in pid.c
        DEBUG_SET(DEBUG_GPS_RESCUE_VELOCITY, 0, lrintf(velocityP));
        DEBUG_SET(DEBUG_GPS_RESCUE_VELOCITY, 1, lrintf(velocityD));
        DEBUG_SET(DEBUG_GPS_RESCUE_VELOCITY, 4, lrintf(velocityI));
        DEBUG_SET(DEBUG_GPS_RESCUE_VELOCITY, 6, lrintf(rescueState.intent.velocityItermRelax)); // factor attenuates velocity iTerm by multiplication
        DEBUG_SET(DEBUG_GPS_RESCUE_VELOCITY, 7, lrintf(pitchAdjustment)); // rescue pitch angle in degrees * 100
    }

    // if initiated too close, and in the climb phase, pitch forward in whatever direction the nose is oriented until min distance away
    // intent is to get far enough away that, with an IMU error, the quad will have enough distance to home to correct that error in the fly home phase
    if (rescueState.phase == RESCUE_ATTAIN_ALT && rescueState.sensor.distanceToHomeM < gpsRescueConfig()->minStartDistM) {
        pitchAdjustment = 1500.0f; // 15 degree pitch forward
    }

    // upsampling and smoothing of pitch angle steps
    float pitchAdjustmentFiltered = pt3FilterApply(&velocityUpsampleLpf, pitchAdjustment);

    gpsRescueAngle[AI_PITCH] = pitchAdjustmentFiltered;
    // this angle gets added to the normal pitch Angle Mode control values in pid.c - will be seen in pitch setpoint

    DEBUG_SET(DEBUG_GPS_RESCUE_VELOCITY, 3, lrintf(rescueState.intent.targetVelocityCmS)); // target velocity to home
    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 1, lrintf(rescueState.intent.targetVelocityCmS)); // target velocity to home
}

static void performSanityChecks(void)
{
    static timeUs_t previousTimeUs = 0; // Last time Stalled/LowSat was checked
    static float prevAltitudeCm = 0.0f; // to calculate ascent or descent change
    static float prevTargetAltitudeCm = 0.0f; // to calculate ascent or descent target change
    static float previousDistanceToHomeCm = 0.0f; // to check that we are returning
    static int8_t secondsLowSats = 0; // Minimum sat detection
    static int8_t secondsDoingNothing; // Limit on doing nothing
    const timeUs_t currentTimeUs = micros();

    if (rescueState.phase == RESCUE_IDLE) {
        rescueState.failure = RESCUE_HEALTHY;
        return;
    } else if (rescueState.phase == RESCUE_INITIALIZE) {
        // Initialize these variables each time a GPS Rescue is started
        previousTimeUs = currentTimeUs;
        prevAltitudeCm = getAltitudeCm();
        prevTargetAltitudeCm = rescueState.intent.targetAltitudeCm;
        previousDistanceToHomeCm = rescueState.sensor.distanceToHomeCm;
        secondsLowSats = 0;
        secondsDoingNothing = 0;
    }

    // Handle events that set a failure mode to other than healthy.
    // Disarm via Abort when sanity on, or for hard Rx loss in FS_ONLY mode
    // Otherwise allow 20s of semi-controlled descent with impact disarm detection
    const bool hardFailsafe = !isRxReceivingSignal();

    if (rescueState.failure != RESCUE_HEALTHY) {
        // Default to 20s semi-controlled descent with impact detection, then abort
        rescueState.phase = RESCUE_DO_NOTHING;

        switch(gpsRescueConfig()->sanityChecks) {
        case RESCUE_SANITY_ON:
            rescueState.phase = RESCUE_ABORT;
            break;
        case RESCUE_SANITY_FS_ONLY:
            if (hardFailsafe) {
                rescueState.phase = RESCUE_ABORT;
            }
            break;
        default:
            // even with sanity checks off,
            // override when Allow Arming without Fix is enabled without GPS_FIX_HOME and no Control link available.
            if (gpsRescueConfig()->allowArmingWithoutFix && !STATE(GPS_FIX_HOME) && hardFailsafe) {
                rescueState.phase = RESCUE_ABORT;
            }
        }
    }

    // Crash detection is enabled in all rescues.  If triggered, immediately disarm.
    if (crashRecoveryModeActive()) {
        setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
        disarm(DISARM_REASON_CRASH_PROTECTION);
        rescueStop();
    }

    // Check if GPS comms are healthy
    // ToDo - check if we have an altitude reading; if we have Baro, we can use Landing mode for controlled descent without GPS
    if (!rescueState.sensor.healthy) {
        rescueState.failure = RESCUE_GPSLOST;
    }

    //  Things that should run at a low refresh rate (such as flyaway detection, etc) will be checked at 1Hz
    const timeDelta_t dTime = cmpTimeUs(currentTimeUs, previousTimeUs);
    if (dTime < 1000000) { //1hz
        return;
    }
    previousTimeUs = currentTimeUs;

    // checks that we are getting closer to home.
    // if the quad is stuck, or if GPS data packets stop, there will be no change in distance to home
    // we can't use rescueState.sensor.currentVelocity because it will be held at the last good value if GPS data updates stop
    if (rescueState.phase == RESCUE_FLY_HOME) {
        const float velocityToHomeCmS = previousDistanceToHomeCm - rescueState.sensor.distanceToHomeCm; // cm/s
        previousDistanceToHomeCm = rescueState.sensor.distanceToHomeCm;
        rescueState.intent.secondsFailing += (velocityToHomeCmS < 0.1f * rescueState.intent.targetVelocityCmS) ? 1 : -1;
        rescueState.intent.secondsFailing = constrain(rescueState.intent.secondsFailing, 0, 30);
        if (rescueState.intent.secondsFailing >= 30) {
#ifdef USE_MAG
            //If there is a mag and has not been disabled, we have to assume is healthy and has been used in imu.c
            if (sensors(SENSOR_MAG) && gpsRescueConfig()->useMag && !magForceDisable) {
                //Try again with mag disabled
                magForceDisable = true;
                rescueState.intent.secondsFailing = 0;
            } else
#endif
            {
                rescueState.failure = RESCUE_FLYAWAY;
            }
        }
    }

    secondsLowSats += (!STATE(GPS_FIX) || (gpsSol.numSat < GPS_MIN_SAT_COUNT)) ? 1 : -1;
    secondsLowSats = constrain(secondsLowSats, 0, 10);

    if (secondsLowSats == 10) {
        rescueState.failure = RESCUE_LOWSATS;
    }

    // These conditions ignore sanity mode settings, and apply in all rescues, to handle getting stuck in a climb or descend

    const float actualAltitudeChange = getAltitudeCm() - prevAltitudeCm;
    // ** possibly could use getAltitudeDerivative() for  for actual altitude change, though it is smoothed
    const float targetAltitudeChange = rescueState.intent.targetAltitudeCm - prevTargetAltitudeCm;
    const float ratio = actualAltitudeChange / targetAltitudeChange;
    prevAltitudeCm = getAltitudeCm();
    prevTargetAltitudeCm = rescueState.intent.targetAltitudeCm;

    switch (rescueState.phase) {
    case RESCUE_LANDING:
        rescueState.intent.secondsFailing += ratio > 0.5f ? -1 : 1;
        rescueState.intent.secondsFailing = constrain(rescueState.intent.secondsFailing, 0, 10);
        if (rescueState.intent.secondsFailing >= 10) {
            rescueState.phase = RESCUE_ABORT;
            // Landing mode shouldn't take more than 10s
        }
        break;
    case RESCUE_ATTAIN_ALT:
    case RESCUE_DESCENT:
        rescueState.intent.secondsFailing += ratio > 0.5f ? -1 : 1;
        rescueState.intent.secondsFailing = constrain(rescueState.intent.secondsFailing, 0, 10);
        if (rescueState.intent.secondsFailing >= 10) {
            rescueState.phase = RESCUE_LANDING;
            rescueState.intent.secondsFailing = 0;
            // if can't climb, or slow descending, enable impact detection and time out in 10s
        }
        break;
    case RESCUE_DO_NOTHING:
        secondsDoingNothing = MIN(secondsDoingNothing + 1, 20);
        if (secondsDoingNothing >= 20) {
            rescueState.phase = RESCUE_ABORT;
            // time-limited semi-controlled fall with impact detection
        }
        break;
    default:
        // do nothing
        break;
    }

    DEBUG_SET(DEBUG_RTH, 2, (rescueState.failure * 10 + rescueState.phase));
    DEBUG_SET(DEBUG_RTH, 3, (rescueState.intent.secondsFailing * 100 + secondsLowSats));
}

static void sensorUpdate(bool newGpsData)
{
    static float prevDistanceToHomeCm = 0.0f;

    const float altitudeCurrentCm = getAltitudeCm();
    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 2, lrintf(altitudeCurrentCm));
    DEBUG_SET(DEBUG_GPS_RESCUE_HEADING, 0, rescueState.sensor.groundSpeedCmS);  // groundspeed cm/s
    DEBUG_SET(DEBUG_GPS_RESCUE_HEADING, 1, gpsSol.groundCourse);                // degrees * 10
    DEBUG_SET(DEBUG_GPS_RESCUE_HEADING, 2, attitude.values.yaw);                // degrees * 10
    DEBUG_SET(DEBUG_GPS_RESCUE_HEADING, 3, rescueState.sensor.directionToHome); // computed from current GPS position in relation to home
    rescueState.sensor.healthy = gpsIsHealthy();

    rescueState.sensor.directionToHome = GPS_directionToHome; // extern value from gps.c using current position relative to home
    rescueState.sensor.errorAngle = (attitude.values.yaw - rescueState.sensor.directionToHome) / 10.0f;
    // both attitude and direction are in degrees * 10, errorAngle is degrees
    if (rescueState.sensor.errorAngle <= -180) {
        rescueState.sensor.errorAngle += 360;
    } else if (rescueState.sensor.errorAngle > 180) {
        rescueState.sensor.errorAngle -= 360;
    }
    rescueState.sensor.absErrorAngle = fabsf(rescueState.sensor.errorAngle);

    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 4, lrintf(attitude.values.yaw));                 // estimated heading of the quad (direction nose is pointing in)
    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 5, lrintf(rescueState.sensor.directionToHome));  // angle to home derived from GPS location and home position

    if (!newGpsData) {
        return;
        // GPS ground speed, velocity and distance to home will be held at last good values if no new packets
    }

    rescueState.sensor.distanceToHomeCm = GPS_distanceToHomeCm;
    rescueState.sensor.distanceToHomeM = rescueState.sensor.distanceToHomeCm / 100.0f;
    rescueState.sensor.groundSpeedCmS = gpsSol.groundSpeed; // cm/s

    rescueState.sensor.gpsDataIntervalSeconds = getGpsDataIntervalSeconds();
    // Range from 50ms (20hz) to 2500ms (0.4Hz). Intended to cover common GPS data rates and exclude unusual values.

    rescueState.sensor.velocityToHomeCmS = ((prevDistanceToHomeCm - rescueState.sensor.distanceToHomeCm) * getGpsDataFrequencyHz());
    // positive = towards home.  First value is useless since prevDistanceToHomeCm was zero.
    prevDistanceToHomeCm = rescueState.sensor.distanceToHomeCm;

    DEBUG_SET(DEBUG_ATTITUDE, 4, rescueState.sensor.velocityToHomeCmS); // velocity to home

    // when there is a flyaway due to IMU disorientation, increase IMU yaw CoG gain, and reduce max pitch angle
    if (gpsRescueConfig()->groundSpeedCmS) {
        // calculate a factor that can reduce pitch angle when flying away
        const float rescueGroundspeed = gpsRescueConfig()->imuYawGain * 100.0f; // in cm/s, imuYawGain is m/s groundspeed
        // rescueGroundspeed is effectively a normalising gain factor for the magnitude of the groundspeed error
        // a higher value reduces groundspeedErrorRatio, making the radius wider and reducing the circling behaviour

        const float groundspeedErrorRatio = fabsf(rescueState.sensor.groundSpeedCmS - rescueState.sensor.velocityToHomeCmS) / rescueGroundspeed;
        // 0 if groundspeed = velocity to home, or both are zero
        // 1 if forward velocity is zero but sideways speed is imuYawGain in m/s
        // 2 if moving backwards at imuYawGain m/s, 4 if moving backwards at 2* imuYawGain m/s, etc

        DEBUG_SET(DEBUG_ATTITUDE, 5, groundspeedErrorRatio * 100);

        rescueState.intent.velocityItermAttenuator = 4.0f / (groundspeedErrorRatio + 4.0f);
        // 1 if groundspeedErrorRatio = 0, falling to 2/3 if groundspeedErrorRatio = 2, 0.5 if groundspeedErrorRatio = 4, etc
        // limit (essentially prevent) velocity iTerm accumulation whenever there is a meaningful groundspeed error
        // this is a crude but simple way to prevent iTerm windup when recovering from an IMU error
        // the magnitude of the effect will be less at low GPS data rates, since there will be fewer multiplications per second
        // but for our purposes this should not matter much, our intent is to severely attenuate iTerm
        // if, for example, we had a 90 degree attitude error, groundspeedErrorRatio = 1, invGroundspeedError = 0.8,
        // after 10 iterations, iTerm is 0.1 of what it would have been
        // also is useful in blocking iTerm accumulation if we overshoot the landing point

        const float pitchForwardAngle = (gpsRescueAngle[AI_PITCH] > 0.0f) ? fminf(gpsRescueAngle[AI_PITCH] / 3000.0f, 2.0f) : 0.0f;
        // pitchForwardAngle is positive early in a rescue, and associates with a nose forward ground course
        // note: gpsRescueAngle[AI_PITCH] is in degrees * 100, and is halved when the IMU is 180 wrong
        // pitchForwardAngle is 0 when flat
        // pitchForwardAngle is 0.5 if pitch angle is 15 degrees (ie with rescue angle of 30 and 180deg IMU error)
        // pitchForwardAngle is 1.0 if pitch angle is 30 degrees (ie with rescue angle of 60 and 180deg IMU error)
        // pitchForwardAngle is 2.0 if pitch angle is 60 degrees and flying towards home (unlikely to be sustained at that angle)

        DEBUG_SET(DEBUG_ATTITUDE, 6, pitchForwardAngle * 100.0f);

        if (rescueState.phase != RESCUE_FLY_HOME) {
            // prevent IMU disorientation arising from drift during climb, rotate or do nothing phases, which have zero pitch angle
            // in descent, or too close, increase IMU yaw gain as pitch angle increases
            rescueState.sensor.imuYawCogGain = pitchForwardAngle;
        } else {
            rescueState.sensor.imuYawCogGain = pitchForwardAngle + fminf(groundspeedErrorRatio, 3.5f);
            // imuYawCogGain will be more positive at higher pitch angles and higher groundspeed errors
            // imuYawCogGain will be lowest (close to zero) at lower pitch angles and when flying straight towards home
        }
    }

    DEBUG_SET(DEBUG_GPS_RESCUE_VELOCITY, 2, lrintf(rescueState.sensor.velocityToHomeCmS));
    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 0, lrintf(rescueState.sensor.velocityToHomeCmS));
}

// This function flashes "RESCUE N/A" in the OSD if:
// 1. sensor healthy - GPS data is being received.
// 2. GPS has a 3D fix.
// 3. GPS number of satellites is greater than or equal to the minimum configured satellite count.
// Note 1: cannot arm without the required number of sats
// hence this flashing indicates that after having enough sats, we now have below the minimum and the rescue likely would fail
// Note 2: this function does not take into account the distance from home
// The sanity checks are independent, this just provides the OSD warning
static bool checkGPSRescueIsAvailable(void)
{
    static timeUs_t previousTimeUs = 0; // Last time LowSat was checked
    const timeUs_t currentTimeUs = micros();
    static int8_t secondsLowSats = 0; // Minimum sat detection
    static bool lowsats = false;
    static bool noGPSfix = false;
    bool result = true;

    if (!gpsIsHealthy() || !STATE(GPS_FIX_HOME)) {
        return false;
    }

    //  Things that should run at a low refresh rate >> ~1hz
    const timeDelta_t dTime = cmpTimeUs(currentTimeUs, previousTimeUs);
    if (dTime < 1000000) { //1hz
        if (noGPSfix || lowsats) {
            result = false;
        }
        return result;
    }

    previousTimeUs = currentTimeUs;

    if (!STATE(GPS_FIX)) {
        result = false;
        noGPSfix = true;
    } else {
        noGPSfix = false;
    }

    secondsLowSats = constrain(secondsLowSats + ((gpsSol.numSat < GPS_MIN_SAT_COUNT) ? 1 : -1), 0, 2);
    if (secondsLowSats == 2) {
        lowsats = true;
        result = false;
    } else {
        lowsats = false;
    }

    return result;
}

static void disarmOnImpact(void)
{
    if (acc.accMagnitude > rescueState.intent.disarmThreshold) {
        setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
        disarm(DISARM_REASON_GPS_RESCUE);
        rescueStop();
    }
}

static void descend(bool newGpsData)
{
    if (newGpsData) {
        // consider landing area to be a circle half landing height around home, to avoid overshooting home point
        const float distanceToLandingAreaM = rescueState.sensor.distanceToHomeM - (0.5f * autopilotConfig()->landingAltitudeM);
        const float proximityToLandingArea = constrainf(distanceToLandingAreaM / rescueState.intent.descentDistanceM, 0.0f, 1.0f);

        // increase the velocity lowpass filter cutoff for more aggressive responses when descending, especially close to home
        // 1.5x when starting descent, 2.5x (smoother) when almost landed
        rescueState.intent.velocityPidCutoffModifier = 2.5f - proximityToLandingArea;

        // reduce target velocity as we get closer to home. Zero within 2m of home, reducing risk of overshooting.
        rescueState.intent.targetVelocityCmS = gpsRescueConfig()->groundSpeedCmS * proximityToLandingArea;

        // attenuate velocity target unless pointing towards home, to minimise circling behaviour during overshoots
        if (rescueState.sensor.absErrorAngle > GPS_RESCUE_ALLOWED_YAW_RANGE) {
            rescueState.intent.targetVelocityCmS = 0;
        } else {
            rescueState.intent.targetVelocityCmS *= (GPS_RESCUE_ALLOWED_YAW_RANGE - rescueState.sensor.absErrorAngle) / GPS_RESCUE_ALLOWED_YAW_RANGE;
        }

        // attenuate velocity iterm towards zero as we get closer to the landing area
        rescueState.intent.velocityItermAttenuator = fminf(proximityToLandingArea, rescueState.intent.velocityItermAttenuator);

        // full pitch angle available all the time
        rescueState.intent.pitchAngleLimitDeg = gpsRescueConfig()->maxRescueAngle;

        // limit roll angle to half the allowed pitch angle and attenuate when closer to home
        // keep some roll when at the landing circle distance to avoid endless circling
        const float proximityToHome = constrainf(rescueState.sensor.distanceToHomeM / rescueState.intent.descentDistanceM, 0.0f, 1.0f);
        rescueState.intent.rollAngleLimitDeg = 0.5f * rescueState.intent.pitchAngleLimitDeg * proximityToHome;
    }

    // ensure we have full yaw authority in case we entered descent mode without enough time in fly home to acquire it gracefully
    rescueState.intent.yawAttenuator = 1.0f;

    // set the altitude step, considering the interval between altitude readings and the descent rate
    float altitudeStepCm = taskIntervalSeconds * gpsRescueConfig()->descendRate;

    // at or below 10m: descend at 0.6x set value; above 10m, descend faster, to max 3.0x at 50m
    altitudeStepCm *= scaleRangef(constrainf(rescueState.intent.targetAltitudeCm, 1000, 5000), 1000, 5000, 0.6f, 3.0f);

    rescueState.intent.targetAltitudeStepCm = -altitudeStepCm;
    rescueState.intent.targetAltitudeCm -= altitudeStepCm;
}

static void initialiseRescueValues (void)
{
    rescueState.intent.secondsFailing = 0; // reset the sanity check timer
    rescueState.intent.yawAttenuator = 0.0f; // no yaw in the climb
    rescueState.intent.targetVelocityCmS = rescueState.sensor.velocityToHomeCmS; // avoid snap from D at the start
    rescueState.intent.rollAngleLimitDeg = 0.0f; // no roll until flying home
    rescueState.intent.velocityPidCutoffModifier = 1.0f; // normal velocity lowpass filter cutoff
    rescueState.intent.pitchAngleLimitDeg = 0.0f; // force pitch adjustment to zero - level mode will level out
    rescueState.intent.velocityItermAttenuator = 1.0f; // allow iTerm to accumulate normally unless constrained by IMU error or descent phase
    rescueState.intent.velocityItermRelax = 0.0f; // but don't accumulate any at the start, not until fly home
    rescueState.intent.targetAltitudeStepCm = 0.0f;
}

void gpsRescueUpdate(void)
// runs at gpsRescueTaskIntervalSeconds, and runs whether or not rescue is active
{
    static uint16_t gpsStamp = 0;
    bool newGpsData = gpsHasNewData(&gpsStamp);

    if (!FLIGHT_MODE(GPS_RESCUE_MODE)) {
        rescueStop(); // sets phase to RESCUE_IDLE; does nothing else.  RESCUE_IDLE tasks still run.
    } else if (FLIGHT_MODE(GPS_RESCUE_MODE) && rescueState.phase == RESCUE_IDLE) {
        rescueStart(); // sets phase to rescue_initialise if we enter GPS Rescue mode while idle
        rescueAttainPosition(false); // Initialise basic parameters when a Rescue starts (can't initialise sensor data reliably)
        performSanityChecks(); // Initialises sanity check values when a Rescue starts
    }
    // Will now be in RESCUE_INITIALIZE mode, if just entered Rescue while IDLE, otherwise stays IDLE

    sensorUpdate(newGpsData); // always get latest GPS and Altitude data, update ascend and descend rates

    static bool returnAltitudeLow = true;
    static bool initialVelocityLow = true;
    rescueState.isAvailable = checkGPSRescueIsAvailable();

    switch (rescueState.phase) {
    case RESCUE_IDLE:
        // in Idle phase = NOT in GPS Rescue
        // update the return altitude and descent distance values, to have valid settings immediately they are needed
        setReturnAltitude(newGpsData);
        break;
        // sanity checks are bypassed in IDLE mode; instead, failure state is always initialised to HEALTHY
        // target altitude is always set to current altitude.

    case RESCUE_INITIALIZE:
        // Things that should be done at the start of a Rescue
        if (!STATE(GPS_FIX_HOME)) {
            // we didn't get a home point on arming
            rescueState.failure = RESCUE_NO_HOME_POINT;
            // will result in a disarm via the sanity check system, or float around if switch induced
        } else {
            if (rescueState.sensor.distanceToHomeM < 5.0f && isBelowLandingAltitude()) {
                // attempted initiation within 5m of home, and 'on the ground' -> instant disarm, for safety reasons
                rescueState.phase = RESCUE_ABORT;
            } else {
                // attempted initiation within minimum rescue distance requires us to fly out to at least that distance
                if (rescueState.sensor.distanceToHomeM < gpsRescueConfig()->minStartDistM) {
                    // climb above current height by buffer height, to at least 10m altitude
                    rescueState.intent.returnAltitudeCm = fmaxf(1000.0f, getAltitudeCm() + (gpsRescueConfig()->initialClimbM * 100.0f));
                    // note that the pitch controller will pitch forward to fly out to minStartDistM
                    // set the descent distance to half the minimum rescue distance. which we should have moved out to in the climb phase
                    rescueState.intent.descentDistanceM = 0.5f * gpsRescueConfig()->minStartDistM;
                }
                // otherwise behave as for a normal rescue
                initialiseRescueValues ();
                returnAltitudeLow = getAltitudeCm() < rescueState.intent.returnAltitudeCm;
                rescueState.phase = RESCUE_ATTAIN_ALT;
            }
        }
        break;

    case RESCUE_ATTAIN_ALT:
        // gradually increment the target altitude until the craft reaches target altitude
        // note that this can mean the target altitude may increase above returnAltitude if the craft lags target
        // sanity check will abort if altitude gain is blocked for a cumulative period
        if (returnAltitudeLow == (getAltitudeCm() < rescueState.intent.returnAltitudeCm)) {
            // we started low, and still are low; also true if we started high, and still are too high
            rescueState.intent.targetAltitudeStepCm = (returnAltitudeLow ? gpsRescueConfig()->ascendRate : -1.0f * gpsRescueConfig()->descendRate) * taskIntervalSeconds;
            rescueState.intent.targetAltitudeCm += rescueState.intent.targetAltitudeStepCm;

        } else {
            // target altitude achieved - move on to ROTATE phase, returning at target altitude
            rescueState.intent.targetAltitudeCm = rescueState.intent.returnAltitudeCm;
            rescueState.intent.targetAltitudeStepCm = 0.0f;
            // if initiated too close, do not rotate or do anything else until sufficiently far away that a 'normal' rescue can happen
            if (rescueState.sensor.distanceToHomeM > gpsRescueConfig()->minStartDistM) {
                rescueState.phase = RESCUE_ROTATE;
            }
        }

        // give velocity P and I no error that otherwise could be present due to velocity drift at the start of the rescue
        rescueState.intent.targetVelocityCmS = rescueState.sensor.velocityToHomeCmS;
        break;

    case RESCUE_ROTATE:
        if (rescueState.intent.yawAttenuator < 1.0f) { // acquire yaw authority over one second
            rescueState.intent.yawAttenuator += taskIntervalSeconds;
        }
        if (rescueState.sensor.absErrorAngle < GPS_RESCUE_ALLOWED_YAW_RANGE) {
            // enter fly home phase, and enable pitch, when the yaw angle error is small enough
            rescueState.intent.pitchAngleLimitDeg = gpsRescueConfig()->maxRescueAngle;
            rescueState.phase = RESCUE_FLY_HOME; // enter fly home phase
            rescueState.intent.secondsFailing = 0; // reset sanity timer for flight home
        }
        initialVelocityLow = rescueState.sensor.velocityToHomeCmS < gpsRescueConfig()->groundSpeedCmS; // used to set direction of velocity target change
        rescueState.intent.targetVelocityCmS = rescueState.sensor.velocityToHomeCmS;
        break;

    case RESCUE_FLY_HOME:
        if (rescueState.intent.yawAttenuator < 1.0f) { // be sure to accumulate full yaw authority
            rescueState.intent.yawAttenuator += taskIntervalSeconds;
        }
        // velocity PIDs are now active
        // update target velocity gradually, aiming for rescueGroundspeed with a time constant of 1.0s
        const float targetVelocityError = gpsRescueConfig()->groundSpeedCmS - rescueState.intent.targetVelocityCmS;
        const float velocityTargetStep = taskIntervalSeconds * targetVelocityError;
        // velocityTargetStep is positive when starting low, negative when starting high
        const bool targetVelocityIsLow = rescueState.intent.targetVelocityCmS < gpsRescueConfig()->groundSpeedCmS;
        if (initialVelocityLow == targetVelocityIsLow) {
            // also true if started faster than target velocity and target is still high
            rescueState.intent.targetVelocityCmS += velocityTargetStep;
        }

        // slowly introduce velocity iTerm accumulation at start, goes 0 ->1 with time constant 2.0s
        rescueState.intent.velocityItermRelax += 0.5f * taskIntervalSeconds * (1.0f - rescueState.intent.velocityItermRelax);
        // there is always a lot of lag at the start, this gradual start avoids excess iTerm accumulation

        rescueState.intent.velocityPidCutoffModifier = 2.0f - rescueState.intent.velocityItermRelax;
        // higher velocity filter cutoff for initial few seconds to improve accuracy; can be smoother later

        if (newGpsData) {
            // cut back on allowed angle if there is a high groundspeed error
            rescueState.intent.pitchAngleLimitDeg = gpsRescueConfig()->maxRescueAngle;
            // introduce roll slowly and limit to half the max pitch angle; earth referenced yaw may add more roll via angle code
            rescueState.intent.rollAngleLimitDeg = 0.5f * rescueState.intent.pitchAngleLimitDeg * rescueState.intent.velocityItermRelax;
            if (rescueState.sensor.distanceToHomeM <= rescueState.intent.descentDistanceM) {
                rescueState.phase = RESCUE_DESCENT;
                rescueState.intent.secondsFailing = 0; // reset sanity timer for descent
            }
        }
        break;

    case RESCUE_DESCENT:
        // attenuate velocity and altitude targets while updating the heading to home
        if (isBelowLandingAltitude()) {
            // enter landing mode once below landing altitude
            rescueState.phase = RESCUE_LANDING;
            rescueState.intent.secondsFailing = 0; // reset sanity timer for landing
        }
        descend(newGpsData);
        break;

    case RESCUE_LANDING:
        // Reduce altitude target steadily until impact, then disarm.
        // control yaw angle and throttle and pitch, attenuate velocity, roll and pitch iTerm
        // increase velocity smoothing cutoff as we get closer to ground
        descend(newGpsData);
        disarmOnImpact();
        break;

    case RESCUE_ABORT:
        setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
        disarm(DISARM_REASON_FAILSAFE);
        rescueState.intent.secondsFailing = 0; // reset sanity timers so we can re-arm
        rescueStop();
        break;

    case RESCUE_DO_NOTHING:
        disarmOnImpact();
        break;

    default:
        break;
    }

    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 3, lrintf(rescueState.intent.targetAltitudeCm));
    DEBUG_SET(DEBUG_RTH, 0, lrintf(rescueState.intent.maxAltitudeCm / 10.0f));

    performSanityChecks();
    rescueAttainPosition(newGpsData);
}

float gpsRescueGetYawRate(void)
{
    return rescueYaw; // the control yaw value for rc.c to be used while flightMode gps_rescue is active.
}

float gpsRescueGetImuYawCogGain(void)
{
    return rescueState.sensor.imuYawCogGain; // to speed up the IMU orientation to COG when needed
}

bool gpsRescueIsConfigured(void)
{
    return failsafeConfig()->failsafe_procedure == FAILSAFE_PROCEDURE_GPS_RESCUE || isModeActivationConditionPresent(BOXGPSRESCUE);
}

bool gpsRescueIsAvailable(void)
{
    return rescueState.isAvailable; // flashes the warning when not available (low sats, etc)
}

bool gpsRescueIsDisabled(void)
// used for OSD warning, needs review
{
    return (!STATE(GPS_FIX_HOME));
}

#ifdef USE_MAG
bool gpsRescueDisableMag(void)
{
    // Enable mag on user request, but don't use it during fly home or if force disabled
    // Note that while flying home the course over ground from GPS provides a heading that is less affected by wind
    return !(gpsRescueConfig()->useMag && rescueState.phase != RESCUE_FLY_HOME && !magForceDisable);
}
#endif
#endif

#endif // !USE_WING
