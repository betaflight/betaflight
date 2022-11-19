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

#ifdef USE_GPS_RESCUE

#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/time.h"

#include "io/gps.h"

#include "config/config.h"
#include "fc/core.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/position.h"

#include "rx/rx.h"

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
    RESCUE_ABORT,
    RESCUE_COMPLETE,
    RESCUE_DO_NOTHING
} rescuePhase_e;

typedef enum {
    RESCUE_HEALTHY,
    RESCUE_FLYAWAY,
    RESCUE_GPSLOST,
    RESCUE_LOWSATS,
    RESCUE_CRASH_FLIP_DETECTED,
    RESCUE_STALLED,
    RESCUE_TOO_CLOSE,
    RESCUE_NO_HOME_POINT
} rescueFailureState_e;

typedef struct {
    float maxAltitudeCm;
    float returnAltitudeCm;
    float targetAltitudeCm;
    float targetLandingAltitudeCm;
    float targetVelocityCmS;
    float pitchAngleLimitDeg;
    float rollAngleLimitDeg;
    float descentDistanceM;
    int8_t secondsFailing;
    float altitudeStep;
    float descentRateModifier;
    float yawAttenuator;
    float disarmThreshold;
} rescueIntent_s;

typedef struct {
    float currentAltitudeCm;
    float distanceToHomeCm;
    float distanceToHomeM;
    uint16_t groundSpeedCmS;
    int16_t directionToHome;
    float accMagnitude;
    bool healthy;
    float errorAngle;
    float gpsDataIntervalSeconds;
    float altitudeDataIntervalSeconds;
    float velocityToHomeCmS;
    float alitutudeStepCm;
    float maxPitchStep;
    float absErrorAngle;
} rescueSensorData_s;

typedef struct {
    rescuePhase_e phase;
    rescueFailureState_e failure;
    rescueSensorData_s sensor;
    rescueIntent_s intent;
    bool isAvailable;
} rescueState_s;

#define GPS_RESCUE_MAX_YAW_RATE          180    // deg/sec max yaw rate
#define GPS_RESCUE_MIN_DESCENT_DIST_M    5      // minimum descent distance
#define GPS_RESCUE_MAX_ITERM_VELOCITY    1000   // max iterm value for velocity
#define GPS_RESCUE_MAX_ITERM_THROTTLE    200    // max iterm value for throttle
#define GPS_RESCUE_MAX_PITCH_RATE        3000   // max change in pitch per second in degrees * 100
#define GPS_RESCUE_DISARM_THRESHOLD      2.0f   // disarm threshold in G's

static float rescueThrottle;
static float rescueYaw;
float       gpsRescueAngle[ANGLE_INDEX_COUNT] = { 0, 0 };
bool        magForceDisable = false;
static bool newGPSData = false;
static pt2Filter_t throttleDLpf;
static pt2Filter_t velocityDLpf;
static pt3Filter_t pitchLpf;

rescueState_s rescueState;

void gpsRescueInit(void)
{
    const float sampleTimeS = HZ_TO_INTERVAL(TASK_GPS_RESCUE_RATE_HZ);
    float cutoffHz, gain;

    cutoffHz = positionConfig()->altitude_d_lpf / 100.0f;
    gain = pt2FilterGain(cutoffHz, sampleTimeS);
    pt2FilterInit(&throttleDLpf, gain);

    cutoffHz = 0.8f;
    gain = pt2FilterGain(cutoffHz, 1.0f);
    pt2FilterInit(&velocityDLpf, gain);

    cutoffHz = 4.0f;
    gain = pt3FilterGain(cutoffHz, sampleTimeS);
    pt3FilterInit(&pitchLpf, gain);
}

/*
 If we have new GPS data, update home heading if possible and applicable.
*/
void gpsRescueNewGpsData(void)
{
    newGPSData = true;
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
static void setReturnAltitude(void)
{
    // Hold maxAltitude at zero while disarmed, but if set_home_point_once is true, hold maxAlt until power cycled
    if (!ARMING_FLAG(ARMED) && !gpsConfig()->gps_set_home_point_once) {
        rescueState.intent.maxAltitudeCm = 0.0f;
        return;
    }

    // While armed, but not during the rescue, update the max altitude value
    rescueState.intent.maxAltitudeCm = fmaxf(rescueState.sensor.currentAltitudeCm, rescueState.intent.maxAltitudeCm);

    if (newGPSData) {
        // set the target altitude to current values, so there will be no D kick on first run
        rescueState.intent.targetAltitudeCm = rescueState.sensor.currentAltitudeCm;

        // Keep the descent distance and intended altitude up to date with latest GPS values
        rescueState.intent.descentDistanceM = constrainf(rescueState.sensor.distanceToHomeM, GPS_RESCUE_MIN_DESCENT_DIST_M, gpsRescueConfig()->descentDistanceM);
        const float initialAltitudeCm = gpsRescueConfig()->initialAltitudeM * 100.0f;
        const float rescueAltitudeBufferCm = gpsRescueConfig()->rescueAltitudeBufferM * 100.0f;
        switch (gpsRescueConfig()->altitudeMode) {
            case GPS_RESCUE_ALT_MODE_FIXED:
                rescueState.intent.returnAltitudeCm = initialAltitudeCm;
                break;
            case GPS_RESCUE_ALT_MODE_CURRENT:
                rescueState.intent.returnAltitudeCm = rescueState.sensor.currentAltitudeCm + rescueAltitudeBufferCm;
                break;
            case GPS_RESCUE_ALT_MODE_MAX:
            default:
                rescueState.intent.returnAltitudeCm = rescueState.intent.maxAltitudeCm + rescueAltitudeBufferCm;
                break;
        }
    }
}

static void rescueAttainPosition(void)
{
    // runs at 100hz, but only updates RPYT settings when new GPS Data arrives and when not in idle phase.
    static float previousVelocityError = 0.0f;
    static float velocityI = 0.0f;
    static float previousPitchAdjustment = 0.0f;
    static float throttleI = 0.0f;
    static float previousAltitudeError = 0.0f;
    static int16_t throttleAdjustment = 0;

    switch (rescueState.phase) {
    case RESCUE_IDLE:
        // values to be returned when no rescue is active
        gpsRescueAngle[AI_PITCH] = 0.0f;
        gpsRescueAngle[AI_ROLL] = 0.0f;
        rescueThrottle = rcCommand[THROTTLE];
        return;
    case RESCUE_INITIALIZE:
        // Initialize internal variables each time GPS Rescue is started
        previousVelocityError = 0.0f;
        velocityI = 0.0f;
        previousPitchAdjustment = 0.0f;
        throttleI = 0.0f;
        previousAltitudeError = 0.0f;
        throttleAdjustment = 0;
        rescueState.intent.disarmThreshold = GPS_RESCUE_DISARM_THRESHOLD;
        return;
    case RESCUE_DO_NOTHING:
        // 20s of slow descent for switch induced sanity failures to allow time to recover
        gpsRescueAngle[AI_PITCH] = 0.0f;
        gpsRescueAngle[AI_ROLL] = 0.0f;
        rescueThrottle = gpsRescueConfig()->throttleHover - 100;
        return;
     default:
        break;
    }

    /**
        Altitude (throttle) controller
    */
    // currentAltitudeCm is updated at TASK_GPS_RESCUE_RATE_HZ
    const float altitudeError = (rescueState.intent.targetAltitudeCm - rescueState.sensor.currentAltitudeCm) * 0.01f;
    // height above target in metres (negative means too low)
    // at the start, the target starts at current altitude plus one step.  Increases stepwise to intended value.

    // P component
    const float throttleP = gpsRescueConfig()->throttleP * altitudeError;

    // I component
    throttleI += 0.1f * gpsRescueConfig()->throttleI * altitudeError * rescueState.sensor.altitudeDataIntervalSeconds;
    throttleI = constrainf(throttleI, -1.0f * GPS_RESCUE_MAX_ITERM_THROTTLE, 1.0f * GPS_RESCUE_MAX_ITERM_THROTTLE);
    // up to 20% increase in throttle from I alone

    // D component is error based, so includes positive boost when climbing and negative boost on descent
    float verticalSpeed = ((altitudeError - previousAltitudeError) / rescueState.sensor.altitudeDataIntervalSeconds);
    previousAltitudeError = altitudeError;
    verticalSpeed += rescueState.intent.descentRateModifier * verticalSpeed;
    // add up to 2x D when descent rate is faster

    float throttleD = pt2FilterApply(&throttleDLpf, verticalSpeed);

    rescueState.intent.disarmThreshold = GPS_RESCUE_DISARM_THRESHOLD - throttleD / 15.0f; // make disarm more likely if throttle D is high

    throttleD = gpsRescueConfig()->throttleD * throttleD;

    // acceleration component not currently implemented - was needed previously due to GPS lag, maybe not needed now.

    float tiltAdjustment = 1.0f - getCosTiltAngle(); // 0 = flat, gets to 0.2 correcting on a windy day
    tiltAdjustment *= (gpsRescueConfig()->throttleHover - 1000);
    // if hover is 1300, and adjustment .2, this gives us 0.2*300 or 60 of extra throttle, not much, but useful
    // too much and landings with lots of pitch adjustment, eg windy days, can be a problem

    throttleAdjustment = throttleP + throttleI + throttleD + tiltAdjustment;

    rescueThrottle = gpsRescueConfig()->throttleHover + throttleAdjustment;
    rescueThrottle = constrainf(rescueThrottle, gpsRescueConfig()->throttleMin, gpsRescueConfig()->throttleMax);
    DEBUG_SET(DEBUG_GPS_RESCUE_THROTTLE_PID, 0, lrintf(throttleP));
    DEBUG_SET(DEBUG_GPS_RESCUE_THROTTLE_PID, 1, lrintf(throttleD));

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
    // A compass (magnetometer) is vital for accurate GPS rescue at slow speeds, but must be calibrated and validated.

    rescueYaw = rescueState.sensor.errorAngle * gpsRescueConfig()->yawP * rescueState.intent.yawAttenuator * 0.1f;
    rescueYaw = constrainf(rescueYaw, -GPS_RESCUE_MAX_YAW_RATE, GPS_RESCUE_MAX_YAW_RATE);
    // rescueYaw is the yaw rate in deg/s to correct the heading error

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

    /**
        Pitch / velocity controller
    */
    static float pitchAdjustment = 0.0f;
    if (newGPSData) {

        const float sampleIntervalNormaliseFactor = rescueState.sensor.gpsDataIntervalSeconds * 10.0f;

        const float velocityError = (rescueState.intent.targetVelocityCmS - rescueState.sensor.velocityToHomeCmS);
        // velocityError is in cm per second, positive means too slow.
        // NB positive pitch setpoint means nose down.

        // P component
        const float velocityP = velocityError * gpsRescueConfig()->velP;

        // I component
        velocityI += 0.01f * gpsRescueConfig()->velI * velocityError * sampleIntervalNormaliseFactor;
        // increase amount added when GPS sample rate is slower
        velocityI = constrainf(velocityI, -1.0f * GPS_RESCUE_MAX_ITERM_VELOCITY, 1.0f * GPS_RESCUE_MAX_ITERM_VELOCITY);
        // I component alone cannot exceed a pitch angle of 10%

        // D component
        float velocityD = ((velocityError - previousVelocityError) / sampleIntervalNormaliseFactor);
        previousVelocityError = velocityError;
        const float gain = pt2FilterGain(0.8f, HZ_TO_INTERVAL(gpsGetSampleRateHz()));
        pt2FilterUpdateCutoff(&velocityDLpf, gain);
        velocityD = pt2FilterApply(&velocityDLpf, velocityD);
        velocityD *= gpsRescueConfig()->velD;

        const float velocityIAttenuator = rescueState.intent.targetVelocityCmS / gpsRescueConfig()->rescueGroundspeed;
        // reduces iTerm as target velocity decreases, to minimise overshoot during deceleration to landing phase

        pitchAdjustment = velocityP + velocityD;
        if (rescueState.phase == RESCUE_FLY_HOME) {
            pitchAdjustment *= 0.7f; // attenuate pitch PIDs during main fly home phase, tighten up in descent.
        }
        pitchAdjustment += velocityI * velocityIAttenuator;

        const float movingAvgPitchAdjustment = 0.5f * (previousPitchAdjustment + pitchAdjustment);
         // moving average seems to work best here, a lot of sequential up and down in velocity data
        previousPitchAdjustment = pitchAdjustment;
        pitchAdjustment = movingAvgPitchAdjustment;
        // pitchAdjustment is the absolute Pitch angle adjustment value in degrees * 100
        // it gets added to the normal level mode Pitch adjustments in pid.c
        DEBUG_SET(DEBUG_GPS_RESCUE_VELOCITY, 0, lrintf(velocityP));
        DEBUG_SET(DEBUG_GPS_RESCUE_VELOCITY, 1, lrintf(velocityD));
    }

    const float pitchAdjustmentFiltered = pt3FilterApply(&pitchLpf, pitchAdjustment);
    // upsampling and smoothing of pitch angle steps

    const float pitchAngleLimit = rescueState.intent.pitchAngleLimitDeg * 100.0f;
    gpsRescueAngle[AI_PITCH] = constrainf(pitchAdjustmentFiltered, -pitchAngleLimit, pitchAngleLimit);
    // this angle gets added to the normal pitch Angle Mode control values in pid.c - will be seen in pitch setpoint

    DEBUG_SET(DEBUG_GPS_RESCUE_VELOCITY, 3, lrintf(rescueState.intent.targetVelocityCmS));
    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 1, lrintf(rescueState.intent.targetVelocityCmS));
}

static void performSanityChecks(void)
{
    static timeUs_t previousTimeUs = 0; // Last time Stalled/LowSat was checked
    static float prevAltitudeCm = 0.0f; // to calculate ascent or descent change
    static float prevTargetAltitudeCm = 0.0f; // to calculate ascent or descent target change
    static float previousDistanceToHomeCm = 0.0f; // to check that we are returning
    static int8_t secondsLowSats = 0; // Minimum sat detection
    static int8_t secondsDoingNothing = 0; // Limit on doing nothing
    const timeUs_t currentTimeUs = micros();

    if (rescueState.phase == RESCUE_IDLE) {
        rescueState.failure = RESCUE_HEALTHY;
        return;
    } else if (rescueState.phase == RESCUE_INITIALIZE) {
        // Initialize these variables each time a GPS Rescue is started
        previousTimeUs = currentTimeUs;
        prevAltitudeCm = rescueState.sensor.currentAltitudeCm;
        prevTargetAltitudeCm = rescueState.intent.targetAltitudeCm;
        previousDistanceToHomeCm = rescueState.sensor.distanceToHomeCm;
        secondsLowSats = 0;
        secondsDoingNothing = 0;
        return;
    }

    // Handle events that set a failure mode to other than healthy.
    // Disarm via Abort when sanity on, or for hard Rx loss in FS_ONLY mode
    // Otherwise allow 20s of semi-controlled descent with impact disarm detection
    const bool hardFailsafe = !rxIsReceivingSignal();
    if (rescueState.failure != RESCUE_HEALTHY) {
        if (gpsRescueConfig()->sanityChecks == RESCUE_SANITY_ON) {
            rescueState.phase = RESCUE_ABORT;
        } else if ((gpsRescueConfig()->sanityChecks == RESCUE_SANITY_FS_ONLY) && hardFailsafe) {
            rescueState.phase = RESCUE_ABORT;
        } else {
            // even with sanity checks off, 
            rescueState.phase = RESCUE_DO_NOTHING; // 20s semi-controlled descent with impact detection, then abort
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
        const float velocityToHomeCmS = previousDistanceToHomeCm- rescueState.sensor.distanceToHomeCm; // cm/s
        previousDistanceToHomeCm = rescueState.sensor.distanceToHomeCm;
        rescueState.intent.secondsFailing += (velocityToHomeCmS < 0.5f * rescueState.intent.targetVelocityCmS) ? 1 : -1;
        rescueState.intent.secondsFailing = constrain(rescueState.intent.secondsFailing, 0, 15);
        if (rescueState.intent.secondsFailing == 15) {
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

    const float actualAltitudeChange = rescueState.sensor.currentAltitudeCm - prevAltitudeCm;
    const float targetAltitudeChange = rescueState.intent.targetAltitudeCm - prevTargetAltitudeCm;
    const float ratio = actualAltitudeChange / targetAltitudeChange;
    prevAltitudeCm = rescueState.sensor.currentAltitudeCm;
    prevTargetAltitudeCm = rescueState.intent.targetAltitudeCm;

    if (rescueState.phase == RESCUE_LANDING) {
        rescueState.intent.secondsFailing += ratio > 0.5f ? -1 : 1;
        rescueState.intent.secondsFailing = constrain(rescueState.intent.secondsFailing, 0, 10);
        if (rescueState.intent.secondsFailing == 10) {
            rescueState.phase = RESCUE_ABORT;
            // Landing mode shouldn't take more than 10s
        }
    } else if (rescueState.phase == RESCUE_ATTAIN_ALT || rescueState.phase == RESCUE_DESCENT) {
        rescueState.intent.secondsFailing += ratio > 0.5f ? -1 : 1;
        rescueState.intent.secondsFailing = constrain(rescueState.intent.secondsFailing, 0, 10);
        if (rescueState.intent.secondsFailing == 10) {
            rescueState.phase = RESCUE_LANDING;
            rescueState.intent.secondsFailing = 0;
            // if can't climb, or slow descending, enable impact detection and time out in 10s
        }
    } else if (rescueState.phase == RESCUE_DO_NOTHING) {
        secondsDoingNothing = MIN(secondsDoingNothing + 1, 20);
        if (secondsDoingNothing == 20) {
            rescueState.phase = RESCUE_ABORT;
            // time-limited semi-controlled fall with impact detection
        }
    }

    DEBUG_SET(DEBUG_RTH, 2, (rescueState.failure * 10 + rescueState.phase));
    DEBUG_SET(DEBUG_RTH, 3, (rescueState.intent.secondsFailing * 100 + secondsLowSats));
}

static void sensorUpdate(void)
{
    static float prevDistanceToHomeCm = 0.0f;
    const timeUs_t currentTimeUs = micros();

    static timeUs_t previousAltitudeDataTimeUs = 0;
    const timeDelta_t altitudeDataIntervalUs = cmpTimeUs(currentTimeUs, previousAltitudeDataTimeUs);
    rescueState.sensor.altitudeDataIntervalSeconds = altitudeDataIntervalUs * 0.000001f;
    previousAltitudeDataTimeUs = currentTimeUs;

    rescueState.sensor.currentAltitudeCm = getAltitude();

    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 2, lrintf(rescueState.sensor.currentAltitudeCm));
    DEBUG_SET(DEBUG_GPS_RESCUE_THROTTLE_PID, 2, lrintf(rescueState.sensor.currentAltitudeCm));
    DEBUG_SET(DEBUG_GPS_RESCUE_HEADING, 0, rescueState.sensor.groundSpeedCmS); // groundspeed cm/s
    DEBUG_SET(DEBUG_GPS_RESCUE_HEADING, 1, gpsSol.groundCourse); // degrees * 10
    DEBUG_SET(DEBUG_GPS_RESCUE_HEADING, 2, attitude.values.yaw); // degrees * 10
    DEBUG_SET(DEBUG_GPS_RESCUE_HEADING, 3, rescueState.sensor.directionToHome); // degrees * 10

    rescueState.sensor.healthy = gpsIsHealthy();

    if (rescueState.phase == RESCUE_LANDING) {
        // do this at sensor update rate, not the much slower GPS rate, for quick disarm
        rescueState.sensor.accMagnitude = (float) sqrtf(sq(acc.accADC[Z]) + sq(acc.accADC[X]) + sq(acc.accADC[Y])) * acc.dev.acc_1G_rec;
    }

    rescueState.sensor.directionToHome = GPS_directionToHome;
    rescueState.sensor.errorAngle = (attitude.values.yaw - rescueState.sensor.directionToHome) * 0.1f;
    // both attitude and direction are in degrees * 10, errorAngle is degrees
    if (rescueState.sensor.errorAngle <= -180) {
        rescueState.sensor.errorAngle += 360;
    } else if (rescueState.sensor.errorAngle > 180) {
        rescueState.sensor.errorAngle -= 360;
    }
    rescueState.sensor.absErrorAngle = fabsf(rescueState.sensor.errorAngle);

    if (!newGPSData) {
        return;
        // GPS ground speed, velocity and distance to home will be held at last good values if no new packets
    }

    rescueState.sensor.distanceToHomeCm = GPS_distanceToHomeCm;
    rescueState.sensor.distanceToHomeM = rescueState.sensor.distanceToHomeCm / 100.0f;
    rescueState.sensor.groundSpeedCmS = gpsSol.groundSpeed; // cm/s

    static timeUs_t previousGPSDataTimeUs = 0;
    const timeDelta_t gpsDataIntervalUs = cmpTimeUs(currentTimeUs, previousGPSDataTimeUs);
    rescueState.sensor.gpsDataIntervalSeconds = constrainf(gpsDataIntervalUs * 0.000001f, 0.01f, 1.0f);
    // Range from 10ms (100hz) to 1000ms (1Hz). Intended to cover common GPS data rates and exclude unusual values.
    previousGPSDataTimeUs = currentTimeUs;

    rescueState.sensor.velocityToHomeCmS = (prevDistanceToHomeCm - rescueState.sensor.distanceToHomeCm) / rescueState.sensor.gpsDataIntervalSeconds;
    // positive = towards home.  First value is useless since prevDistanceToHomeCm was zero.
    prevDistanceToHomeCm = rescueState.sensor.distanceToHomeCm;

    rescueState.sensor.maxPitchStep = rescueState.sensor.gpsDataIntervalSeconds * GPS_RESCUE_MAX_PITCH_RATE;

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

void disarmOnImpact(void)
{
    if (rescueState.sensor.accMagnitude > rescueState.intent.disarmThreshold) {
        setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
        disarm(DISARM_REASON_GPS_RESCUE);
        rescueStop();
    }
}

void descend(void)
{
    if (newGPSData) {
        const float distanceToLandingAreaM = rescueState.sensor.distanceToHomeM - (rescueState.intent.targetLandingAltitudeCm / 200.0f);
        // considers home to be a circle half landing height around home to avoid overshooting home point
        const float proximityToLandingArea = constrainf(distanceToLandingAreaM / rescueState.intent.descentDistanceM, 0.0f, 1.0f);
        rescueState.intent.targetVelocityCmS = gpsRescueConfig()->rescueGroundspeed * proximityToLandingArea;
        // reduce target velocity as we get closer to home. Zero within 2m of home, reducing risk of overshooting.
        // if quad drifts further than 2m away from home, should by then have rotated towards home, so pitch is allowed
        rescueState.intent.rollAngleLimitDeg = gpsRescueConfig()->angle * proximityToLandingArea;
        // reduce roll capability when closer to home, none within final 2m
    }

    // adjust altitude step for interval between altitude readings
    rescueState.intent.altitudeStep = -1.0f * rescueState.sensor.altitudeDataIntervalSeconds * gpsRescueConfig()->descendRate;

    // descend more slowly if return altitude is less than 20m
    const float descentAttenuator = rescueState.intent.returnAltitudeCm / 2000.0f;
    if (descentAttenuator < 1.0f) {
        rescueState.intent.altitudeStep *= descentAttenuator;
    }
    // descend more quickly from higher altitude
    rescueState.intent.descentRateModifier = constrainf(rescueState.intent.targetAltitudeCm / 5000.0f, 0.0f, 1.0f);
    rescueState.intent.targetAltitudeCm += rescueState.intent.altitudeStep * (1.0f + (2.0f * rescueState.intent.descentRateModifier));
    // increase descent rate to max of 3x default above 50m, 2x above 25m, 1.2 at 5m, default by ground level.
}

void altitudeAchieved(void)
{
    rescueState.intent.targetAltitudeCm = rescueState.intent.returnAltitudeCm;
    rescueState.intent.altitudeStep = 0;
    rescueState.phase = RESCUE_ROTATE;
}

void gpsRescueUpdate(void)
// this runs a lot faster than the GPS Data update rate, and runs whether or not rescue is active
{
    if (!FLIGHT_MODE(GPS_RESCUE_MODE)) {
        rescueStop(); // sets phase to RESCUE_IDLE; does nothing else.  RESCUE_IDLE tasks still run.
    } else if (FLIGHT_MODE(GPS_RESCUE_MODE) && rescueState.phase == RESCUE_IDLE) {
        rescueStart(); // sets phase to rescue_initialise if we enter GPS Rescue mode while idle
        rescueAttainPosition(); // Initialise basic parameters when a Rescue starts (can't initialise sensor data reliably)
        performSanityChecks(); // Initialises sanity check values when a Rescue starts
    }

    // Will now be in RESCUE_INITIALIZE mode, if just entered Rescue while IDLE, otherwise stays IDLE

    sensorUpdate(); // always get latest GPS and Altitude data, update ascend and descend rates

    bool startedLow = true;
    rescueState.isAvailable = checkGPSRescueIsAvailable();

    switch (rescueState.phase) {
    case RESCUE_IDLE:
        // in Idle phase = NOT in GPS Rescue
        // update the return altitude and descent distance values, to have valid settings immediately they are needed
        setReturnAltitude();
        break;
        // sanity checks are bypassed in IDLE mode; instead, failure state is always initialised to HEALTHY
        // target altitude is always set to current altitude.

    case RESCUE_INITIALIZE:
        // Things that should abort the start of a Rescue
        if (!STATE(GPS_FIX_HOME)) {
            // we didn't get a home point on arming
            rescueState.failure = RESCUE_NO_HOME_POINT;
            // will result in a disarm via the sanity check system, with delay if switch induced
            // alternative is to prevent the rescue by returning to IDLE, but this could cause flyaways
        } else if (rescueState.sensor.distanceToHomeM < gpsRescueConfig()->minRescueDth) {
            // Attempt to initiate inside minimum activation distance -> landing mode
            rescueState.intent.altitudeStep = -rescueState.sensor.altitudeDataIntervalSeconds * gpsRescueConfig()->descendRate;
            rescueState.intent.targetVelocityCmS = 0; // zero forward velocity
            rescueState.intent.pitchAngleLimitDeg = 0; // flat on pitch
            rescueState.intent.rollAngleLimitDeg = 0.0f; // flat on roll also
            rescueState.intent.targetAltitudeCm = rescueState.sensor.currentAltitudeCm + rescueState.intent.altitudeStep;
            rescueState.phase = RESCUE_LANDING;
            // start landing from current altitude
        } else {
            rescueState.phase = RESCUE_ATTAIN_ALT;
            rescueState.intent.secondsFailing = 0; // reset the sanity check timer for the climb
            rescueState.intent.targetLandingAltitudeCm = 100.0f * gpsRescueConfig()->targetLandingAltitudeM;
            startedLow = (rescueState.sensor.currentAltitudeCm <= rescueState.intent.returnAltitudeCm);
            rescueState.intent.yawAttenuator = 0.0f;
            rescueState.intent.targetVelocityCmS = 0.0f; // zero forward velocity while climbing
            rescueState.intent.pitchAngleLimitDeg = 0.0f; // no pitch
            rescueState.intent.rollAngleLimitDeg = 0.0f; // no roll until flying home
            rescueState.intent.altitudeStep = 0.0f;
            rescueState.intent.descentRateModifier = 0.0f;
        }
        break;

    case RESCUE_ATTAIN_ALT:
        // gradually increment the target altitude until the craft reaches target altitude
        // note that this can mean the target altitude may increase above returnAltitude if the craft lags target
        // sanity check will abort if altitude gain is blocked for a cumulative period
        if (startedLow) {
            if (rescueState.intent.targetAltitudeCm < rescueState.intent.returnAltitudeCm) {
                rescueState.intent.altitudeStep = rescueState.sensor.altitudeDataIntervalSeconds * gpsRescueConfig()->ascendRate;
            } else if (rescueState.sensor.currentAltitudeCm > rescueState.intent.returnAltitudeCm) {
                altitudeAchieved();
            } 
        } else {
            if (rescueState.intent.targetAltitudeCm > rescueState.intent.returnAltitudeCm) {
                rescueState.intent.altitudeStep = -rescueState.sensor.altitudeDataIntervalSeconds * gpsRescueConfig()->descendRate;
            } else if (rescueState.sensor.currentAltitudeCm < rescueState.intent.returnAltitudeCm) {
                altitudeAchieved();
            }
        }
        rescueState.intent.targetAltitudeCm += rescueState.intent.altitudeStep;
        break;

    case RESCUE_ROTATE:
        if (rescueState.intent.yawAttenuator < 1.0f) { // gradually acquire yaw authority
            rescueState.intent.yawAttenuator += 0.01f;
        }
        if (rescueState.sensor.absErrorAngle < 30.0f) {
            rescueState.intent.pitchAngleLimitDeg = gpsRescueConfig()->angle; // allow pitch
            rescueState.phase = RESCUE_FLY_HOME; // enter fly home phase
            rescueState.intent.secondsFailing = 0; // reset sanity timer for flight home
        }
        break;

    case RESCUE_FLY_HOME:
        if (rescueState.intent.yawAttenuator < 1.0f) { // be sure to accumulate full yaw authority
            rescueState.intent.yawAttenuator += 0.01f;
        }
        // steadily increase target velocity target until full return velocity is acquired
        if (rescueState.intent.targetVelocityCmS < gpsRescueConfig()->rescueGroundspeed) {
            rescueState.intent.targetVelocityCmS += 0.01f * gpsRescueConfig()->rescueGroundspeed;
        }
        // acquire full roll authority slowly when pointing to home
        if (rescueState.sensor.absErrorAngle < 10.0f && rescueState.intent.rollAngleLimitDeg < gpsRescueConfig()->angle) {
            // roll is primarily intended to deal with wind drift causing small yaw errors during return
            rescueState.intent.rollAngleLimitDeg += 0.1f;
        } 

        if (newGPSData) {
            if (rescueState.sensor.distanceToHomeM <= rescueState.intent.descentDistanceM) {
                rescueState.phase = RESCUE_DESCENT;
                rescueState.intent.secondsFailing = 0; // reset sanity timer for descent
            }
        }
        break;

    case RESCUE_DESCENT:
        // attenuate velocity and altitude targets while updating the heading to home
        if (rescueState.sensor.currentAltitudeCm < rescueState.intent.targetLandingAltitudeCm) {
            // enter landing mode once below landing altitude
            rescueState.phase = RESCUE_LANDING;
            rescueState.intent.secondsFailing = 0; // reset sanity timer for landing
        }
        descend();
        break;

    case RESCUE_LANDING:
        // Reduce altitude target steadily until impact, then disarm.
        // control yaw angle and throttle and pitch, attenuate velocity, roll and pitch iTerm
        descend();
        disarmOnImpact();
        break;

    case RESCUE_COMPLETE:
        rescueStop();
        break;

    case RESCUE_ABORT:
        setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
        disarm(DISARM_REASON_FAILSAFE);
        rescueStop();
        break;

    case RESCUE_DO_NOTHING:
        disarmOnImpact();
        break;

    default:
        break;
    }

    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 3, lrintf(rescueState.intent.targetAltitudeCm));
    DEBUG_SET(DEBUG_GPS_RESCUE_THROTTLE_PID, 3, lrintf(rescueState.intent.targetAltitudeCm));
    DEBUG_SET(DEBUG_RTH, 0, lrintf(rescueState.intent.maxAltitudeCm));

    performSanityChecks();
    rescueAttainPosition();

    newGPSData = false;
}

float gpsRescueGetYawRate(void)
{
    return rescueYaw;
}

float gpsRescueGetThrottle(void)
{
    // Calculated a desired commanded throttle scaled from 0.0 to 1.0 for use in the mixer.
    // We need to compensate for min_check since the throttle value set by gps rescue
    // is based on the raw rcCommand value commanded by the pilot.
    float commandedThrottle = scaleRangef(rescueThrottle, MAX(rxConfig()->mincheck, PWM_RANGE_MIN), PWM_RANGE_MAX, 0.0f, 1.0f);
    commandedThrottle = constrainf(commandedThrottle, 0.0f, 1.0f);

    return commandedThrottle;
}

bool gpsRescueIsConfigured(void)
{
    return failsafeConfig()->failsafe_procedure == FAILSAFE_PROCEDURE_GPS_RESCUE || isModeActivationConditionPresent(BOXGPSRESCUE);
}

bool gpsRescueIsAvailable(void)
{
    return rescueState.isAvailable;
}

bool gpsRescueIsDisabled(void)
// used for OSD warning
{
    return (!STATE(GPS_FIX_HOME));
}

#ifdef USE_MAG
bool gpsRescueDisableMag(void)
{
    return ((!gpsRescueConfig()->useMag || magForceDisable) && (rescueState.phase >= RESCUE_INITIALIZE) && (rescueState.phase <= RESCUE_LANDING));
}
#endif
#endif
