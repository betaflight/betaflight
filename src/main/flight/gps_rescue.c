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
#include <math.h>

#include "platform.h"

#ifdef USE_GPS_RESCUE

#include "build/debug.h"

#include "common/axis.h"
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

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"

#include "gps_rescue.h"

typedef enum {
    RESCUE_SANITY_OFF = 0,
    RESCUE_SANITY_ON,
    RESCUE_SANITY_FS_ONLY
} gpsRescueSanity_e;

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
    float returnAltitudeCm;
    float targetAltitudeCm;
    uint16_t targetVelocityCmS;
    int8_t minAnglePitchDeg;         // can be negative
    uint8_t maxAnglePitchDeg;        // must be positive
    int8_t maxAngleRollDeg;          // only one value since must always have a negative to positive range
    bool updateYaw;
    float descentDistanceM;
} rescueIntent_s;

typedef struct {
    int32_t maxAltitudeCm;
    int32_t currentAltitudeCm;
    float distanceToHomeCm;
    float distanceToHomeM;
    uint16_t maxDistanceToHomeM;
    uint16_t groundSpeedCmS;  //cm/s
    int16_t directionToHome;
    uint8_t numSat;
    float accMagnitude;
    bool healthy;
    float errorAngle;
    float gpsDataIntervalSeconds;
    float velocityToHomeCmS;
    float ascendStepCm;
    float descendStepCm;
    float maxPitchStep;
    float filterK;
} rescueSensorData_s;

typedef struct {
    bool bumpDetection;
    bool convergenceDetection;
} rescueSanityFlags;

typedef struct {
    rescuePhase_e phase;
    rescueFailureState_e failure;
    rescueSensorData_s sensor;
    rescueIntent_s intent;
    bool isAvailable;
} rescueState_s;

typedef enum {
    MAX_ALT,
    FIXED_ALT,
    CURRENT_ALT
} altitudeMode_e;

#define GPS_RESCUE_MAX_YAW_RATE          90     // deg/sec max yaw rate
#define GPS_RESCUE_MIN_DESCENT_DIST_M    10     // minimum descent distance allowed
#define GPS_RESCUE_MAX_ITERM_VELOCITY    1000   // max allowed iterm value for velocity
#define GPS_RESCUE_MAX_ITERM_THROTTLE    200    // max allowed iterm value for throttle
#define GPS_RESCUE_MAX_PITCH_RATE        3000   // max allowed change in pitch per second in degrees * 100

#ifdef USE_MAG
#define GPS_RESCUE_USE_MAG              true
#else
#define GPS_RESCUE_USE_MAG              false
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(gpsRescueConfig_t, gpsRescueConfig, PG_GPS_RESCUE, 1);

PG_RESET_TEMPLATE(gpsRescueConfig_t, gpsRescueConfig,
    .angle = 32,
    .initialAltitudeM = 30,
    .descentDistanceM = 20,
    .rescueGroundspeed = 500,
    .throttleP = 18,
    .throttleI = 40,
    .throttleD = 13,
    .velP = 6,
    .velI = 20,
    .velD = 70,
    .yawP = 25,
    .throttleMin = 1100,
    .throttleMax = 1600,
    .throttleHover = 1275,
    .sanityChecks = RESCUE_SANITY_FS_ONLY,
    .minSats = 8,
    .minRescueDth = 30,
    .allowArmingWithoutFix = false,
    .useMag = GPS_RESCUE_USE_MAG,
    .targetLandingAltitudeM = 5,
    .targetLandingDistanceM = 5,
    .altitudeMode = MAX_ALT,
    .ascendRate = 500,          // cm/s, for altitude corrections on ascent
    .descendRate = 125,         // cm/s, for descent and landing phase, or negative ascent
    .rescueAltitudeBufferM = 10,
    .rollMix = 100
);

static float rescueThrottle;
static float rescueYaw;
float velocityToHomeCmS;
float       gpsRescueAngle[ANGLE_INDEX_COUNT] = { 0, 0 };
bool        magForceDisable = false;
static bool newGPSData = false;

rescueState_s rescueState;

/*
 If we have new GPS data, update home heading if possible and applicable.
*/
void rescueNewGpsData(void)
{
    newGPSData = true;
}

static void rescueStart()
{
    rescueState.phase = RESCUE_INITIALIZE;
}

static void rescueStop()
{
    rescueState.phase = RESCUE_IDLE;
}

// Things that need to run regardless of GPS rescue mode being enabled or not (?? huh ?? ct)
static void idleTasks()
{
    // Do not calculate any of the idle task values when we are not flying
    if (!ARMING_FLAG(ARMED)) {
        rescueState.sensor.maxAltitudeCm = 0;
        rescueState.sensor.maxDistanceToHomeM = 0;
        return;
    }

    // Don't update any rescue flight statistics if we haven't applied a proper altitude offset yet
    if (!isAltitudeOffset()) {
        return;
    }

    // Store the max altitude we see not during RTH so we know our fly-back minimum alt
    rescueState.sensor.maxAltitudeCm = MAX(rescueState.sensor.currentAltitudeCm, rescueState.sensor.maxAltitudeCm);
    // Store the max distance to home during normal flight so we know if a flyaway is happening
    rescueState.sensor.maxDistanceToHomeM = MAX(rescueState.sensor.maxDistanceToHomeM, rescueState.sensor.distanceToHomeCm / 100.0f);

    // Keep the descent distance and intended altitude up to date with latest GPS values, to be available immediately when needed
    if (newGPSData) {
        rescueState.intent.descentDistanceM = constrainf(rescueState.sensor.distanceToHomeM, GPS_RESCUE_MIN_DESCENT_DIST_M, gpsRescueConfig()->descentDistanceM);
        const float initialAltitudeCm = gpsRescueConfig()->initialAltitudeM * 100.0f;
        const float rescueAltitudeBufferCm = gpsRescueConfig()->rescueAltitudeBufferM * 100.0f;
        switch (gpsRescueConfig()->altitudeMode) {
            case FIXED_ALT:
                rescueState.intent.returnAltitudeCm = initialAltitudeCm;
                break;
            case CURRENT_ALT:
                rescueState.intent.returnAltitudeCm = rescueState.sensor.currentAltitudeCm + rescueAltitudeBufferCm;
                break;
            case MAX_ALT:
            default:
                rescueState.intent.returnAltitudeCm = rescueState.sensor.maxAltitudeCm + rescueAltitudeBufferCm;
                break;
        }

        // set the target altitude and velocity to current values, so there will be no D kick on first run
        rescueState.intent.targetAltitudeCm = rescueState.sensor.currentAltitudeCm;
        rescueState.intent.targetVelocityCmS = rescueState.sensor.velocityToHomeCmS;
        // set the landingInitiationAltitude for sanity check of Landing Mode.
    }

    rescueThrottle = rcCommand[THROTTLE]; // value to be returned when idle = normal throttle TODO ?? is this needed ??
}

static void rescueAttainPosition()
{
    // runs at 100hz, but only updates RPYT settings when new GPS Data arrives and when not in idle phase.
    static float previousVelocityError = 0.0f;
    static float velocityI = 0.0f;
    static float previousVelocityD = 0.0f;      // for smoothing
    static float previousPitchAdjustment = 0.0f;
    static float previousAltitudeError = 0.0f;
    static float throttleI = 0.0f;
    static float previousThrottleD = 0.0f;      // for jerk calc from raw Derivative
    static float previousThrottleDVal = 0.0f;   // for moving average of D and jerk
    static float previousThrottleD2 = 0.0f;     // for additional D first order smoothing
    static int16_t throttleAdjustment = 0;

    if (rescueState.phase == RESCUE_INITIALIZE) {
        // Initialize internal variables each time GPS Rescue is started
        // Note valid sensor data can't be used here.  Use idleTasks() for those kind of values
        previousVelocityError = 0.0f;
        velocityI = 0.0f;
        previousVelocityD = 0.0f;
        previousPitchAdjustment = 0.0f;
        previousAltitudeError = 0.0f;
        throttleI = 0.0f;
        previousThrottleD = 0.0f;
        previousThrottleDVal = 0.0f;
        previousThrottleD2 = 0.0f;
        throttleAdjustment = 0;
        velocityToHomeCmS = 0.0f;
        gpsRescueAngle[AI_ROLL] = 0.0f;
        gpsRescueAngle[AI_PITCH] = 0.0f;
        return;
    }

     if (rescueState.phase == RESCUE_DO_NOTHING) {
        gpsRescueAngle[AI_PITCH] = 0.0f;
        gpsRescueAngle[AI_ROLL] = 0.0f;
        rescueThrottle = gpsRescueConfig()->throttleHover;
        return;
     }

    if (!newGPSData) {
        return;
    }

    const float sampleIntervalNormaliseFactor = rescueState.sensor.gpsDataIntervalSeconds * 10.0f;

    /**
        Heading / yaw controller
    */
    // directionToHome and distanceToHome are accurate if the GPS Home point is accurate.
    // attitude.values.yaw is set by imuCalculateEstimatedAttitude() but only updated while groundspeed exceeds 2 m/s
    // for accurate return, the craft should exceed 5m/s in clean nose-forward flight at some point
    // the faster the return speed, the more accurate the IMU will be, but the consequences of IMU error at the start are greater
    // A compass (magnetometer) is vital for accurate GPS rescue at slow speeds

    // if the quad is pointing 180 degrees wrong at failsafe time, it will take 2s to rotate fully at 90 deg/s max rate
    // this gives the level mode controller time to adjust pitch and roll during the yaw
    // we need a relatively gradual trajectory change for attitude.values.yaw to update effectively

    rescueYaw = constrainf(rescueState.sensor.errorAngle * gpsRescueConfig()->yawP * 0.1f, -GPS_RESCUE_MAX_YAW_RATE, GPS_RESCUE_MAX_YAW_RATE);
    // rescueYaw is the yaw rate in deg/s to correct the heading error

    const float rollMixAttenuator = constrainf(1.0f - ABS(rescueYaw) * 0.01f, 0.0f, 1.0f);
    // attenuate roll as yaw rate increases, no roll at 100 deg/s of yaw
    const float rollAdjustment = - rescueYaw * gpsRescueConfig()->rollMix * rollMixAttenuator;
    // mix in the desired amount of roll; 1:1 yaw:roll when rollMix = 100 and yaw angles are small
    // when gpsRescueConfig()->rollMix is zero, there is no roll adjustment
    // rollAdjustment is degrees * 100
    // note that the roll element has the same sign as the yaw element *before* GET_DIRECTION
    gpsRescueAngle[AI_ROLL] = constrainf(rollAdjustment, rescueState.intent.maxAngleRollDeg * -100.0f, rescueState.intent.maxAngleRollDeg * 100.0f);
    // gpsRescueAngle is added to the normal roll Angle Mode corrections in pid.c

    rescueYaw *= GET_DIRECTION(rcControlsConfig()->yaw_control_reversed);
    if (!rescueState.intent.updateYaw) {
        rescueYaw = 0.0f;
    }

    DEBUG_SET(DEBUG_GPS_RESCUE_HEADING, 0, rescueYaw * 10.0f); // Rescue yaw rate in degrees/sec * 10
    DEBUG_SET(DEBUG_GPS_RESCUE_HEADING, 1, gpsRescueAngle[AI_ROLL]); // roll correction degrees * 100

    /**
        Pitch / velocity controller
    */
    float velocityError = (rescueState.intent.targetVelocityCmS - rescueState.sensor.velocityToHomeCmS);
    // velocityError is in cm per second, positive means too slow.
    // NB positive pitch setpoint means nose down.
    // quite heavily smoothed
    // IdleTasks sets target velocity to current velocity to minimise D spike when starting, and keep error = 0

    // P component
    const float velocityP = velocityError * gpsRescueConfig()->velP;

    // I component
    velocityI += 0.01f * gpsRescueConfig()->velI * velocityError * sampleIntervalNormaliseFactor;
    // normalisation increases amount added when data rate is slower than expected
    velocityI *= rescueState.intent.targetVelocityCmS / rescueState.intent.targetVelocityCmS;
    // attenuate iTerm at slower target velocity, to minimise overshoot, mostly during deceleration to landing phase
    velocityI = constrainf(velocityI, -1.0f * GPS_RESCUE_MAX_ITERM_VELOCITY, 1.0f * GPS_RESCUE_MAX_ITERM_VELOCITY);
    // I component alone cannot exceed a pitch angle of 10%

    // D component
    float velocityD = ((velocityError - previousVelocityError) / sampleIntervalNormaliseFactor);
    previousVelocityError = velocityError;
    // simple first order filter on derivative with k = 0.5 for 200ms steps
    velocityD = previousVelocityD + rescueState.sensor.filterK * (velocityD - previousVelocityD);
    previousVelocityD = velocityD;
    velocityD *= gpsRescueConfig()->velD;

    // Pitch PIDsum and smoothing
    float pitchAdjustment = velocityP + velocityD + velocityI;
    // simple rate of change limiter - not more than 25 deg/s of pitch change to keep pitch smooth
    float pitchAdjustmentDelta = pitchAdjustment - previousPitchAdjustment;
    if (pitchAdjustmentDelta > rescueState.sensor.maxPitchStep) {
        pitchAdjustment = previousPitchAdjustment + rescueState.sensor.maxPitchStep;
    } else if (pitchAdjustmentDelta < -rescueState.sensor.maxPitchStep) {
        pitchAdjustment = previousPitchAdjustment - rescueState.sensor.maxPitchStep;
    }
    const float movingAvgPitchAdjustment = 0.5f * (previousPitchAdjustment + pitchAdjustment);
     // moving average seems to work best here, a lot of sequential up and down in velocity data
    previousPitchAdjustment = pitchAdjustment;
    pitchAdjustment = movingAvgPitchAdjustment;
    // pitchAdjustment is the absolute Pitch angle adjustment value in degrees * 100
    // it gets added to the normal level mode Pitch adjustments in pid.c

    gpsRescueAngle[AI_PITCH] = constrainf(pitchAdjustment, rescueState.intent.minAnglePitchDeg * 100.0f, rescueState.intent.maxAnglePitchDeg * 100.0f);
    // this angle gets added to the normal pitch Angle Mode control values in pid.c - will be seen in pitch setpoint

    DEBUG_SET(DEBUG_RTH, 0, gpsRescueAngle[AI_PITCH]);
    DEBUG_SET(DEBUG_GPS_RESCUE_VELOCITY, 0, velocityP);
    DEBUG_SET(DEBUG_GPS_RESCUE_VELOCITY, 1, velocityD);
    DEBUG_SET(DEBUG_GPS_RESCUE_VELOCITY, 3, rescueState.intent.targetVelocityCmS);
    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 1, rescueState.intent.targetVelocityCmS);

    /**
        Altitude (throttle) controller
    */
    // note that currentAltitudeCm can be updated more frequently than GPS rate from Baro, but this code requires GPS data to update
    // ToDo: use a delta time for changes in currentAltitudeCm, and run more frequently than GPS rate
    const float altitudeError = (rescueState.intent.targetAltitudeCm - rescueState.sensor.currentAltitudeCm) * 0.01f;
    // height above target in metres (negative means too low)
    // at the start, the target starts at current altitude plus one step.  Increases stepwise to intended value.

    // P component
    const float throttleP = gpsRescueConfig()->throttleP * altitudeError;

    // I component
    throttleI += 0.01f * gpsRescueConfig()->throttleI * altitudeError * sampleIntervalNormaliseFactor;
    throttleI = constrainf(throttleI, -1.0f * GPS_RESCUE_MAX_ITERM_THROTTLE, 1.0f * GPS_RESCUE_MAX_ITERM_THROTTLE);
    // up to 20% increase in throttle from I alone

    // D component
    // is error based, so includes positive boost when climbing and negative boost on descent
    float throttleD = ((altitudeError - previousAltitudeError) / sampleIntervalNormaliseFactor);
    previousAltitudeError = altitudeError;

    // Acceleration (Jerk) component
    const float throttleDJerk = 2.0f * (throttleD - previousThrottleD);
    previousThrottleD = throttleD;
    throttleD += throttleDJerk;

    // D Smoothing
    const float movingAvgAltitudeD = 0.5f * (previousThrottleDVal + throttleD);
     // moving average seems to work best here, a lot of sequential up and down in altitude data
    previousThrottleDVal = throttleD;
    throttleD = movingAvgAltitudeD;
    throttleD = previousThrottleD2 + rescueState.sensor.filterK * (throttleD - previousThrottleD2);
    // additional final first order D throttle smoothing
    previousThrottleD2 = throttleD;

    throttleD = 10.0f * gpsRescueConfig()->throttleD * throttleD;

    float tiltAdjustment = 1.0f - getCosTiltAngle(); // 0 = flat, gets to 0.2 correcting on a windy day
    tiltAdjustment *= (gpsRescueConfig()->throttleHover - 1000);
    // if hover is 1300, and adjustment .2, this gives us 0.2*300 or 60 of extra throttle, not much, but useful
    // too much and landings with lots of pitch adjustment, eg windy days, can be a problem

    throttleAdjustment = throttleP + throttleI + throttleD + tiltAdjustment;

    rescueThrottle = gpsRescueConfig()->throttleHover + throttleAdjustment;
    rescueThrottle = constrainf(rescueThrottle, gpsRescueConfig()->throttleMin, gpsRescueConfig()->throttleMax);
    DEBUG_SET(DEBUG_GPS_RESCUE_THROTTLE_PID, 0, throttleP);
    DEBUG_SET(DEBUG_GPS_RESCUE_THROTTLE_PID, 1, throttleD);
}

static void performSanityChecks()
{
    static timeUs_t previousTimeUs = 0; // Last time Stalled/LowSat was checked
    static int8_t secondsStalled = 0; // Stalled movement detection
    static int8_t secondsNotDescending = 0; // Stalled descent detection
    static int8_t secondsNotAscending = 0; // Stalled ascent detection
    static int32_t prevAltitudeCm = 0.0f; // to calculate ascent or descent change
    static int8_t secondsLowSats = 0; // Minimum sat detection
    static int8_t secondsDoingNothing = 0; // Limit on doing nothing
    const timeUs_t currentTimeUs = micros();

    if (rescueState.phase == RESCUE_IDLE) {
        rescueState.failure = RESCUE_HEALTHY;
        return;
    } else if (rescueState.phase == RESCUE_INITIALIZE) {
        // Initialize internal variables each time GPS Rescue is started
        previousTimeUs = currentTimeUs;
        secondsStalled = 5; // Start the count at 5 to be less forgiving at the beginning
        secondsNotDescending = 0;
        secondsNotAscending = 0;
        prevAltitudeCm = rescueState.sensor.currentAltitudeCm;
        secondsLowSats = 5;  // Start the count at 5 to be less forgiving at the beginning
        secondsDoingNothing = 0;
        return;
    }

    // Handle rescue failures.  Don't disarm for rescue failure during stick induced rescues.
    const bool hardFailsafe = !rxIsReceivingSignal();
    if (rescueState.failure != RESCUE_HEALTHY) {
        if (gpsRescueConfig()->sanityChecks == RESCUE_SANITY_ON) {
            rescueState.phase = RESCUE_ABORT;
        } else if ((gpsRescueConfig()->sanityChecks == RESCUE_SANITY_FS_ONLY) && hardFailsafe) {
            rescueState.phase = RESCUE_ABORT;
        } else {
            rescueState.phase = RESCUE_DO_NOTHING;
        }
    }

    DEBUG_SET(DEBUG_RTH, 2, rescueState.failure);

    // Check if crash recovery mode is active
    if (crashRecoveryModeActive()) {
        rescueState.failure = RESCUE_CRASH_FLIP_DETECTED;
    }

    // Check if GPS comms are healthy
    if (!rescueState.sensor.healthy) {
        rescueState.failure = RESCUE_GPSLOST;
    }

    //  Things that should run at a low refresh rate (such as flyaway detection, etc)
    //  This runs at 1hz
    const timeDelta_t dTime = cmpTimeUs(currentTimeUs, previousTimeUs);
    if (dTime < 1000000) { //1hz
        return;
    }
    previousTimeUs = currentTimeUs;

    if (rescueState.phase == RESCUE_FLY_HOME) {
        secondsStalled = constrain(secondsStalled + ((rescueState.sensor.velocityToHomeCmS < (0.5 * rescueState.intent.targetVelocityCmS)) ? 1 : -1), 0, 20);
        if (secondsStalled == 20) {
#ifdef USE_MAG
            //If there is a mag and has not been disabled, we have to assume is healthy and has been used in imu.c
            if (sensors(SENSOR_MAG) && gpsRescueConfig()->useMag && !magForceDisable) {
                //Try again with mag disabled
                magForceDisable = true;
                secondsStalled = 0;
            } else
#endif
            {
            rescueState.failure = RESCUE_STALLED;
            }
        }
    }

    // These conditions are 'special', in that even with sanity checks off, they should still apply
    if (rescueState.phase == RESCUE_ATTAIN_ALT) {
        secondsNotAscending = constrain(secondsNotAscending + (((rescueState.sensor.currentAltitudeCm - prevAltitudeCm) > (0.5f *  gpsRescueConfig()->ascendRate)) ? -1 : 1), 0, 10);
        if (secondsNotAscending == 10) {
            {
            rescueState.phase = RESCUE_ABORT;
            // if stuck in a tree while climbing, or otherwise can't climb, stop motors and disarm
            }
        }
    } else if (rescueState.phase == RESCUE_LANDING || rescueState.phase == RESCUE_DESCENT) {
        secondsNotDescending = constrain(secondsNotDescending + (((prevAltitudeCm - rescueState.sensor.currentAltitudeCm) > (0.5f * gpsRescueConfig()->descendRate)) ? -1 : 1), 0, 10);
        if (secondsNotDescending == 10) {
            {
            rescueState.phase = RESCUE_ABORT;
            // if stuck in a tree while climbing, or don't disarm on impact, or enable GPS rescue on the ground too close
            }
        }
    } else if (rescueState.phase == RESCUE_DO_NOTHING) {
        secondsDoingNothing = MIN(secondsDoingNothing + 1, 10);
        if (secondsDoingNothing == 10) {
            rescueState.phase = RESCUE_ABORT;
            // prevent indefinite flyaways when sanity checks are off, and
            // time limit the "do nothing" period when a switch initiated failsafe fails sanity checks
            // this is controversial
        }
    }
    prevAltitudeCm = rescueState.sensor.currentAltitudeCm;

    secondsLowSats = constrain(secondsLowSats + ((rescueState.sensor.numSat < (gpsRescueConfig()->minSats / 2)) ? 1 : -1), 0, 10);

    if (secondsLowSats == 10) {
        rescueState.failure = RESCUE_LOWSATS;
    }

    DEBUG_SET(DEBUG_RTH, 3, (secondsStalled * 100 + secondsLowSats)); //Failure can change with no new GPS Data
}

static void sensorUpdate()
{
    static timeUs_t previousDataTimeUs = 0;
    static float prevDistanceToHomeCm = 0.0f;

    rescueState.sensor.currentAltitudeCm = getEstimatedAltitudeCm();
    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 2, rescueState.sensor.currentAltitudeCm);
    // may be updated more frequently than GPS data

    rescueState.sensor.healthy = gpsIsHealthy();

    if (rescueState.phase == RESCUE_LANDING) {
        // do this at sensor update rate, not the much slower GPS rate, for quick disarm
        rescueState.sensor.accMagnitude = (float) sqrtf(sq(acc.accADC[Z]) + sq(acc.accADC[X]) + sq(acc.accADC[Y])) * acc.dev.acc_1G_rec;
    }

    if (!newGPSData) {
        return;
    }

    rescueState.sensor.distanceToHomeCm = GPS_distanceToHomeCm;
    rescueState.sensor.distanceToHomeM = rescueState.sensor.distanceToHomeCm / 100.0f;
    rescueState.sensor.groundSpeedCmS = gpsSol.groundSpeed; // cm/s
    rescueState.sensor.directionToHome = GPS_directionToHome;
    rescueState.sensor.numSat = gpsSol.numSat;
    rescueState.sensor.errorAngle = (attitude.values.yaw - rescueState.sensor.directionToHome) * 0.1f;
    // both attitude and direction are in degrees * 10, errorAngle is degrees
    if (rescueState.sensor.errorAngle <= -180) {
        rescueState.sensor.errorAngle += 360;
    } else if (rescueState.sensor.errorAngle > 180) {
        rescueState.sensor.errorAngle -= 360;
    }

    const timeUs_t currentTimeUs = micros();
    const timeDelta_t gpsDataIntervalUs = cmpTimeUs(currentTimeUs, previousDataTimeUs);
    rescueState.sensor.gpsDataIntervalSeconds = constrainf(gpsDataIntervalUs * 0.000001f, 0.01f, 1.0f);
    // Range from 10ms (100hz) to 1000ms (1Hz). Intended to cover common GPS data rates and exclude unusual values.
    previousDataTimeUs = currentTimeUs;

    rescueState.sensor.filterK = pt1FilterGain(0.8, rescueState.sensor.gpsDataIntervalSeconds);
    // 0.8341 for 1hz, 0.5013 for 5hz, 0.3345 for 10hz, 0.1674 for 25Hz, etc

    rescueState.sensor.velocityToHomeCmS = (prevDistanceToHomeCm - rescueState.sensor.distanceToHomeCm) / rescueState.sensor.gpsDataIntervalSeconds;
    // positive = towards home.  First value is useless since prevDistanceToHomeCm was zero.
    prevDistanceToHomeCm = rescueState.sensor.distanceToHomeCm;

    rescueState.sensor.ascendStepCm = rescueState.sensor.gpsDataIntervalSeconds * gpsRescueConfig()->ascendRate;
    rescueState.sensor.descendStepCm = rescueState.sensor.gpsDataIntervalSeconds * gpsRescueConfig()->descendRate;
    rescueState.sensor.maxPitchStep = rescueState.sensor.gpsDataIntervalSeconds * GPS_RESCUE_MAX_PITCH_RATE;

    DEBUG_SET(DEBUG_GPS_RESCUE_HEADING, 2, attitude.values.yaw);  // degrees * 10
    DEBUG_SET(DEBUG_GPS_RESCUE_HEADING, 3, rescueState.sensor.directionToHome);  // degrees * 10
    DEBUG_SET(DEBUG_GPS_RESCUE_VELOCITY, 2, rescueState.sensor.velocityToHomeCmS);
    DEBUG_SET(DEBUG_GPS_RESCUE_THROTTLE_PID, 2, rescueState.sensor.currentAltitudeCm);
    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 0, rescueState.sensor.velocityToHomeCmS);
}

// This function checks the following conditions to determine if GPS rescue is available:
// 1. sensor healthy - GPS data is being received.
// 2. GPS has a valid fix.
// 3. GPS number of satellites is less than the minimum configured for GPS rescue.
// Note: this function does not take into account the distance from homepoint etc. (gps_rescue_min_dth) and
// is also independent of the gps_rescue_sanity_checks configuration
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

    secondsLowSats = constrain(secondsLowSats + ((gpsSol.numSat < gpsRescueConfig()->minSats) ? 1 : -1), 0, 2);
    if (secondsLowSats == 2) {
        lowsats = true;
        result = false;
    } else {
        lowsats = false;
    }

    return result;
}

void updateGPSRescueState(void)
// this runs a lot faster than the GPS Data update rate, and runs whether or not rescue is active
{
    if (!FLIGHT_MODE(GPS_RESCUE_MODE)) {
        rescueStop(); // sets phase to idle and does nothing else
    } else if (FLIGHT_MODE(GPS_RESCUE_MODE) && rescueState.phase == RESCUE_IDLE) {
        rescueStart(); // sets phase to initialise if we enter GPS Rescue mode while idle
        rescueAttainPosition(); // Initialise basic parameters when a Rescue starts (can't initialise sensor data reliably)
        performSanityChecks(); // Initialises sanity check values when a Rescue starts
    }

    // Will now be in INITIALIZE mode, if just entered Rescue while IDLE, otherwise stays IDLE

    sensorUpdate(); // always get latest GPS / Altitude data

    uint8_t halfAngle = gpsRescueConfig()->angle / 2;
    bool startedLow = true;
    rescueState.isAvailable = checkGPSRescueIsAvailable();

    switch (rescueState.phase) {
    case RESCUE_IDLE:
        // in Idle phase = NOT in GPS Rescue
        // set maxAltitude for flight
        // update the return altitude and descent distance values, to have valid settings immediately they are needed
        // initialise the target altitude and velocity to current values to minimise D spike on startup
        idleTasks();
        break;
        // sanity checks are bypassed in IDLE mode; instead, failure state is always initialised to HEALTHY
        // target altitude is always set to current altitude.

    case RESCUE_INITIALIZE:
        // Things that should abort the start of a Rescue
        if (!STATE(GPS_FIX_HOME)) {
            // we didn't get a home point on arming
            rescueState.failure = RESCUE_NO_HOME_POINT; // abort, or 20s of do nothing, as per sanity check settings
            rescueState.phase = RESCUE_DO_NOTHING;
        } else if (rescueState.sensor.distanceToHomeM < gpsRescueConfig()->minRescueDth) {
            // Attempt to initiate inside minimum activation distance -> landing mode
            rescueState.intent.targetAltitudeCm = rescueState.sensor.currentAltitudeCm - rescueState.sensor.descendStepCm;
            rescueState.phase = RESCUE_LANDING;
            // start landing from current altitude
        } else {
            startedLow = (rescueState.sensor.currentAltitudeCm <= rescueState.intent.returnAltitudeCm);
            rescueState.phase = RESCUE_ATTAIN_ALT;
        }
        break;

    case RESCUE_ATTAIN_ALT:
        // increase target altitude from current altitude towards the "fly home" altitude at ascent or descent rate
        // sanity check will abort if altitude gain is blocked in a reasonable time
        // exit once we are within one ascend/descend step of the target altitude
        // while climbing, rotate; use pitch only to get velocity to home close to zero if possible
        if (newGPSData) {
            if (startedLow) {
                if (rescueState.sensor.currentAltitudeCm > (rescueState.intent.returnAltitudeCm - rescueState.sensor.ascendStepCm)) {
                    rescueState.phase = RESCUE_ROTATE;
                } else if (rescueState.intent.targetAltitudeCm < rescueState.intent.returnAltitudeCm) {
                    rescueState.intent.targetAltitudeCm += rescueState.sensor.ascendStepCm;
                    // note targetAltitudeCm is dynamically updated each new GPS data in idleTasks()
                }
            } else {
                if (rescueState.sensor.currentAltitudeCm < (rescueState.intent.returnAltitudeCm + rescueState.sensor.descendStepCm)) {
                    rescueState.phase = RESCUE_ROTATE;
                } else if (rescueState.intent.targetAltitudeCm > rescueState.intent.returnAltitudeCm) {
                    rescueState.intent.targetAltitudeCm -= rescueState.sensor.descendStepCm;
                }
            }
            // try to correct velocity relative to home by allowing pitch corrections with zero target velocity
            rescueState.intent.targetVelocityCmS = 0;
            rescueState.intent.minAnglePitchDeg = -halfAngle; // reduces forward velocity
            rescueState.intent.maxAnglePitchDeg = halfAngle;
            rescueState.intent.updateYaw = true; // allow yaw during the climb, since this helps gain altitude
        }
        rescueState.intent.maxAngleRollDeg = 0; // no roll yet
        break;

    case RESCUE_ROTATE:
        // we may bypass attain_alt so this must stand alone.
        // complete the rotation, allowing pitch when pointing towards home
        if (newGPSData) {
            if (rescueState.sensor.errorAngle < 15.0f) {
                rescueState.phase = RESCUE_FLY_HOME;
                // include roll corrections
            } else if (rescueState.sensor.errorAngle < 45.0f) {
                // pitch forward
                rescueState.intent.minAnglePitchDeg = -gpsRescueConfig()->angle;
                rescueState.intent.maxAnglePitchDeg = gpsRescueConfig()->angle;
                rescueState.intent.targetVelocityCmS = gpsRescueConfig()->rescueGroundspeed;
            } else {
                rescueState.intent.minAnglePitchDeg = -halfAngle;
                rescueState.intent.maxAnglePitchDeg = halfAngle;
                rescueState.intent.targetVelocityCmS = 0;
            }
            rescueState.intent.targetAltitudeCm = rescueState.intent.returnAltitudeCm;
            rescueState.intent.updateYaw = true;
        }
        rescueState.intent.maxAngleRollDeg = 0; // no roll until yaw error is less than 15 degrees
        break;

    case RESCUE_FLY_HOME:
        // fly home with full control on all axes, pitching forward to gain speed
        if (newGPSData) {
            if (rescueState.sensor.distanceToHomeM <= rescueState.intent.descentDistanceM) {
            rescueState.phase = RESCUE_DESCENT;
            }
        }
        rescueState.intent.targetAltitudeCm = rescueState.intent.returnAltitudeCm;
        rescueState.intent.targetVelocityCmS = gpsRescueConfig()->rescueGroundspeed;
        rescueState.intent.minAnglePitchDeg = -gpsRescueConfig()->angle;
        rescueState.intent.maxAnglePitchDeg = gpsRescueConfig()->angle;
        rescueState.intent.updateYaw = true;
        rescueState.intent.maxAngleRollDeg = gpsRescueConfig()->angle;
        break;

    case RESCUE_DESCENT:
        // attenuate velocity and altitude targets while updating the heading to home
        // once inside the landing box, stop rotating, just descend
        if (newGPSData) {
            const int32_t targetLandingAltitudeCm = 100.0f * gpsRescueConfig()->targetLandingAltitudeM;
            if (rescueState.sensor.distanceToHomeM > gpsRescueConfig()->targetLandingDistanceM && rescueState.sensor.currentAltitudeCm > targetLandingAltitudeCm) {
                // outside the landing box
                const float proximityToHome = constrainf(rescueState.sensor.distanceToHomeM / rescueState.intent.descentDistanceM, 0.0f, 1.0f);
                const float targetAltitudeModified = (proximityToHome * (rescueState.intent.returnAltitudeCm - targetLandingAltitudeCm)) + targetLandingAltitudeCm;
                // aim for top of box
                float targetAltitudeWhileDescendingCm = 0.0f;
                targetAltitudeWhileDescendingCm = MIN(targetAltitudeWhileDescendingCm, targetAltitudeModified);
                // don't let target altitude increase.  leads to landing 'short of the box' but much safer if it overshoots, won't climb again
                float landingDescendStepCm = rescueState.sensor.descendStepCm * (1.0f + proximityToHome);
                // this is the maximum allowed step in target altitude, a bit faster at the start, since velocity also will be faster at the start
                rescueState.intent.targetAltitudeCm -= constrain((rescueState.sensor.currentAltitudeCm - targetAltitudeWhileDescendingCm), 0, landingDescendStepCm);
                // reduce current altitude towards target altitude, but not by more than landingDescendStepCm
                rescueState.intent.targetVelocityCmS = gpsRescueConfig()->rescueGroundspeed * proximityToHome;
                // slow down as we get closer to home, allowing zero for vertical drops
            } else {
                // go to landing mode once inside the box, with zero groundspeed target
                rescueState.phase = RESCUE_LANDING;
                rescueState.intent.targetAltitudeCm -= rescueState.sensor.descendStepCm;
                // take one descent step off our target altitude now
            }
        }
        rescueState.intent.minAnglePitchDeg = -gpsRescueConfig()->angle; // aids slowing down
        rescueState.intent.maxAnglePitchDeg = gpsRescueConfig()->angle;
        rescueState.intent.updateYaw = true;
        rescueState.intent.maxAngleRollDeg = gpsRescueConfig()->angle;
        break;

    case RESCUE_LANDING:
        // let Level Mode keep the quad flat and true and just lose altitude until it hits the ground
        // it will be moved crabwise by wind and momentum but we don't know how best to respond; better to do nothing at all
        // Note: no iTerm on throttle during landing
        if (newGPSData) {
            rescueState.intent.targetAltitudeCm -= rescueState.sensor.descendStepCm;
            // take one step off target altitude every time we get new GPS data
        }
        // keep descending at descendStepCm until we hit the ground
        // can't use altitude since the "zero altitude" value may be incorrect

        if (rescueState.sensor.accMagnitude > 2.0f) {
            setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
            disarm(DISARM_REASON_GPS_RESCUE);
            rescueState.phase = RESCUE_COMPLETE;
        }

        rescueState.intent.minAnglePitchDeg = -halfAngle;
        rescueState.intent.maxAnglePitchDeg = halfAngle;
        rescueState.intent.targetVelocityCmS = 0.0f;
        rescueState.intent.updateYaw = true;
        rescueState.intent.maxAngleRollDeg = 0;
        // keep current heading and attitude, just drop
        break;

    case RESCUE_COMPLETE:
        rescueStop();
        break;

    case RESCUE_ABORT:
        setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
        disarm(DISARM_REASON_GPS_RESCUE);
        rescueStop();
        break;

    case RESCUE_DO_NOTHING:
        break;
    default:
        break;
    }

    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 3, rescueState.intent.targetAltitudeCm);
    DEBUG_SET(DEBUG_GPS_RESCUE_THROTTLE_PID, 3, rescueState.intent.targetAltitudeCm);
    DEBUG_SET(DEBUG_RTH, 1, rescueState.phase);

    performSanityChecks();

    if (rescueState.phase != RESCUE_IDLE) {
        rescueAttainPosition();
    }

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
