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
#include "sensors/compass.h"

#include "gps_rescue.h"

typedef enum {
    RESCUE_IDLE,
    RESCUE_INITIALIZE,
    RESCUE_ATTAIN_ALT,
    RESCUE_PITCH_FORWARD,
    RESCUE_ROTATE,
    RESCUE_FLY_HOME,
    RESCUE_DESCENT,
    RESCUE_LANDING,
    RESCUE_EMERG_DESCENT,
    RESCUE_DO_NOTHING,
} rescuePhase_e;

typedef enum {
    RESCUE_HEALTHY,
    RESCUE_FLYAWAY,
    RESCUE_GPSLOST,
    RESCUE_LOWSATS,
    RESCUE_CRASHFLIP_DETECTED,
    RESCUE_STALLED,
    RESCUE_TOO_CLOSE,
    RESCUE_NO_HOME_POINT,
    RESCUE_NO_ALTITUDE,
    RESCUE_NO_HEADING,
} rescueFailureState_e;

typedef struct {
    float maxAltitudeCm;
    float returnAltitudeCm;
    float targetAltitudeCm;
    float targetAltitudeStepCm;
    float targetVelocityCmS;
    float descentDistanceCm;
    int8_t secondsFailing;
    float rescueYawRate;
    float yawAttenuator;
    float xyAttenuator;
    float proximityAttenuator;
    bool initialLocationSet;
    float disarmThreshold;
    vector2_t latLonSteps; // lat first to match gpsLocation_t which has latitude first
    float cmToEarthAngle;
    float initialClimbCm;
    bool forceDisableMag;
} rescueIntent_s;

typedef struct {
    bool gpsHealthy;
    float errorAngleDeg;
    float velocityToHomeCmS;
} rescueSensorData_s;

typedef struct {
    rescuePhase_e phase;
    rescueFailureState_e failure;
    rescueSensorData_s sensor;
    rescueIntent_s intent;
    bool isAvailable;
    bool isOK;
} rescueState_s;

#define GPS_RESCUE_MAX_YAW_RATE          180    // deg/sec max yaw rate
#define GPS_RESCUE_ALLOWED_YAW_RANGE   30.0f   // yaw error must be less than this to enter fly home phase, and to pitch during descend()

static const float taskIntervalSeconds = HZ_TO_INTERVAL(TASK_GPS_RESCUE_RATE_HZ); // i.e. 0.01 s
static float rescueYawRate = 0.0f;

rescueState_s rescueState;

void gpsRescueInit(void)
{
    rescueState.intent.cmToEarthAngle = 1.0f / EARTH_ANGLE_TO_CM; // approx 0.898 cm per unit lat at equator
    rescueState.intent.initialClimbCm = gpsRescueConfig()->initialClimbM * 100.0f;
    rescueState.intent.disarmThreshold = gpsRescueConfig()->disarmThreshold * 0.1f;
    rescueState.intent.descentDistanceCm = gpsRescueConfig()->descentDistanceM * 100.0f;
    rescueState.isAvailable = true;
    rescueState.isOK = true;
}

static void rescueStart(void)
{
    rescueState.phase = RESCUE_INITIALIZE;
}

static void rescueStop(void)
{
    rescueState.phase = RESCUE_IDLE;
}


// While idle, update the altitude targets 
static void updateMaxAltitude(void)
{
    // While disarmed, force maxAltitude to zero, unless set_home_point_once is true, when maxAlt is only reset on a power cycle
    if (!ARMING_FLAG(ARMED) && !gpsConfig()->gps_set_home_point_once) {
        rescueState.intent.maxAltitudeCm = 0.0f;
        return;
    }
    // Otherwise store maxAltitude for this armed period
    rescueState.intent.maxAltitudeCm = fmaxf(getAltitudeCm(), rescueState.intent.maxAltitudeCm);
}

static bool isHeadingOK(void)
{
    return (
#ifdef USE_MAG
    compassEnabledAndCalibrated() ||  // Compass is being used and has been calibrated
#endif
    canUseGPSHeading);                // IMU heading has been aligned to GPS course over ground
}

void smoothStart(float *attenuator)
{
    // increases an attenuator value that starts at 0 to 1.0, typically over 1s, each task interval step
    *attenuator += taskIntervalSeconds;
    if (*attenuator > 1.0f) {
        *attenuator = 1.0f;
    }
}

static void controlYaw(void)
{
    // Heading / yaw controller
    if (rescueState.intent.yawAttenuator < 1.0f) {
        smoothStart(&rescueState.intent.yawAttenuator);
    }
    float yawRateTemp = rescueState.sensor.errorAngleDeg * rescueState.intent.yawAttenuator * gpsRescueConfig()->yawP * 0.1f;
    yawRateTemp = constrainf(yawRateTemp, -GPS_RESCUE_MAX_YAW_RATE, GPS_RESCUE_MAX_YAW_RATE);
    yawRateTemp *= GET_DIRECTION(rcControlsConfig()->yaw_control_reversed);
    // rescueYaw is the yaw rate in deg/s to correct the heading error
    rescueYawRate = yawRateTemp;
    DEBUG_SET(DEBUG_GPS_RESCUE_HEADING, 7, rescueYawRate);
}

static void setLatLonSteps(void)
{
    // calculate the latitude and longitude step per cm according to direction to home
    float directionToHomeDegrees = DECIDEGREES_TO_DEGREES(GPS_directionToHome);
    if (directionToHomeDegrees > 180) {
        directionToHomeDegrees -= 360;
    } else if (directionToHomeDegrees < -180) {
        directionToHomeDegrees += 360;
    }
    const float directionToHomeRadians = DEGREES_TO_RADIANS(directionToHomeDegrees);
    rescueState.intent.latLonSteps.v[0] = cos_approx(directionToHomeRadians) * rescueState.intent.cmToEarthAngle;  // Latitude (North)
    rescueState.intent.latLonSteps.v[1] = sin_approx(directionToHomeRadians) * rescueState.intent.cmToEarthAngle / getGpsCosLat(); // Longitude (East)
}

static void updatePosition(void)
{
    if (rescueState.intent.targetVelocityCmS > 0.0f) {
        // only possible in fly home or descend modes
        // move target location along a path, step by step
        // start smoothly over 1s
        const float distanceToMove = rescueState.intent.targetVelocityCmS * getGpsDataIntervalSeconds() * rescueState.intent.xyAttenuator;
        setLatLonSteps(); // update latitude and longitude step from current location to home at current target velocity
        vector2Scale(&rescueState.intent.latLonSteps, &rescueState.intent.latLonSteps, distanceToMove);
        // send steps to update the target location in autopilot.c 
        moveTargetLocation(rescueState.intent.latLonSteps);
        // run the autopilot function that calculates earth frame PID sums and converts to pitch and roll values
        // must have an accurate aircraft heading estimate from the IMU
    }
    posControlOnNewGpsData(); // hold position if zero set velocity, otherwise move at target velocity
    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 1, lrintf(rescueState.intent.targetVelocityCmS));
}

void rescueControlPosition (bool newGpsData)
{
    switch (rescueState.phase) {
        case RESCUE_ROTATE:
        case RESCUE_FLY_HOME:
        case RESCUE_DESCENT:
        case RESCUE_LANDING:
            if (newGpsData) {
                if (!rescueState.intent.initialLocationSet) {
                    // initialise position hold at this location
                    resetPositionControl(&gpsSol.llh, TASK_GPS_RESCUE_RATE_HZ);
                    rescueState.intent.initialLocationSet = true;
                    return; // skip updatePosition until the next GPS data point, which will have a full getGpsDataIntervalSeconds of data
                }
                updatePosition();
            }
            if (rescueState.intent.yawAttenuator < 1.0f) {
                smoothStart(&rescueState.intent.xyAttenuator);
            }
            // smooth and upsample the pitch and roll setpoints for pid.c at gps_rescue task rate
            posControlOutput();
            return;
        default:
            return;
    }
}

bool oneSecondPassed(timeUs_t currentTimeUs, timeUs_t *lastTimeUs) {
    timeDelta_t deltaTime = cmpTimeUs(currentTimeUs, *lastTimeUs);
    if (deltaTime >= 1000000) {
        *lastTimeUs = currentTimeUs;
        return true;
    }
    return false;
}

void rescueDisarmNow(void)
{
    rescueState.intent.secondsFailing = 0; // reset sanity timers so we can re-arm
    rescueStop();
    setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
    disarm(DISARM_REASON_FAILSAFE);
}

void rescueEmergDescent(void)
{
    if (isAltitudeAvailable()) {
        rescueState.phase = RESCUE_EMERG_DESCENT;
        // same as RESCUE_DESCENT phase, but no XY control
        // if baro is available then emergency descent is possible even if the GPS module fails
    } else {
        rescueDisarmNow();
        // no other option but to drop out of the sky
    }
}

static void performSanityChecks(void)
{
    static float prevAltitudeCm = 0.0f;            // to calculate ascent or descent change
    static float prevTargetAltitudeCm = 0.0f;      // to calculate ascent or descent target change
    static float previousDistanceToHomeCm = 0.0f;
    static int8_t secondsLowSats = 0;              // Minimum sat detection
    static int8_t secondsDoingNothing;             // Limit on doing nothing
    const timeUs_t currentTimeUs = micros();

    if (rescueState.phase == RESCUE_IDLE) {
        rescueState.failure = RESCUE_HEALTHY;
        return;
    }

    if (rescueState.phase == RESCUE_INITIALIZE) {
        if (GPS_distanceToHome < 5 && isBelowLandingAltitude()) {
            // the rescue started within 5m of home, or 'on the ground'
            // -> instant disarm, for safety reasons
            rescueDisarmNow();
            //  ** this could be closer perhaps ** ?? should it be landing mode ??
            return;
        }

        // it's a failure if there is no home fix when the rescue started
        if (STATE(GPS_FIX_HOME)) {
            rescueState.failure = RESCUE_HEALTHY;
        } else {
            rescueState.failure = RESCUE_NO_HOME_POINT;
        }

        // Initialize these variables each time a GPS Rescue is started
        prevAltitudeCm = getAltitudeCm();
        prevTargetAltitudeCm = rescueState.intent.targetAltitudeCm;
        previousDistanceToHomeCm = GPS_distanceToHomeCm;
        rescueState.intent.secondsFailing = 0;
        secondsLowSats = 0;
        secondsDoingNothing = 0;
        rescueState.isAvailable = true;
        rescueState.isOK = true;
    }

    const bool hardFailsafe = !isRxReceivingSignal(); // ie true Rx signal loss

    // Handle failures, typically after a 10-20s up/down counter timeout
    // Note that there is 'no going back' once a rescue failure response is initiated
    // either the aircraft is disarmed, or it enters an inevitable descent/landing phase.

    if (rescueState.failure != RESCUE_HEALTHY) {
        switch(gpsRescueConfig()->sanityChecks) {
        case RESCUE_SANITY_ON:
            rescueEmergDescent();
            // if sanity checks are 'ON', handle both switch-initiated, and true Rx signal loss, by attempting to descend and disarm
            break;
        case RESCUE_SANITY_FS_ONLY:
            if (hardFailsafe) {
                rescueEmergDescent(); // as above, for true Rx signal loss rescues
            } else {
                rescueState.phase = RESCUE_DO_NOTHING; // additional 20s allowed to revert the switch
            }
            break;
        default:
            // with sanity checks off, ignore sanity failures (very unsafe)
            // aircraft can climb, drift, or flyaway indefinitely
            // can disarm if a crash is detected, but otherwise will never disarm if stuck in a tree, etc
            // However if no Home Fix, and a true Rx signal loss, it will attempt an emergency descent
            if (!STATE(GPS_FIX_HOME) && hardFailsafe) {
                rescueEmergDescent();
            }
        }
    }

    // Crash detection is enabled in all rescues.  If crash is detected, immediately disarm.
    if (crashRecoveryModeActive()) {
        setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
        rescueDisarmNow();
    }

    DEBUG_SET(DEBUG_RTH, 2, rescueState.phase);

    // ** once per second from now on **

    static timeUs_t lastSanityCheck = 0;
    if (!oneSecondPassed(currentTimeUs, &lastSanityCheck)) {
        return;
    }

    // GPS comms failure always results in rescue failure
    if (!rescueState.sensor.gpsHealthy) {
        rescueState.failure = RESCUE_GPSLOST;
    }

    secondsLowSats += (!STATE(GPS_FIX) || (gpsSol.numSat < GPS_MIN_SAT_COUNT)) ? 1 : -1;
    secondsLowSats = MAX(0, secondsLowSats);
    // note that GPS_MIN_SAT_COUNT is 4, and a 3D fix requires at least 4 sats
    // but we could lose the 3D fix with more than 4 sats.
    // ?? not sure that the GPS_MIN_SAT_COUNT check is useful here ??
    if (secondsLowSats >= 10) {
        rescueState.failure = RESCUE_LOWSATS;
    }

    // calculate altitude control error magnitude
    const float measuredAltitudeChange = getAltitudeCm() - prevAltitudeCm;
    prevAltitudeCm = getAltitudeCm();
    const float targetAltitudeChange = rescueState.intent.targetAltitudeCm - prevTargetAltitudeCm;
    prevTargetAltitudeCm = rescueState.intent.targetAltitudeCm;
    float altitudeControlError;
    if (targetAltitudeChange == 0.0f) {
        altitudeControlError = fabsf(measuredAltitudeChange);
        // in return phase, where targetAltitudeCm may not change, the altitude change in the last second is an error
        // 1.0 means 1m of altitude change in 1s (when there should be no altitude change)
    } else {
        altitudeControlError = measuredAltitudeChange / targetAltitudeChange;
        // 1.0 means perfect control; 0 or negative means total failure
        // > 0.5 means achieving at least 50% of the required altitude change
    }

    // calculate velocity to home to compare to set velocity
    const float velocityToHomeCmS = previousDistanceToHomeCm - GPS_distanceToHomeCm; // cm/s
    previousDistanceToHomeCm = GPS_distanceToHomeCm;

    switch (rescueState.phase) {
    case RESCUE_ATTAIN_ALT:
        rescueState.intent.secondsFailing += altitudeControlError > 0.5f ? -1 : 1;
        rescueState.intent.secondsFailing = MAX(0, rescueState.intent.secondsFailing);
        if (rescueState.intent.secondsFailing >= 10) {
            // if can't climb, enter to descend phase and try to descend normally
            rescueState.phase = RESCUE_DESCENT;
            rescueState.intent.secondsFailing = 0;
        }
        break;
    case RESCUE_PITCH_FORWARD:
        rescueState.intent.secondsFailing = rescueState.intent.secondsFailing + 1;
        if (rescueState.intent.secondsFailing >= 15) {
            rescueState.intent.secondsFailing = 15;
            // if unable to orient the IMU after 15s of forward flight, give up
            rescueState.failure = RESCUE_NO_HEADING;
        }
        break;
    case RESCUE_ROTATE:
        rescueState.intent.secondsFailing = 0;
        break;
    case RESCUE_FLY_HOME:
        rescueState.intent.secondsFailing += (velocityToHomeCmS < 0.2f * rescueState.intent.targetVelocityCmS) ? 1 : -1;
        // note this is velocity in direction of home
        rescueState.intent.secondsFailing = MAX(0, rescueState.intent.secondsFailing);
        if (rescueState.intent.secondsFailing >= 20) {
            rescueState.intent.secondsFailing = 0;
#ifdef USE_MAG
            //If there is an active mag, try again without it
            if (compassEnabledAndCalibrated() && !rescueState.intent.forceDisableMag) {
                //Try again with mag disabled
                rescueState.intent.forceDisableMag = true;
            } else
#endif
            {
                rescueState.failure = RESCUE_FLYAWAY;
            }
        }
        break;
    case RESCUE_EMERG_DESCENT:
        // same as descent, but has no XY control
    case RESCUE_DESCENT:
        rescueState.intent.secondsFailing += altitudeControlError > 0.5f ? -1 : 1;
        rescueState.intent.secondsFailing = MAX(0, rescueState.intent.secondsFailing);
        // on average, must achieve at least half the requested descent rate
        if (rescueState.intent.secondsFailing >= 15) {
            rescueState.phase = RESCUE_LANDING;
            rescueState.intent.secondsFailing = 0;
            // if can't descend, go to landing phase
            // this provides another 15s to make 30s in total of net failure before forced disarm
            // typically a failure to descend would mean being stuck in a tree, or similar
        }
        break;
    case RESCUE_LANDING:
        rescueState.intent.secondsFailing += altitudeControlError > 0.5f ? -1 : 1;
        rescueState.intent.secondsFailing = MAX(0, rescueState.intent.secondsFailing);
        if (rescueState.intent.secondsFailing >= 15) {
            rescueDisarmNow();
            // No alternative but disarm if descent fails during landing mode
            // Will trigger after timeout if the aircraft has landed but did not auto-disarm
        }
        break;
    case RESCUE_DO_NOTHING:
        // switch-induced rescue failure is allowed to drift around for 20s before starting emergency descent
        secondsDoingNothing = secondsDoingNothing + 1;
        if (secondsDoingNothing >= 20) {
            secondsDoingNothing = 0;
            rescueEmergDescent();
        }
        break;
    default:
        // do nothing
        break;
    }
    const int8_t failWarningTime = 5;
    if (rescueState.failure != RESCUE_HEALTHY || rescueState.intent.secondsFailing > failWarningTime || secondsLowSats > failWarningTime || secondsDoingNothing > failWarningTime){
        rescueState.isOK = false;
        // flashes the GPS RESCUE FAIL warning in OSD to warn the pilot visually before a disarm, or in established failure mode
    } else {
        rescueState.isOK = true;
    }
    DEBUG_SET(DEBUG_RTH, 3, rescueState.failure);
    DEBUG_SET(DEBUG_RTH, 4, rescueState.intent.secondsFailing);
    DEBUG_SET(DEBUG_RTH, 5, secondsLowSats);
    DEBUG_SET(DEBUG_RTH, 6, secondsDoingNothing);
}

static void sensorUpdate(bool newGpsData)
{
    rescueState.sensor.gpsHealthy = gpsIsHealthy();

    static float prevDistanceToHomeCm = 0.0f;
    if (newGpsData) {
        rescueState.sensor.velocityToHomeCmS = (prevDistanceToHomeCm - GPS_distanceToHomeCm) * getGpsDataFrequencyHz();
        prevDistanceToHomeCm = GPS_distanceToHomeCm;
        // positive = towards home.  First value is useless since prevDistanceToHomeCm was zero.
    }

    // ** heading values **
    const float bearingToHomeDeg = DECIDEGREES_TO_DEGREES(GPS_directionToHome); // 0 to 360
    const float aircraftHeadingDeg = DECIDEGREES_TO_DEGREES(attitude.values.yaw); // 0 to 360

    rescueState.sensor.errorAngleDeg = aircraftHeadingDeg - bearingToHomeDeg;
    // normalise to -180 ... + 180 for plus and minus yaw corrections
    if (rescueState.sensor.errorAngleDeg <= -180) {
        rescueState.sensor.errorAngleDeg += 360;
    } else if (rescueState.sensor.errorAngleDeg > 180) {
        rescueState.sensor.errorAngleDeg -= 360;
    }

    DEBUG_SET(DEBUG_ATTITUDE, 0, lrintf(aircraftHeadingDeg));
    DEBUG_SET(DEBUG_ATTITUDE, 2, lrintf(rescueState.sensor.velocityToHomeCmS));

    DEBUG_SET(DEBUG_GPS_RESCUE_VELOCITY, 0, lrintf(rescueState.intent.targetVelocityCmS)); // target velocity to home
    DEBUG_SET(DEBUG_GPS_RESCUE_VELOCITY, 1, lrintf(rescueState.sensor.velocityToHomeCmS)); // target velocity to home
    DEBUG_SET(DEBUG_GPS_RESCUE_VELOCITY, 2, lrintf(rescueState.intent.latLonSteps.v[0] * 100.0f)); // latitude step
    DEBUG_SET(DEBUG_GPS_RESCUE_VELOCITY, 3, lrintf(rescueState.intent.latLonSteps.v[1] * 100.0f)); // longitude step

    DEBUG_SET(DEBUG_GPS_RESCUE_HEADING, 0, lrintf(rescueState.sensor.velocityToHomeCmS));
    DEBUG_SET(DEBUG_GPS_RESCUE_HEADING, 1, gpsSol.groundCourse);            // deg * 10
    DEBUG_SET(DEBUG_GPS_RESCUE_HEADING, 2, attitude.values.yaw);            // deg * 10
    DEBUG_SET(DEBUG_GPS_RESCUE_HEADING, 3, GPS_directionToHome);            // deg * 10

    const float currentAltitudeCm = getAltitudeCm();
    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 0, lrintf(rescueState.sensor.velocityToHomeCmS));
    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 2, lrintf(currentAltitudeCm));
    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 4, lrintf(aircraftHeadingDeg));    // estimated heading of the quad (direction nose is pointing in)
    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 5, lrintf(bearingToHomeDeg));      // angle to home derived from GPS location and home position
}

static void disarmOnImpact(void)
{
    if (acc.accMagnitude > rescueState.intent.disarmThreshold) {
        setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
        disarm(DISARM_REASON_GPS_RESCUE);
        rescueStop();
    }
}

void initDescent(void)
{
    rescueState.intent.proximityAttenuator = GPS_distanceToHomeCm / rescueState.intent.descentDistanceCm;
}

void descend(bool newGpsData)
{
    if (newGpsData) {
        rescueState.intent.proximityAttenuator = GPS_distanceToHomeCm / rescueState.intent.descentDistanceCm;
        rescueState.intent.targetVelocityCmS = gpsRescueConfig()->groundSpeedCmS * rescueState.intent.proximityAttenuator;
    }

    float altitudeStepCm = taskIntervalSeconds * gpsRescueConfig()->descendRate;

    // at or below 10m: descend at 0.6x set value; above 10m, descend faster, to max 3.0x at 50m
    altitudeStepCm *= scaleRangef(constrainf(rescueState.intent.targetAltitudeCm, 1000, 5000), 1000, 5000, 0.6f, 3.0f);

    rescueState.intent.targetAltitudeStepCm = -altitudeStepCm;
    rescueState.intent.targetAltitudeCm -= altitudeStepCm;

    rescueYawRate = 0.0f; // make sure yaw rate is zero
}


void initRescueValues (void)
{
    if (GPS_distanceToHomeCm < gpsRescueConfig()->minStartDistM * 100.0f) {
        rescueState.intent.returnAltitudeCm = fmaxf(500.0f, getAltitudeCm() + rescueState.intent.initialClimbCm);
        // if rescue starts within minStartDistM, climb at least 5m above arming height
    } else {
        // set return altitude according to user configuration
        switch (gpsRescueConfig()->altitudeMode) {
            case GPS_RESCUE_ALT_MODE_FIXED:
                rescueState.intent.returnAltitudeCm = gpsRescueConfig()->returnAltitudeM * 100.0f;
                break;
            case GPS_RESCUE_ALT_MODE_CURRENT:
                // climb above current altitude, but always return at least initial height above takeoff point, in case current altitude was negative
                rescueState.intent.returnAltitudeCm = fmaxf(rescueState.intent.initialClimbCm, getAltitudeCm() + rescueState.intent.initialClimbCm);
                break;
            case GPS_RESCUE_ALT_MODE_MAX:
            default:
                rescueState.intent.returnAltitudeCm = rescueState.intent.maxAltitudeCm + rescueState.intent.initialClimbCm;
                break;
        }
    }
    rescueState.intent.targetAltitudeCm = getAltitudeCm();  // initial target altitude is current altitude
    rescueState.intent.targetAltitudeStepCm = 0.0f;         // hold at this altitude
    // initialise altitude control
    resetAltitudeControl();

    rescueState.sensor.errorAngleDeg = 0.0f;       // prevent yaw adjustments
    rescueYawRate = 0.0f;                          // no yaw until climb is complete
    rescueState.intent.yawAttenuator = 0.0f;       // for a smooth start to the yaw

    rescueState.intent.targetVelocityCmS = 0.0f;   // zero velocity
    rescueState.sensor.velocityToHomeCmS = 0.0f;

    rescueState.intent.initialLocationSet = false;
    rescueState.intent.xyAttenuator = 0.0f;        // for a slower start to gaining velocity with less overshoot from excessive iTerm
    autopilotAngle[AI_ROLL] = 0.0f;
    autopilotAngle[AI_PITCH] = 0.0f;
    vector2Zero(&rescueState.intent.latLonSteps);
    rescueState.intent.forceDisableMag = false; // use Mag every rescue start even if it failed on a previous rescue
}

static void checkGPSRescueIsAvailable(void)
{
    bool rescueAvailable = true;
    if (!gpsIsHealthy() || !STATE(GPS_FIX_HOME) || !isHeadingOK() || !isAltitudeAvailable()) {
        rescueAvailable = false;
    }
    // note that GPS_FIX_HOME is set at arm time, and requires a 3D fix and minSats at that time
    // Flashes "RESCUE N/A" in the OSD if triggered
    rescueState.isAvailable = rescueAvailable;
}

void gpsRescueUpdate(void)
// runs at gpsRescueTaskIntervalSeconds, and runs whether or not rescue is active
{
    static uint16_t gpsStamp = 0;
    bool newGpsData = gpsHasNewData(&gpsStamp);

    if (!FLIGHT_MODE(GPS_RESCUE_MODE)) {
        rescueStop(); // sets phase to RESCUE_IDLE
    } else if (FLIGHT_MODE(GPS_RESCUE_MODE) && rescueState.phase == RESCUE_IDLE) {
        rescueStart(); // sets phase to RESCUE_INITIALIZE
    }

    sensorUpdate(newGpsData); // get latest velocity to home and Altitude data, set some defaults

    checkGPSRescueIsAvailable();
    // Show the 'RESCUE N/A' warning in OSD if Rescue is unavailable
    // *** strangely, checkGPSRescueIsAvailable won't work properly if put within performSanityChecks() ??? ***

    performSanityChecks();

    static bool returnAltitudeLow = true;

    switch (rescueState.phase) {
    case RESCUE_IDLE:
        updateMaxAltitude();
        break;
    case RESCUE_INITIALIZE:
        initRescueValues();
        returnAltitudeLow = getAltitudeCm() < rescueState.intent.returnAltitudeCm;
        rescueState.phase = RESCUE_ATTAIN_ALT;
        break;

    case RESCUE_ATTAIN_ALT:
        // increment the target altitude until the craft reaches target altitude
        // the target altitude may increase above returnAltitude if the craft lags target
        // sanity check will disarm if altitude gain is blocked for a cumulative period
        if (returnAltitudeLow == (getAltitudeCm() < rescueState.intent.returnAltitudeCm)) {
            // we started low, and still are low; also true if we started high, and still are too high
            rescueState.intent.targetAltitudeStepCm = (returnAltitudeLow ? gpsRescueConfig()->ascendRate : -1.0f * gpsRescueConfig()->descendRate) * taskIntervalSeconds;
            rescueState.intent.targetAltitudeCm += rescueState.intent.targetAltitudeStepCm;
        } else {
            // target altitude achieved
            rescueState.intent.targetAltitudeCm = rescueState.intent.returnAltitudeCm;
            rescueState.intent.targetAltitudeStepCm = 0.0f;
            rescueState.intent.secondsFailing = 0;
            if (isHeadingOK()) {
                rescueState.phase = RESCUE_ROTATE;
            } else {
                // if no Mag and IMU not oriented, pitch forward until oriented
                // typically requires about 50m of forward flight at this angle
                rescueState.phase = RESCUE_PITCH_FORWARD;
                autopilotAngle[AI_ROLL] = 0.0f;
                autopilotAngle[AI_PITCH] = 35.0f;
            }
        }
        break;

    case RESCUE_PITCH_FORWARD:
        // pitch forward at high angle until IMU is oriented to  GPS course over ground
        // only applies if Mag is not available and IMU was not yet oriented
        if (isHeadingOK()) {
            rescueState.phase = RESCUE_ROTATE;
        }
        break;

    case RESCUE_ROTATE:
        // accumulate yaw authority smoothly
        controlYaw();
        if (fabsf(rescueState.sensor.errorAngleDeg) < GPS_RESCUE_ALLOWED_YAW_RANGE) {
            // yaw angle error is small enough to allow us to enter fly home or descend phase
            if (GPS_distanceToHomeCm < rescueState.intent.descentDistanceCm) {
                rescueState.phase = RESCUE_DESCENT; // enter descend phase
                initDescent();
            } else {
                rescueState.phase = RESCUE_FLY_HOME; // enter fly home phase
                rescueState.intent.targetVelocityCmS = gpsRescueConfig()->groundSpeedCmS;
            }
            rescueState.intent.secondsFailing = 0; // reset sanity timer for flight home
        }
        break;

    case RESCUE_FLY_HOME:
        controlYaw();
        if (newGpsData) {
            if (GPS_distanceToHomeCm < rescueState.intent.descentDistanceCm) {
                rescueState.phase = RESCUE_DESCENT; // enter descend phase
                initDescent();
                rescueState.intent.secondsFailing = 0; // reset sanity timer for descent
            }
        }
        break;

    case RESCUE_DESCENT:
        descend(newGpsData);
        if (isBelowLandingAltitude()) {
            // enter landing mode once below landing altitude
            rescueState.phase = RESCUE_LANDING;
            rescueState.intent.secondsFailing = 0; // reset sanity timer for landing
        }
        break;

    case RESCUE_LANDING:
        // same as Descent mode, but with a more sensitive disarm threshold
        descend(newGpsData);
        disarmOnImpact();
        break;

    case RESCUE_EMERG_DESCENT:
        // like Descent mode, but no XY control, and a sensitive disarm threshold
        descend(newGpsData);
        disarmOnImpact();
        break;

    case RESCUE_DO_NOTHING:
        // no active control, except altitude; just hovers and will drift
        autopilotAngle[AI_ROLL] = 0.0f;
        autopilotAngle[AI_PITCH] = 0.0f;
        rescueState.intent.targetAltitudeStepCm = 0.0f;
        rescueYawRate = 0.0f;
        disarmOnImpact();
        break;

    default:
        break;
    }

    DEBUG_SET(DEBUG_GPS_RESCUE_VELOCITY, 1, rescueState.phase);
    DEBUG_SET(DEBUG_ATTITUDE, 5, rescueState.phase);
    DEBUG_SET(DEBUG_ATTITUDE, 6, lrintf(rescueState.intent.targetVelocityCmS));
    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 3, lrintf(rescueState.intent.targetAltitudeCm));
    DEBUG_SET(DEBUG_RTH, 0, lrintf(rescueState.intent.maxAltitudeCm / 10.0f));
    DEBUG_SET(DEBUG_RTH, 5, (rescueState.isAvailable == true ? 1 : 0));
    DEBUG_SET(DEBUG_RTH, 6, (rescueState.isOK == true ? 1 : 0));

    // only active in rotate, fly home, descent and landing phases:
    rescueControlPosition(newGpsData);

    // autopilot control of altitude is always active; rescue won't start unless altitude data is available
    if (rescueState.phase > RESCUE_INITIALIZE) {
        altitudeControl(rescueState.intent.targetAltitudeCm, taskIntervalSeconds, rescueState.intent.targetAltitudeStepCm);
    }
}

float gpsRescueGetYawRate(void)
{
    return rescueYawRate; // the control yaw value for rc.c to be used while flightMode gps_rescue is active.
}

bool gpsRescueIsConfigured(void)
{
    return failsafeConfig()->failsafe_procedure == FAILSAFE_PROCEDURE_GPS_RESCUE || isModeActivationConditionPresent(BOXGPSRESCUE);
}

bool gpsRescueIsAvailable(void)
{
    return rescueState.isAvailable; // when false, flashes `RESCUE N/A` when not available (IMU disoriented, no Home Point)
}

bool gpsRescueIsOK(void)
{
    return rescueState.isOK; // when false, flashes `RESCUE FAIL` in OSD when failure time exceeds 5s or when in a failure mode
}

#ifdef USE_MAG
bool gpsRescueDisableMag(void)
{
    // Enable mag on user request, but don't use it during fly home or if force disabled
    // Note that while flying home the course over ground from GPS provides a heading that is less affected by wind
    return rescueState.intent.forceDisableMag;
}
#endif // USE_MAG
#endif // USE_GPS_RESCUE
#endif // !USE_WING