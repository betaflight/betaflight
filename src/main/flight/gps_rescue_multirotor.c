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
#include "flight/position_estimator.h"

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
    float targetAltitudeVelCmS;
    float pitchAngleLimitDeg;
    float rollAngleLimitDeg;
    float descentDistanceCm;
    float descentDistanceM;
    int8_t secondsFailing;
    float yawAttenuator;
    float targetVelocityCmS;
    float xyAttenuator;
    float proximityAttenuator;
    vector2_t stepEF; // distance to move the craft along the path each iteration
    float cmToEarthAngle;
    float initialClimbCm;
} rescueIntent_s;

typedef struct {
    bool gpsHealthy;
    bool isHeadingOK;
    bool newGpsData;
    bool positionXYAvailable;
    float aircraftHeadingDeg;
    float bearingToHomeDeg;
    float errorAngleDeg;
    float distanceToHomeCm;
    float velocityCmS; // for debugs only
    vector2_t currentVelocityV;
    vector2_t currentPositionV; // relative to arming location, not absolute
    vector2_t previousPositionV;
        float currentAltitudeCm;
} rescueSensorData_s;

typedef struct {
    rescuePhase_e phase;
    rescueFailureState_e failure;
    rescueSensorData_s sensor;
    rescueIntent_s intent;
    bool isAvailable;
    bool isOK;
} rescueState_s;
rescueState_s rescueState;

#define GPS_RESCUE_MAX_YAW_RATE          180    // deg/sec max yaw rate
#define GPS_RESCUE_ALLOWED_YAW_RANGE   30.0f   // yaw error must be less than this to enter fly home phase, and to pitch during descend()
#define GPS_RESCUE_ACCEPT_RADIUS       20.0f // ignore closer than this Cm
static const float gpsRescueTaskIntervalSeconds = HZ_TO_INTERVAL(TASK_GPS_RESCUE_RATE_HZ); // i.e. 0.01s
static float rescueYawRate = 0.0f;



void gpsRescueInit(void)
{
    rescueState.intent.cmToEarthAngle = 1.0f / EARTH_ANGLE_TO_CM; // approx 0.898 cm per unit lat at equator
    rescueState.intent.initialClimbCm = gpsRescueConfig()->initialClimbM * 100.0f;
    rescueState.intent.descentDistanceCm = gpsRescueConfig()->descentDistanceM * 100.0f;
    rescueState.sensor.currentPositionV  = (vector2_t){{0.0f, 0.0f}};
    rescueState.intent.targetAltitudeVelCmS = 0.0f;

    rescueState.sensor.previousPositionV = rescueState.sensor.currentPositionV;
    // Zero out flight angles to keep the craft flat on initiation
    autopilotAngle[AI_ROLL] = 0.0f;
    autopilotAngle[AI_PITCH] = 0.0f;
    rescueState.isAvailable = true;
    rescueState.sensor.isHeadingOK = true;
    rescueState.isOK = true;

}

static void rescueStart(void)
{
     initPositionHold(); // initialise position hold at current location

    rescueState.phase = RESCUE_INITIALIZE;
}

static void rescueStop(void)
{
    rescueState.phase = RESCUE_IDLE;
    pitchForwardOverride(false);
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
    rescueState.intent.maxAltitudeCm = fmaxf(rescueState.sensor.currentAltitudeCm, rescueState.intent.maxAltitudeCm);
    rescueState.intent.targetAltitudeCm = rescueState.sensor.currentAltitudeCm;

}

static void sensorUpdate(void)
{
    static uint16_t gpsStamp = 0;
    rescueState.sensor.newGpsData = gpsHasNewData(&gpsStamp);
    rescueState.sensor.isHeadingOK = imuIsHeadingValid();
    rescueState.sensor.gpsHealthy = gpsIsHealthy();

rescueState.sensor.currentAltitudeCm = getAltitudeCmControl();

rescueState.sensor.positionXYAvailable = positionEstimatorIsValidXY();
if (rescueState.sensor.positionXYAvailable) {
    const positionEstimate3d_t *est = positionEstimatorGetEstimate();
    rescueState.sensor.currentPositionV  = *(const vector2_t *)&est->position.v;
    rescueState.sensor.currentVelocityV  = *(const vector2_t *)&est->velocity.v;
    rescueState.sensor.previousPositionV = rescueState.sensor.currentPositionV;
}
    rescueState.sensor.distanceToHomeCm = vector2Norm(&rescueState.sensor.currentPositionV);
    rescueState.sensor.velocityCmS =vector2Norm(&rescueState.sensor.currentVelocityV); // only for debugs

    rescueState.sensor.aircraftHeadingDeg = DECIDEGREES_TO_DEGREES(attitude.values.yaw); // for debugs only

    if (rescueState.sensor.distanceToHomeCm > GPS_RESCUE_ACCEPT_RADIUS) {
        const float headingRad = DECIDEGREES_TO_RADIANS(attitude.values.yaw);
        vector2_t headingV;

        headingV.v[EF_NORTH] = cosf(headingRad);
        headingV.v[EF_EAST]  = sinf(headingRad);

        const float dotProduct = vector2Dot(&headingV, &rescueState.sensor.currentPositionV);
        const float crossProduct = vector2Cross(&headingV, &rescueState.sensor.currentPositionV);

        rescueState.sensor.errorAngleDeg = RADIANS_TO_DEGREES(atan2f(-crossProduct, -dotProduct));
    } else {
        rescueState.sensor.errorAngleDeg = 0.0f;
    }

    DEBUG_SET(DEBUG_ATTITUDE, 0, lrintf(rescueState.sensor.aircraftHeadingDeg));
    DEBUG_SET(DEBUG_ATTITUDE, 2, lrintf(rescueState.sensor.velocityCmS));

    DEBUG_SET(DEBUG_GPS_RESCUE_VELOCITY, 0, lrintf(rescueState.intent.targetVelocityCmS));
    DEBUG_SET(DEBUG_GPS_RESCUE_VELOCITY, 1, lrintf(rescueState.sensor.velocityCmS));
    DEBUG_SET(DEBUG_GPS_RESCUE_VELOCITY, 2, lrintf(rescueState.intent.stepEF.v[EF_EAST] * 100.0f));
    DEBUG_SET(DEBUG_GPS_RESCUE_VELOCITY, 3, lrintf(rescueState.intent.stepEF.v[EF_NORTH] * 100.0f));

    DEBUG_SET(DEBUG_GPS_RESCUE_HEADING, 0, lrintf(rescueState.sensor.velocityCmS));
    DEBUG_SET(DEBUG_GPS_RESCUE_HEADING, 1, gpsSol.groundCourse);
    DEBUG_SET(DEBUG_GPS_RESCUE_HEADING, 2, attitude.values.yaw);
    DEBUG_SET(DEBUG_GPS_RESCUE_HEADING, 3, GPS_directionToHome);

    const float currentAltitudeCm = getAltitudeCm();
    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 0, lrintf(rescueState.sensor.velocityCmS));
    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 2, lrintf(currentAltitudeCm));
    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 4, lrintf(rescueState.sensor.aircraftHeadingDeg));
    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 5, lrintf(rescueState.sensor.bearingToHomeDeg));
}

static void updateYawStartupAttenuator(void)
{
    if (rescueState.intent.yawAttenuator < 1.0f) {
        rescueState.intent.yawAttenuator += gpsRescueTaskIntervalSeconds;
        if (rescueState.intent.yawAttenuator > 1.0f) {
            rescueState.intent.yawAttenuator = 1.0f;
        }
    }
}
static void updateVelocityStartupAttenuator(void)
{
    if (rescueState.intent.xyAttenuator < 1.0f) {
        rescueState.intent.xyAttenuator += gpsRescueTaskIntervalSeconds;
        if (rescueState.intent.xyAttenuator > 1.0f) {
            rescueState.intent.xyAttenuator = 1.0f;
        }
    }
}


static void controlYaw(void)
{
    float yawRateTemp = rescueState.sensor.errorAngleDeg * rescueState.intent.yawAttenuator * gpsRescueConfig()->yawP * 0.1f;
    yawRateTemp = constrainf(yawRateTemp, -GPS_RESCUE_MAX_YAW_RATE, GPS_RESCUE_MAX_YAW_RATE);
    yawRateTemp *= GET_DIRECTION(rcControlsConfig()->yaw_control_reversed);
    // rescueYaw is the yaw rate in deg/s to correct the heading error
    rescueYawRate = yawRateTemp;
    DEBUG_SET(DEBUG_GPS_RESCUE_HEADING, 7, rescueYawRate);
}

static void calculateTargetStep(void)
{
    if (rescueState.sensor.distanceToHomeCm > GPS_RESCUE_ACCEPT_RADIUS) {
        const float distanceToMoveCm = rescueState.intent.targetVelocityCmS * gpsRescueTaskIntervalSeconds * rescueState.intent.xyAttenuator;
        const float scale = distanceToMoveCm / rescueState.sensor.distanceToHomeCm;
            DEBUG_SET(DEBUG_RTH, 5, lrintf(scale*100.0f));

        vector2Scale(&rescueState.intent.stepEF, &rescueState.sensor.currentPositionV, -scale);
    } else {
        vector2Zero(&rescueState.intent.stepEF);
    }

    moveTargetLocation(&rescueState.intent.stepEF, TASK_GPS_RESCUE_RATE_HZ, false);
}

static void clearTargetStep(void)
{
    vector2Zero(&rescueState.intent.stepEF);
    moveTargetLocation(&rescueState.intent.stepEF, TASK_GPS_RESCUE_RATE_HZ, false);
}


static void controlAltitude(void)
{
    const float vzLim = 3.0f * fmaxf((float)gpsRescueConfig()->ascendRate, (float)gpsRescueConfig()->descendRate);
    altitudeControl(rescueState.intent.targetAltitudeCm, gpsRescueTaskIntervalSeconds, rescueState.intent.targetAltitudeVelCmS, vzLim);
}

bool oneSecondPassed(timeUs_t currentTimeUs, timeUs_t *lastTimeUs) {
    timeDelta_t deltaTime = cmpTimeUs(currentTimeUs, *lastTimeUs);
    if (deltaTime >= 1000000) {
        *lastTimeUs = currentTimeUs;
        return true;
    }
    return false;
}

static void rescueDisarmNow(void)
{
    rescueState.intent.secondsFailing = 0;
    rescueStop();
    setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
    disarm(DISARM_REASON_FAILSAFE);
}

static void rescueEmergDescent(void)
{
    if (isAltitudeAvailable()) {
        rescueState.phase = RESCUE_EMERG_DESCENT;
        // same as RESCUE_DESCENT phase, but no XY control
        // if baro is available then emergency descent is possible even if the GPS module fails
    } else {
        rescueDisarmNow();
    }
}


static void performSanityChecks(void)
{
    static float prevAltitudeCm = 0.0f;
    static float prevTargetAltitudeCm = 0.0f;
    static float previousDistanceToHomeCm = 0.0f;
    static int8_t secondsLowSats = 0;
    static int8_t secondsDoingNothing;
    const timeUs_t currentTimeUs = micros();

    if (rescueState.phase == RESCUE_IDLE) {
        rescueState.failure = RESCUE_HEALTHY;
        return;
    }

    DEBUG_SET(DEBUG_RTH, 2, rescueState.phase);
    DEBUG_SET(DEBUG_RTH, 3, rescueState.failure);
    DEBUG_SET(DEBUG_RTH, 4, rescueState.intent.secondsFailing);
    DEBUG_SET(DEBUG_RTH, 5, secondsLowSats);
    DEBUG_SET(DEBUG_RTH, 6, lrintf(rescueState.sensor.distanceToHomeCm));

    if (rescueState.phase == RESCUE_INITIALIZE) {
        if (rescueState.sensor.distanceToHomeCm < GPS_RESCUE_ACCEPT_RADIUS && isBelowLandingAltitude()) {
            rescueDisarmNow();
            return;
        }

        // Check home point fix ONLY during initialization pass
        if (STATE(GPS_FIX_HOME)) {
            rescueState.failure = RESCUE_HEALTHY;
        } else {
            rescueState.failure = RESCUE_NO_HOME_POINT;
        }

        prevAltitudeCm = getAltitudeCm();
        prevTargetAltitudeCm = rescueState.intent.targetAltitudeCm;
        previousDistanceToHomeCm = rescueState.sensor.distanceToHomeCm;
        rescueState.intent.secondsFailing = 0;
        secondsLowSats = 0;
        secondsDoingNothing = 0;
        rescueState.isOK = true;
    }

    if (crashRecoveryModeActive()) {
        setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
        rescueDisarmNow();
    }

    const bool hardFailsafe = !isRxReceivingSignal(); 

    if (rescueState.failure == RESCUE_HEALTHY) {
        rescueState.isOK = true;
    } else {
        rescueState.isOK = false;
        switch(gpsRescueConfig()->sanityChecks) {
        case RESCUE_SANITY_ON:
            rescueEmergDescent();
            return;
        case RESCUE_SANITY_FS_ONLY:
            if (hardFailsafe) {
                rescueEmergDescent();
                return;
            }
            break;
        default: 
            if ((!rescueState.isAvailable) && hardFailsafe) {
                rescueEmergDescent();
                return;
            }
            break;
        }
    }

    DEBUG_SET(DEBUG_RTH, 2, rescueState.phase);

    static timeUs_t lastSanityCheck = 0;
    if (!oneSecondPassed(currentTimeUs, &lastSanityCheck)) {
        return;
    }

    if (!rescueState.sensor.gpsHealthy) {
        rescueState.failure = RESCUE_GPSLOST;
    }
    
    secondsLowSats += (!STATE(GPS_FIX) || (gpsSol.numSat < GPS_MIN_SAT_COUNT)) ? 1 : -1;
    secondsLowSats = MAX(0, secondsLowSats);
    if (secondsLowSats >= 15) {
        rescueState.failure = RESCUE_LOWSATS;
    }

    const float measuredAltitudeChange = getAltitudeCm() - prevAltitudeCm;
    prevAltitudeCm = getAltitudeCm();
    const float targetAltitudeChange = rescueState.intent.targetAltitudeCm - prevTargetAltitudeCm;
    prevTargetAltitudeCm = rescueState.intent.targetAltitudeCm;

    float altitudeControlError;
    if (targetAltitudeChange == 0.0f) {
        altitudeControlError = fabsf(measuredAltitudeChange);
    } else {
        altitudeControlError = measuredAltitudeChange / targetAltitudeChange;
    }

    const float velocityToHomeCmS = previousDistanceToHomeCm - rescueState.sensor.distanceToHomeCm; 
    previousDistanceToHomeCm = rescueState.sensor.distanceToHomeCm;

    switch (rescueState.phase) {
    case RESCUE_ATTAIN_ALT:
        rescueState.intent.secondsFailing += altitudeControlError > 0.5f ? -1 : 1;
        rescueState.intent.secondsFailing = MAX(0, rescueState.intent.secondsFailing);
        if (rescueState.intent.secondsFailing >= 10) {
            rescueState.intent.targetAltitudeCm = getAltitudeCm();
            rescueState.failure = RESCUE_STALLED;
            rescueState.intent.secondsFailing = 0;
        }
        break;

    case RESCUE_PITCH_FORWARD:
        // attempting to re-orient IMU by forcing pitch forward
        rescueState.intent.secondsFailing += 1;
        if (rescueState.intent.secondsFailing >= 15) {
            pitchForwardOverride(false); //give up after 15s  TODO: check if this is enough time or too much
            rescueEmergDescent();
            rescueState.intent.secondsFailing = 0;
        }
        break;

     case RESCUE_ROTATE:
        rescueState.intent.secondsFailing += 1;
        if (rescueState.intent.secondsFailing >= 10) {
            //almost never fails to rotate to the required angle, so this should never trigger
            // does not evaluate whether  attitude.values.yaw  is correct
            rescueState.phase = RESCUE_FLY_HOME;
            rescueState.intent.targetVelocityCmS = gpsRescueConfig()->groundSpeedCmS;
            rescueState.intent.secondsFailing = 0;
        }
         break;

    case RESCUE_FLY_HOME:
        rescueState.intent.secondsFailing += (velocityToHomeCmS < 0.2f * rescueState.intent.targetVelocityCmS) ? 1 : -1;
        rescueState.intent.secondsFailing = MAX(0, rescueState.intent.secondsFailing);
        if (rescueState.intent.secondsFailing >= 20) {
            rescueState.intent.secondsFailing = 0;
            rescueState.failure = RESCUE_FLYAWAY;
        }
        break;
    case RESCUE_DO_NOTHING:
        secondsDoingNothing = secondsDoingNothing + 1;
        if (secondsDoingNothing >= 15) {
            secondsDoingNothing = 0;
            rescueState.phase = RESCUE_DESCENT;
        }
        break;
    case RESCUE_DESCENT:
        rescueState.intent.secondsFailing += altitudeControlError > 0.5f ? -1 : 1;
        rescueState.intent.secondsFailing = MAX(0, rescueState.intent.secondsFailing);
        if (rescueState.intent.secondsFailing >= 15) {
            rescueState.phase = RESCUE_LANDING;
            rescueState.intent.secondsFailing = 0;
        }
        break;
    case RESCUE_LANDING:
        rescueState.intent.secondsFailing += altitudeControlError > 0.5f ? -1 : 1;
        rescueState.intent.secondsFailing = MAX(0, rescueState.intent.secondsFailing);
        if (rescueState.intent.secondsFailing >= 15) {
            rescueDisarmNow();
        }
        break;
    default:
        break;
    }
}


static void initDescent(void)
{
    if (rescueState.sensor.distanceToHomeCm < rescueState.intent.descentDistanceCm) {
        rescueState.intent.descentDistanceCm = rescueState.sensor.distanceToHomeCm;
    }
}
static void descend(void)
{
   if (rescueState.intent.descentDistanceCm > GPS_RESCUE_ACCEPT_RADIUS) {
        float attenuator = rescueState.sensor.distanceToHomeCm / rescueState.intent.descentDistanceCm;
        attenuator = constrainf(attenuator, 0.0f, 1.0f);
        rescueState.intent.targetVelocityCmS = gpsRescueConfig()->groundSpeedCmS * attenuator;
    } else {
        rescueState.intent.targetVelocityCmS = 0.0f;
    }
    float verticalVelMax = -(float)gpsRescueConfig()->descendRate;
    float verticalVelAttenuator = scaleRangef(constrainf(rescueState.intent.targetAltitudeCm, 1000, 5000), 1000, 5000, 0.6f, 3.0f);
    rescueState.intent.targetAltitudeVelCmS = verticalVelMax * verticalVelAttenuator;
    rescueYawRate = 0.0f; // keep yaw rate  zero in case we float past the home point
}

void initRescueValues(void)
{
   rescueState.intent.descentDistanceCm = gpsRescueConfig()->descentDistanceM * 100.0f;
    if (rescueState.sensor.distanceToHomeCm < gpsRescueConfig()->minStartDistM * 100.0f) {
        // 7.5m high for close-range headroom
        rescueState.intent.returnAltitudeCm = fmaxf(750.0f, rescueState.sensor.currentAltitudeCm + rescueState.intent.initialClimbCm);
    } else {
        // Set return altitude according to user configuration
        switch (gpsRescueConfig()->altitudeMode) {
            case GPS_RESCUE_ALT_MODE_FIXED:
                rescueState.intent.returnAltitudeCm = gpsRescueConfig()->returnAltitudeM * 100.0f;
                break;
            case GPS_RESCUE_ALT_MODE_CURRENT:
                // Climb above current altitude, but always return at least initial height above takeoff point
                rescueState.intent.returnAltitudeCm = fmaxf(rescueState.intent.initialClimbCm, rescueState.sensor.currentAltitudeCm + rescueState.intent.initialClimbCm);
                break;
            case GPS_RESCUE_ALT_MODE_MAX:
            default:
                rescueState.intent.returnAltitudeCm = rescueState.intent.maxAltitudeCm + rescueState.intent.initialClimbCm;
                break;
        }
    }
    rescueState.intent.targetAltitudeCm = rescueState.sensor.currentAltitudeCm;  // Initial target altitude is current filtered altitude

    rescueState.sensor.errorAngleDeg = 0.0f;       // Prevent yaw adjustments
    rescueYawRate = 0.0f;                          // No yaw until climb is complete
    rescueState.intent.targetVelocityCmS = 0.0f;   // Zero initial velocity
    clearTargetStep();
    rescueState.intent.targetAltitudeVelCmS = 0.0f;
    rescueState.sensor.velocityCmS = 0.0f;
    rescueState.intent.yawAttenuator = 0.0f;       // For a smooth start to the yaw
    rescueState.intent.xyAttenuator = 0.0f;        // For a slower start to gaining velocity

    resetAltitudeControl(); // Initialise altitude in autopilot multirotor
    resetPositionControl(TASK_GPS_RESCUE_RATE_HZ); // Initialise position control in autopilot multirotor

}

static void checkGPSRescueIsAvailable(void)
{
rescueState.isAvailable = STATE(GPS_FIX_HOME) && rescueState.sensor.gpsHealthy && rescueState.sensor.isHeadingOK && isAltitudeAvailable() && rescueState.sensor.positionXYAvailable;
        // Flash "RESCUE N/A" in the OSD if false
        // Note that GPS_FIX_HOME is set at arm time, and requires a 3D fix and minSats at that time
}


void gpsRescueUpdate(void) // called from core.c at TASK_GPS_RESCUE_RATE_HZ
{
    if (!FLIGHT_MODE(GPS_RESCUE_MODE)) {
        rescueStop(); // sets phase to RESCUE_IDLE
    } else if (FLIGHT_MODE(GPS_RESCUE_MODE) && rescueState.phase == RESCUE_IDLE) {
        rescueStart(); // sets phase to RESCUE_INITIALIZE
    }
    sensorUpdate(); // get latest velocity to home and Altitude data, set some defaults
    checkGPSRescueIsAvailable();
    performSanityChecks();

    static bool returnAltitudeLow = true;

    switch (rescueState.phase) {
    case RESCUE_IDLE:
        updateMaxAltitude();
        break;
        
    case RESCUE_INITIALIZE:
        returnAltitudeLow = getAltitudeCm() < rescueState.intent.returnAltitudeCm;
        if (!STATE(GPS_FIX_HOME)) {
            rescueState.failure = RESCUE_NO_HOME_POINT;
        } else {
            if (rescueState.sensor.distanceToHomeCm < GPS_RESCUE_ACCEPT_RADIUS && isBelowLandingAltitude()) {
                rescueState.phase = RESCUE_DO_NOTHING;
            } else {
                initRescueValues(); // fix the target location
                returnAltitudeLow = rescueState.sensor.currentAltitudeCm < rescueState.intent.returnAltitudeCm;
                rescueState.phase = RESCUE_ATTAIN_ALT;
            }
        }
        break;

    case RESCUE_ATTAIN_ALT:
        clearTargetStep(); // don't change the target location
        if (returnAltitudeLow == (rescueState.sensor.currentAltitudeCm < rescueState.intent.returnAltitudeCm)) {
            rescueState.intent.targetAltitudeVelCmS = returnAltitudeLow ? (float)gpsRescueConfig()->ascendRate : -(float)gpsRescueConfig()->descendRate;
        } else {
            // climb target achieved
            rescueState.intent.targetAltitudeCm = rescueState.intent.returnAltitudeCm;
            rescueState.intent.targetAltitudeVelCmS = 0.0f;
            rescueState.intent.secondsFailing = 0;
            if (!rescueState.sensor.isHeadingOK) {
                rescueState.phase = RESCUE_PITCH_FORWARD;
            } else {
            rescueState.phase = RESCUE_ROTATE;
            }
        }
        break;

    case RESCUE_PITCH_FORWARD:
        if (!rescueState.sensor.isHeadingOK) { // sanity check allows 15s for this to be true
            clearTargetStep();
            rescueYawRate = 0.0f;
            rescueState.intent.targetVelocityCmS = 0.0f;
            pitchForwardOverride(true); // instructs autopilot to apply forward pitch angle to recover IMU
        } else {
            pitchForwardOverride(false);
            initPositionHold(); // re-anchor position target at the current location
            rescueState.phase = RESCUE_ROTATE;
            rescueState.intent.secondsFailing = 0;
        }
        break;

    case RESCUE_ROTATE:
        clearTargetStep(); // don't change the target location
        updateYawStartupAttenuator();
        controlYaw();
        if (fabsf(rescueState.sensor.errorAngleDeg) < GPS_RESCUE_ALLOWED_YAW_RANGE) {
            rescueState.phase = RESCUE_FLY_HOME; 
            rescueState.intent.targetVelocityCmS = gpsRescueConfig()->groundSpeedCmS;
            rescueState.intent.secondsFailing = 0;
        }
        break;

    case RESCUE_FLY_HOME:
    updateVelocityStartupAttenuator();
        calculateTargetStep();
        controlYaw();
        if (rescueState.sensor.distanceToHomeCm < rescueState.intent.descentDistanceCm) {
            rescueState.phase = RESCUE_DESCENT; // enter descend phase
            initDescent();
            rescueState.intent.secondsFailing = 0; // reset sanity timer for descent
        }
        break;

    case RESCUE_DESCENT:
        updateVelocityStartupAttenuator();
        descend(); // sets a negstive targetAltitudeVelocity
        calculateTargetStep();
        if (isBelowLandingAltitude()) {
            // enter landing mode once below landing altitude
            rescueState.phase = RESCUE_LANDING;
            rescueState.intent.secondsFailing = 0; // reset sanity timer for landing
        }
        break;

    case RESCUE_LANDING:
        descend();
        clearTargetStep();
        // same as Descent mode
        break;

    case RESCUE_EMERG_DESCENT:
        descend();
        vector2Zero(&rescueState.intent.stepEF);
        moveTargetLocation(&rescueState.intent.stepEF, TASK_GPS_RESCUE_RATE_HZ, true); // stop position control

        break;

    case RESCUE_DO_NOTHING:
    clearTargetStep();
    rescueState.intent.targetAltitudeVelCmS = 0.0f;
    rescueYawRate = 0.0f;
    // altitude control with no change in height or heading,  position control at zero target velocity

        break;

    default:
        break;
    }

    DEBUG_SET(DEBUG_GPS_RESCUE_VELOCITY, 1, rescueState.phase);
    DEBUG_SET(DEBUG_ATTITUDE,            5, rescueState.phase);
    DEBUG_SET(DEBUG_ATTITUDE,            6, lrintf(rescueState.intent.targetVelocityCmS));
    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 3, lrintf(rescueState.intent.targetAltitudeCm));
    DEBUG_SET(DEBUG_RTH,                 7, lrintf(rescueState.intent.targetVelocityCmS));
    DEBUG_SET(DEBUG_RTH, 0, lrintf(rescueState.sensor.velocityCmS / 10.0f));



    // Autopilot control of altitude is always active when rescue mode is engaged
    if (rescueState.phase > RESCUE_INITIALIZE) {
        rescueState.intent.targetAltitudeCm += rescueState.intent.targetAltitudeVelCmS * gpsRescueTaskIntervalSeconds;
        controlAltitude();
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

bool gpsRescueIsHeadingOK(void)
{
    return rescueState.sensor.isHeadingOK; // when false, flashes `HEADING N/A` until IMU is oriented
}

bool gpsRescueIsOK(void)
{
    return rescueState.isOK; // when false, flashes `RESCUE FAIL` in OSD when failure time exceeds 5s or when in a failure mode
}

#endif // USE_GPS_RESCUE
#endif // !USE_WING