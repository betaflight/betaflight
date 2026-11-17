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
    float targetAltitudeVelCmS;
    float descentDistanceCm;
    int8_t secondsFailing;
    float landingAltitudeCm;
    float yawAttenuator;
    float targetHeadingDeg;
    float xyVelStartRampGain; // adjusts ramp to constrain acceleration at the start to MAX_XY_ACCELERATION
    float xyVelStartRamp;     // time based, 0..1 groundspeed start attenuator
    float targetSpeedCmS;

    vector2_t targetVelocityEF; // target EF velocity vector towards home
    float initialClimbCm;
} rescueIntent_s;

typedef struct {
    bool gpsHealthy;
    bool isHeadingOK;
    bool newGpsData;
    bool positionXYAvailable;
    float aircraftHeadingDeg;
    float errorAngleDeg;
    float distanceToHomeCm;
    float currentSpeedCmS; // for debugs only
    vector2_t currentVelocityV;
    vector2_t currentPositionV; // relative to arming location, not absolute
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

#define GPS_RESCUE_MAX_YAW_RATE          180   // deg/sec max yaw rate
#define GPS_RESCUE_ALLOWED_YAW_RANGE   30.0f   // deg, yaw error must be less than this to enter fly home phase, and to pitch during descend()
#define GPS_RESCUE_ACCEPT_RADIUS       30.0f   // closer than this distance in cm is considered home
#define MAX_XY_ACCELERATION 500.0f  // 5 m/s²
#define GPS_RESCUE_DESCENT_RATE_MIN 0.6f
#define GPS_RESCUE_DESCENT_RATE_MAX 3.0f
#define GPS_RESCUE_APPROACH_DECEL_CMSS 150.0f

#if !ENABLE_RESCUE_PLAN
static const float gpsRescueTaskIntervalSeconds = HZ_TO_INTERVAL(TASK_GPS_RESCUE_RATE_HZ); // i.e. 0.01s
#endif

void gpsRescueInit(void)
{
    rescueState.intent.initialClimbCm = gpsRescueConfig()->initialClimbM * 100.0f;
    rescueState.intent.descentDistanceCm = gpsRescueConfig()->descentDistanceM * 100.0f;
    rescueState.intent.xyVelStartRampGain = gpsRescueConfig()->groundSpeedCmS > 0 ? MAX_XY_ACCELERATION / gpsRescueConfig()->groundSpeedCmS : 0.0f;
    rescueState.intent.landingAltitudeCm = 100.0f * autopilotConfig()->landingAltitudeM;
    rescueState.isAvailable = true;
    rescueState.sensor.isHeadingOK = true;
    rescueState.isOK = true;
    
}

#if !ENABLE_RESCUE_PLAN
static void rescueStart(void)
{
    rescueState.phase = RESCUE_INITIALIZE;
}

static void rescueStop(void)
{
    rescueState.phase = RESCUE_IDLE;
    pitchForwardOverride(false);
}
#endif // !ENABLE_RESCUE_PLAN
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
    }

    rescueState.sensor.distanceToHomeCm = vector2Norm(&rescueState.sensor.currentPositionV);
    rescueState.sensor.currentSpeedCmS = vector2Norm(&rescueState.sensor.currentVelocityV); // only for debugs

    rescueState.sensor.aircraftHeadingDeg = DECIDEGREES_TO_DEGREES(attitude.values.yaw); // for debugs only
    if (rescueState.sensor.distanceToHomeCm > GPS_RESCUE_ACCEPT_RADIUS) {
        rescueState.intent.targetHeadingDeg = RADIANS_TO_DEGREES(atan2f(
            -rescueState.sensor.currentPositionV.v[EF_EAST],
            -rescueState.sensor.currentPositionV.v[EF_NORTH]));

        if (rescueState.intent.targetHeadingDeg < 0.0f) {
            rescueState.intent.targetHeadingDeg += 360.0f;
        }
    }

    rescueState.sensor.errorAngleDeg =
        rescueState.sensor.aircraftHeadingDeg - rescueState.intent.targetHeadingDeg;
    rescueState.sensor.errorAngleDeg =
        fmodf(rescueState.sensor.errorAngleDeg + 540.0f, 360.0f) - 180.0f;

}

#if !ENABLE_RESCUE_PLAN
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
    if (rescueState.intent.xyVelStartRamp < 1.0f) {
        rescueState.intent.xyVelStartRamp += gpsRescueTaskIntervalSeconds * rescueState.intent.xyVelStartRampGain;
        rescueState.intent.xyVelStartRamp = fminf(rescueState.intent.xyVelStartRamp, 1.0f);
    }
}

static void controlYaw(void)
{
    const float targetHeadingDeg =
        rescueState.sensor.aircraftHeadingDeg -
        rescueState.sensor.errorAngleDeg * rescueState.intent.yawAttenuator;

    autopilotSetYawTarget(targetHeadingDeg);
}
static void controlVelocity(void)
{
    if (rescueState.sensor.distanceToHomeCm > 1.0f) { // 1.0f to avoid div by zero
        const float scale = rescueState.intent.targetSpeedCmS / rescueState.sensor.distanceToHomeCm;
        vector2Scale(&rescueState.intent.targetVelocityEF, &rescueState.sensor.currentPositionV, -scale);
    } else {
        vector2Zero(&rescueState.intent.targetVelocityEF);
    }

    setTargetVelocity(&rescueState.intent.targetVelocityEF, false);
}

static void clearTargetVelocity(void)
{
    rescueState.intent.targetSpeedCmS = 0.0f;
    vector2Zero(&rescueState.intent.targetVelocityEF);
    setTargetVelocity(&rescueState.intent.targetVelocityEF, false);
}

static void controlAltitude(void)
{
    const float vzLim = 3.0f * fmaxf((float)gpsRescueConfig()->ascendRate, (float)gpsRescueConfig()->descendRate);
    altitudeControl(rescueState.intent.targetAltitudeCm, HZ_TO_INTERVAL_US(TASK_GPS_RESCUE_RATE_HZ), rescueState.intent.targetAltitudeVelCmS, vzLim);
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
        if (rescueState.phase != RESCUE_EMERG_DESCENT) {
            rescueState.phase = RESCUE_EMERG_DESCENT;
            rescueState.intent.secondsFailing = 0;
        }
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
    static int8_t secondsTooSlow = 0;
    const timeUs_t currentTimeUs = micros();

    static timeUs_t lastSanityCheck = 0;
    static bool oneSecondPassed = true;
    if (!oneSecondPassed && cmpTimeUs(currentTimeUs, lastSanityCheck) >= 1000000) {
        oneSecondPassed = true;
    }

    if (rescueState.phase == RESCUE_IDLE) {
        rescueState.failure = RESCUE_HEALTHY;
        return;
    }

    DEBUG_SET(DEBUG_RTH, 2, rescueState.phase);                 //!< Rescue Phase [enum:rescuePhase_e]
    DEBUG_SET(DEBUG_RTH, 3, rescueState.failure);               //!< Rescue Failure Code [enum:rescueFailureState_e]
    DEBUG_SET(DEBUG_RTH, 4, rescueState.intent.secondsFailing); //!< Seconds Failing [unit:s]
    DEBUG_SET(DEBUG_RTH, 5, secondsLowSats);                    //!< Seconds With Low Satellite Count [unit:s]

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
        secondsTooSlow = 0;
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
            break;
        case RESCUE_SANITY_FS_ONLY:
            if (hardFailsafe) {
                rescueEmergDescent();
            }
            break;
        default:
            if ((!rescueState.isAvailable) && hardFailsafe) {
                rescueEmergDescent();
            }
            break;
        }
        if (rescueState.phase == RESCUE_IDLE) {
            return;
        }
    }

    DEBUG_SET(DEBUG_RTH, 2, rescueState.phase);  //!< Rescue Phase [enum:rescuePhase_e]

    if (!oneSecondPassed) {
        return;
    }

    lastSanityCheck = currentTimeUs;
    oneSecondPassed = false;

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
        if (++rescueState.intent.secondsFailing >= 10) {
            if (!descendIfNeeded()) {
                rescueState.phase = RESCUE_FLY_HOME;
            }
            rescueState.intent.secondsFailing = 0;
        }
        break;

    case RESCUE_FLY_HOME:
        rescueState.intent.secondsFailing += (velocityToHomeCmS < 0.2f * rescueState.intent.targetSpeedCmS) ? 1 : -1;
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
        // Monitor horizontal progress throughout descent, as for fly home
        secondsTooSlow += (velocityToHomeCmS < 0.2f * rescueState.intent.targetSpeedCmS) ? 1 : -1;
        secondsTooSlow = MAX(0, secondsTooSlow);
        if (secondsTooSlow >= 15) {
            rescueState.phase = RESCUE_LANDING;
            secondsTooSlow = 0;
        }
        // Monitor altitude progress
        if (fabsf(targetAltitudeChange) >= 1.0f) {
            rescueState.intent.secondsFailing += altitudeControlError > 0.5f ? -1 : 1;
            rescueState.intent.secondsFailing = MAX(0, rescueState.intent.secondsFailing);
            if (rescueState.intent.secondsFailing >= 15) {
                rescueState.phase = RESCUE_LANDING;
                rescueState.intent.secondsFailing = 0;
            }
        }
        break;
    case RESCUE_LANDING:
    case RESCUE_EMERG_DESCENT:
        if (rescueState.phase == RESCUE_LANDING || isBelowLandingAltitude() || hardFailsafe) {
            rescueState.intent.secondsFailing += altitudeControlError > 0.5f ? -1 : 1;
            rescueState.intent.secondsFailing = MAX(0, rescueState.intent.secondsFailing);
            if (rescueState.intent.secondsFailing >= 15) {
                rescueDisarmNow();
            }
        } else {
            rescueState.intent.secondsFailing = 0;
        }
        break;
    default:
        break;
    }
}

static float setDescentRateCmS(float altitudeCm)
{
    const float descendRateCmS = gpsRescueConfig()->descendRate;

    return scaleRangef(
        constrainf(altitudeCm, 1000.0f, 5000.0f),
        1000.0f, 5000.0f,
        GPS_RESCUE_DESCENT_RATE_MIN * descendRateCmS,
        GPS_RESCUE_DESCENT_RATE_MAX * descendRateCmS); // same method as rescue flightplan mode
}

static void updateAltitudeControl(void)
{
    rescueState.intent.targetAltitudeCm +=
        rescueState.intent.targetAltitudeVelCmS * gpsRescueTaskIntervalSeconds;
    controlAltitude();
}

static void descend(void)
{
    rescueState.intent.targetAltitudeVelCmS =
        -setDescentRateCmS(rescueState.intent.targetAltitudeCm);
}

static void land(void)
{
    const float descendRateCmS = gpsRescueConfig()->descendRate;
    const landingProfile_t profile = {
        .touchdownRateCmS = GPS_RESCUE_DESCENT_RATE_MIN * descendRateCmS,
        .maxRateCmS = GPS_RESCUE_DESCENT_RATE_MAX * descendRateCmS,
    };

    autopilotLand(&profile, rescueState.intent.targetAltitudeCm,
        HZ_TO_INTERVAL_US(TASK_GPS_RESCUE_RATE_HZ));

    rescueState.intent.targetAltitudeCm = autopilotLandingTargetAltitudeCm();
}

static bool descendIfNeeded(void)
{
    if (rescueState.sensor.distanceToHomeCm < rescueState.intent.descentDistanceCm) {
        rescueState.phase = RESCUE_DESCENT;
        rescueState.intent.secondsFailing = 0;
        return true;
    }
    return false;
}

static void updateTargetSpeed(void)
{
const float distanceCm = rescueState.sensor.distanceToHomeCm;
const float decelCmSS = GPS_RESCUE_APPROACH_DECEL_CMSS;
const float kneeCm = decelCmSS;
const float approachSpeedCmS = distanceCm <= kneeCm
    ? distanceCm
    : sqrtf(2.0f * decelCmSS * distanceCm - decelCmSS * kneeCm);

rescueState.intent.targetSpeedCmS = fminf(gpsRescueConfig()->groundSpeedCmS, approachSpeedCmS)
    * rescueState.intent.xyVelStartRamp;
}

void initRescueValues(void)
{
    if (rescueState.sensor.distanceToHomeCm < gpsRescueConfig()->minStartDistM * 100.0f) {
        // 7.5m high for close-range headroom
        rescueState.intent.returnAltitudeCm = fmaxf(750.0f, rescueState.sensor.currentAltitudeCm + rescueState.intent.initialClimbCm);
    } else {
        switch (gpsRescueConfig()->altitudeMode) {
            case GPS_RESCUE_ALT_MODE_FIXED:
                rescueState.intent.returnAltitudeCm = gpsRescueConfig()->returnAltitudeM * 100.0f;
                break;
            case GPS_RESCUE_ALT_MODE_CURRENT:
                rescueState.intent.returnAltitudeCm = fmaxf(rescueState.intent.initialClimbCm, rescueState.sensor.currentAltitudeCm + rescueState.intent.initialClimbCm);
                break;
            case GPS_RESCUE_ALT_MODE_MAX:
            default:
                rescueState.intent.returnAltitudeCm = rescueState.intent.maxAltitudeCm + rescueState.intent.initialClimbCm;
                break;
        }

        // Return altitude must be at least 1m above landing altitude
        rescueState.intent.returnAltitudeCm = fmaxf( rescueState.intent.returnAltitudeCm, rescueState.intent.landingAltitudeCm + 100.0f);
    }
    rescueState.intent.targetAltitudeCm = rescueState.sensor.currentAltitudeCm;  // Initial target altitude is current filtered altitude
    autopilotSetYawTarget(DECIDEGREES_TO_DEGREES(attitude.values.yaw));
    rescueState.intent.targetAltitudeVelCmS = 0.0f;
    rescueState.intent.targetSpeedCmS = 0.0f;

    rescueState.intent.yawAttenuator = 0.0f; // For a smooth start to the yaw
    rescueState.intent.xyVelStartRamp = 0.0f; // For a smooth start to gaining velocity
    clearTargetVelocity();
    resetAltitudeControl(); // Initialise altitude in autopilot multirotor
}
#endif // !ENABLE_RESCUE_PLAN

static void checkGPSRescueIsAvailable(void)
{
    rescueState.isAvailable = STATE(GPS_FIX_HOME) && rescueState.sensor.gpsHealthy && rescueState.sensor.isHeadingOK && isAltitudeAvailable() && rescueState.sensor.positionXYAvailable;
    // Flash "RESCUE N/A" in the OSD if false
    // Note that GPS_FIX_HOME is set at arm time, and requires a 3D fix and minSats at that time
}

#if !ENABLE_RESCUE_PLAN
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
        if (!STATE(GPS_FIX_HOME)) {
            rescueState.failure = RESCUE_NO_HOME_POINT;
        } else {
            if (rescueState.sensor.distanceToHomeCm < GPS_RESCUE_ACCEPT_RADIUS
                && isBelowLandingAltitude()) {
                rescueState.phase = RESCUE_DO_NOTHING;
            } else {
                initRescueValues();
                returnAltitudeLow = rescueState.sensor.currentAltitudeCm
                    < rescueState.intent.returnAltitudeCm;
                rescueState.phase = RESCUE_ATTAIN_ALT;
            }
            updateAltitudeControl();
        }
        break;

    case RESCUE_ATTAIN_ALT:
        clearTargetVelocity();
        if (returnAltitudeLow == (rescueState.sensor.currentAltitudeCm < rescueState.intent.returnAltitudeCm)) {
            rescueState.intent.targetAltitudeVelCmS = returnAltitudeLow
                ? (float)gpsRescueConfig()->ascendRate
                : -(float)gpsRescueConfig()->descendRate;
        } else {
            rescueState.intent.targetAltitudeCm = rescueState.intent.returnAltitudeCm;
            rescueState.intent.targetAltitudeVelCmS = 0.0f;
            rescueState.intent.secondsFailing = 0;
            if (!rescueState.sensor.isHeadingOK) {
                rescueState.phase = RESCUE_PITCH_FORWARD;
            } else {
                rescueState.phase = RESCUE_ROTATE;
            }
        }
        updateAltitudeControl();
        break;

    case RESCUE_PITCH_FORWARD:
        if (!rescueState.sensor.isHeadingOK) {
            clearTargetVelocity();
            pitchForwardOverride(true);
        } else {
            pitchForwardOverride(false);
            initPositionHold();
            rescueState.phase = RESCUE_ROTATE;
            rescueState.intent.secondsFailing = 0;
        }
        updateAltitudeControl();
        break;

    case RESCUE_ROTATE:
        clearTargetVelocity(); // don't change the target location
        updateYawStartupAttenuator();
        controlYaw();
        if (fabsf(rescueState.sensor.errorAngleDeg) < GPS_RESCUE_ALLOWED_YAW_RANGE) {
            if (!descendIfNeeded()) {
                rescueState.phase = RESCUE_FLY_HOME;
            }
            rescueState.intent.secondsFailing = 0;
        }
        updateAltitudeControl();
        break;

    case RESCUE_FLY_HOME:
        updateVelocityStartupAttenuator();
        if (!descendIfNeeded()) {
            updateTargetSpeed();
            controlVelocity();
            controlYaw();
        }
        updateAltitudeControl();
        break;

    case RESCUE_DESCENT:
        updateVelocityStartupAttenuator();
        updateTargetSpeed();
        controlVelocity();

        if (rescueState.intent.targetAltitudeCm > rescueState.intent.landingAltitudeCm) {
            descend();
            updateAltitudeControl();
        } else if (rescueState.sensor.distanceToHomeCm < GPS_RESCUE_ACCEPT_RADIUS) {
            rescueState.phase = RESCUE_LANDING;
            rescueState.intent.secondsFailing = 0;
            land();
        } else {
            rescueState.intent.targetAltitudeVelCmS = 0.0f;
            updateAltitudeControl();
        }
        break;

    case RESCUE_LANDING:
        clearTargetVelocity();
        land();
        break;

    case RESCUE_EMERG_DESCENT:
        vector2Zero(&rescueState.intent.targetVelocityEF);
        setTargetVelocity(&rescueState.intent.targetVelocityEF, true); // force abort position control
        land();
        break;

    case RESCUE_DO_NOTHING:
        clearTargetVelocity();
        rescueState.intent.targetAltitudeVelCmS = 0.0f;
        updateAltitudeControl(); // altitude control with no change in height or heading, position control at zero target velocity
        break;

    default:
        break;
    }

    DEBUG_SET(DEBUG_GPS_RESCUE_VELOCITY, 4, rescueState.phase);  //!< Rescue Phase [enum:rescuePhase_e]
    DEBUG_SET(DEBUG_ATTITUDE, 5, rescueState.phase);             //!< Rescue Phase [enum:rescuePhase_e]
    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 7, rescueState.phase);  //!< Rescue Phase [enum:rescuePhase_e]

}
#else // ENABLE_RESCUE_PLAN
// GPS rescue is flown as an autopilot mission (the rescue plan in
// flight_plan_nav): the legacy controller above is retired. This task only
// maintains the sensor/availability state and the running max altitude that the
// rescue-plan builder reads for its return-altitude modes.
void gpsRescueUpdate(void)
{
    sensorUpdate();
    updateMaxAltitude();
    checkGPSRescueIsAvailable();
}
#endif // !ENABLE_RESCUE_PLAN

float gpsRescueGetMaxAltitudeCm(void)
{
    return rescueState.intent.maxAltitudeCm; // running max since arming, tracked while the rescue task idles
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
