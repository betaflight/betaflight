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

#include "fc/config.h"
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
    RESCUE_CROSSTRACK,
    RESCUE_LANDING_APPROACH,
    RESCUE_LANDING,
    RESCUE_ABORT,
    RESCUE_COMPLETE
} rescuePhase_e;

typedef enum {
    RESCUE_HEALTHY,
    RESCUE_FLYAWAY,
    RESCUE_GPSLOST,
    RESCUE_LOWSATS,
    RESCUE_CRASH_FLIP_DETECTED,
    RESCUE_STALLED,
    RESCUE_TOO_CLOSE
} rescueFailureState_e;

typedef struct {
    int32_t targetAltitudeCm;
    int32_t targetGroundspeed;
    uint8_t minAngleDeg;
    uint8_t maxAngleDeg;
    bool crosstrack;
} rescueIntent_s;

typedef struct {
    int32_t maxAltitudeCm;
    int32_t currentAltitudeCm;
    uint16_t distanceToHomeM;
    uint16_t maxDistanceToHomeM;
    int16_t directionToHome;
    uint16_t groundSpeed;
    uint8_t numSat;
    float zVelocity; // Up/down movement in cm/s
    float zVelocityAvg; // Up/down average in cm/s
    float accMagnitude;
    float accMagnitudeAvg;
    bool healthy;
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
    bool isFailsafe;
    bool isAvailable;
} rescueState_s;

#define GPS_RESCUE_MAX_YAW_RATE         180 // deg/sec max yaw rate
#define GPS_RESCUE_RATE_SCALE_DEGREES    45 // Scale the commanded yaw rate when the error is less then this angle
#define GPS_RESCUE_SLOWDOWN_DISTANCE_M  200 // distance from home to start decreasing speed

PG_REGISTER_WITH_RESET_TEMPLATE(gpsRescueConfig_t, gpsRescueConfig, PG_GPS_RESCUE, 1);

PG_RESET_TEMPLATE(gpsRescueConfig_t, gpsRescueConfig,
    .angle = 32,
    .initialAltitudeM = 50,
    .descentDistanceM = 200,
    .rescueGroundspeed = 2000,
    .throttleP = 150,
    .throttleI = 20,
    .throttleD = 50,
    .velP = 80,
    .velI = 20,
    .velD = 15,
    .yawP = 40,
    .throttleMin = 1200,
    .throttleMax = 1600,
    .throttleHover = 1280,
    .sanityChecks = RESCUE_SANITY_ON,
    .minSats = 8,
    .minRescueDth = 100,
    .allowArmingWithoutFix = false,
    .useMag = true,
    .targetLandingAltitudeM = 5,
    .targetLandingDistanceM = 10,
);

static uint16_t rescueThrottle;
static float    rescueYaw;

int32_t       gpsRescueAngle[ANGLE_INDEX_COUNT] = { 0, 0 };
uint16_t      hoverThrottle = 0;
float         averageThrottle = 0.0;
float         altitudeError = 0.0;
uint32_t      throttleSamples = 0;
bool          magForceDisable = false;

static bool newGPSData = false;

rescueState_s rescueState;

/*
 If we have new GPS data, update home heading
 if possible and applicable.
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

// Things that need to run regardless of GPS rescue mode being enabled or not
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

    gpsRescueAngle[AI_PITCH] = 0;
    gpsRescueAngle[AI_ROLL] = 0;

    // Store the max altitude we see not during RTH so we know our fly-back minimum alt
    rescueState.sensor.maxAltitudeCm = MAX(rescueState.sensor.currentAltitudeCm, rescueState.sensor.maxAltitudeCm);
    // Store the max distance to home during normal flight so we know if a flyaway is happening
    rescueState.sensor.maxDistanceToHomeM = MAX(rescueState.sensor.distanceToHomeM, rescueState.sensor.maxDistanceToHomeM);

    rescueThrottle = rcCommand[THROTTLE];

    //to do: have a default value for hoverThrottle

    // FIXME: GPS Rescue throttle handling should take into account min_check as the
    // active throttle is from min_check through PWM_RANGE_MAX. Currently adjusting for this
    // in gpsRescueGetThrottle() but it would be better handled here.

    const float ct = getCosTiltAngle();
    if (ct > 0.5 && ct < 0.96 && throttleSamples < 1E6 && rescueThrottle > 1070) { //5 to 45 degrees tilt
        //TO DO: only sample when acceleration is low
        uint16_t adjustedThrottle = 1000 + (rescueThrottle - PWM_RANGE_MIN) * ct;
        if (throttleSamples == 0) {
            averageThrottle = adjustedThrottle;
        } else {
            averageThrottle += (adjustedThrottle - averageThrottle) / (throttleSamples + 1);
        }
        hoverThrottle = lrintf(averageThrottle);
        throttleSamples++;
    }
}

// Very similar to maghold function on betaflight/cleanflight
static void setBearing(int16_t desiredHeading)
{
    float errorAngle = (attitude.values.yaw / 10.0f) - desiredHeading;

    // Determine the most efficient direction to rotate
    if (errorAngle <= -180) {
        errorAngle += 360;
    } else if (errorAngle > 180) {
        errorAngle -= 360;
    }

    errorAngle *= -GET_DIRECTION(rcControlsConfig()->yaw_control_reversed);
        
    // Calculate a desired yaw rate based on a maximum limit beyond
    // an error window and then scale the requested rate down inside
    // the window as error approaches 0.
    rescueYaw = -constrainf(errorAngle / GPS_RESCUE_RATE_SCALE_DEGREES * GPS_RESCUE_MAX_YAW_RATE, -GPS_RESCUE_MAX_YAW_RATE, GPS_RESCUE_MAX_YAW_RATE);
}

static void rescueAttainPosition()
{
    // Speed and altitude controller internal variables
    static float previousSpeedError = 0;
    static int16_t speedIntegral = 0;
    static float previousAltitudeError = 0;
    static int16_t altitudeIntegral = 0;

    if (rescueState.phase == RESCUE_INITIALIZE) {
        // Initialize internal variables each time GPS Rescue is started
        previousSpeedError = 0;
        speedIntegral = 0;
        previousAltitudeError = 0;
        altitudeIntegral = 0;
    }

    // Point to home if that is in our intent
    if (rescueState.intent.crosstrack) {
        setBearing(rescueState.sensor.directionToHome);
    }

    DEBUG_SET(DEBUG_RTH, 3, rescueState.failure); //Failure can change with no new GPS Data

    if (!newGPSData) {
        return;
    }

    /**
        Speed controller
    */
    const int16_t speedError = (rescueState.intent.targetGroundspeed - rescueState.sensor.groundSpeed) / 100;
    const int16_t speedDerivative = speedError - previousSpeedError;

    speedIntegral = constrain(speedIntegral + speedError, -100, 100);

    previousSpeedError = speedError;

    const int16_t angleAdjustment =  gpsRescueConfig()->velP * speedError + (gpsRescueConfig()->velI * speedIntegral) / 100 +  gpsRescueConfig()->velD * speedDerivative;

    gpsRescueAngle[AI_PITCH] = constrain(gpsRescueAngle[AI_PITCH] + MIN(angleAdjustment, 80), rescueState.intent.minAngleDeg * 100, rescueState.intent.maxAngleDeg * 100);

    const float ct = cos(DECIDEGREES_TO_RADIANS(gpsRescueAngle[AI_PITCH] / 10));

    /**
        Altitude controller
    */
    const int16_t altitudeError = (rescueState.intent.targetAltitudeCm - rescueState.sensor.currentAltitudeCm) / 100; // Error in meters
    const int16_t altitudeDerivative = altitudeError - previousAltitudeError;

    // Only allow integral windup within +-15m absolute altitude error
    if (ABS(altitudeError) < 25) {
        altitudeIntegral = constrain(altitudeIntegral + altitudeError, -250, 250);
    } else {
        altitudeIntegral = 0;
    }

    previousAltitudeError = altitudeError;

    const int16_t altitudeAdjustment = (gpsRescueConfig()->throttleP * altitudeError + (gpsRescueConfig()->throttleI * altitudeIntegral) / 10 + gpsRescueConfig()->throttleD * altitudeDerivative) / ct / 20;
    const int16_t hoverAdjustment = (hoverThrottle - 1000) / ct;

    rescueThrottle = constrain(1000 + altitudeAdjustment + hoverAdjustment, gpsRescueConfig()->throttleMin, gpsRescueConfig()->throttleMax);

    DEBUG_SET(DEBUG_RTH, 0, rescueThrottle);
    DEBUG_SET(DEBUG_RTH, 1, gpsRescueAngle[AI_PITCH]);
    DEBUG_SET(DEBUG_RTH, 2, altitudeAdjustment);
}

static void performSanityChecks()
{
    static uint32_t previousTimeUs = 0; // Last time Stalled/LowSat was checked
    static int8_t secondsStalled = 0; // Stalled movement detection
    static uint16_t lastDistanceToHomeM = 0; // Fly Away detection
    static int8_t secondsFlyingAway = 0; 
    static int8_t secondsLowSats = 0; // Minimum sat detection

    const uint32_t currentTimeUs = micros();

    if (rescueState.phase == RESCUE_IDLE) {
        rescueState.failure = RESCUE_HEALTHY;
        return;
    } else if (rescueState.phase == RESCUE_INITIALIZE) {
        // Initialize internal variables each time GPS Rescue is started
        previousTimeUs = currentTimeUs;
        secondsStalled = 10; // Start the count at 10 to be less forgiving at the beginning
        lastDistanceToHomeM = rescueState.sensor.distanceToHomeM;
        secondsFlyingAway = 0;
        secondsLowSats = 5;  // Start the count at 5 to be less forgiving at the beginning
        return;
    }

    // Do not abort until each of these items is fully tested
    if (rescueState.failure != RESCUE_HEALTHY) {
        if (gpsRescueConfig()->sanityChecks == RESCUE_SANITY_ON
            || (gpsRescueConfig()->sanityChecks == RESCUE_SANITY_FS_ONLY && rescueState.isFailsafe == true)) {
            rescueState.phase = RESCUE_ABORT;
        }
    }

    // Check if crash recovery mode is active, disarm if so.
    if (crashRecoveryModeActive()) {
        rescueState.failure = RESCUE_CRASH_FLIP_DETECTED;
    }

    // Check if GPS comms are healthy
    if (!rescueState.sensor.healthy) {
        rescueState.failure = RESCUE_GPSLOST;
    }

    //  Things that should run at a low refresh rate (such as flyaway detection, etc)
    //  This runs at ~1hz
    const uint32_t dTime = currentTimeUs - previousTimeUs;
    if (dTime < 1000000) { //1hz
        return;
    }

    previousTimeUs = currentTimeUs;

    if (rescueState.phase == RESCUE_CROSSTRACK) {
        secondsStalled = constrain(secondsStalled + ((rescueState.sensor.groundSpeed < 150) ? 1 : -1), 0, 20);

        if (secondsStalled == 20) {
            rescueState.failure = RESCUE_STALLED;
        }

        secondsFlyingAway = constrain(secondsFlyingAway + ((lastDistanceToHomeM < rescueState.sensor.distanceToHomeM) ? 1 : -1), 0, 10);
        lastDistanceToHomeM = rescueState.sensor.distanceToHomeM;

        if (secondsFlyingAway == 10) {
            //If there is a mag and has not been disabled, we have to assume is healthy and has been used in imu.c
            if (sensors(SENSOR_MAG) && gpsRescueConfig()->useMag && !magForceDisable) {
                //Try again with mag disabled
                magForceDisable = true;
                secondsFlyingAway = 0;
            } else {
                rescueState.failure = RESCUE_FLYAWAY;
            }
        }
    }

    secondsLowSats = constrain(secondsLowSats + ((rescueState.sensor.numSat < gpsRescueConfig()->minSats) ? 1 : -1), 0, 10);

    if (secondsLowSats == 10) {
        rescueState.failure = RESCUE_LOWSATS;
    }
}

static void sensorUpdate()
{
    rescueState.sensor.currentAltitudeCm = getEstimatedAltitudeCm();
    rescueState.sensor.healthy = gpsIsHealthy();

    // Calculate altitude velocity
    static uint32_t previousTimeUs;
    static int32_t previousAltitudeCm;

    const uint32_t currentTimeUs = micros();
    const float dTime = currentTimeUs - previousTimeUs;

    if (newGPSData) { // Calculate velocity at lowest common denominator
        rescueState.sensor.distanceToHomeM = GPS_distanceToHome;
        rescueState.sensor.directionToHome = GPS_directionToHome;
        rescueState.sensor.numSat = gpsSol.numSat;
        rescueState.sensor.groundSpeed = gpsSol.groundSpeed;

        rescueState.sensor.zVelocity = (rescueState.sensor.currentAltitudeCm - previousAltitudeCm) * 1000000.0f / dTime;
        rescueState.sensor.zVelocityAvg = 0.8f * rescueState.sensor.zVelocityAvg + rescueState.sensor.zVelocity * 0.2f;

        rescueState.sensor.accMagnitude = (float) sqrtf(sq(acc.accADC[Z]) + sq(acc.accADC[X]) + sq(acc.accADC[Y])) * acc.dev.acc_1G_rec;
        rescueState.sensor.accMagnitudeAvg = (rescueState.sensor.accMagnitudeAvg * 0.8f) + (rescueState.sensor.accMagnitude * 0.2f);

        previousAltitudeCm = rescueState.sensor.currentAltitudeCm;
        previousTimeUs = currentTimeUs;
    }
}

// This function checks the following conditions to determine if GPS rescue is available:
// 1. sensor healthy - GPS data is being received.
// 2. GPS has a valid fix.
// 3. GPS number of satellites is less than the minimum configured for GPS rescue.
// Note: this function does not take into account the distance from homepoint etc. (gps_rescue_min_dth) and
// is also independent of the gps_rescue_sanity_checks configuration
static bool checkGPSRescueIsAvailable(void)
{
    static uint32_t previousTimeUs = 0; // Last time LowSat was checked
    const uint32_t currentTimeUs = micros();
    static int8_t secondsLowSats = 0; // Minimum sat detection
    static bool lowsats = false;
    static bool noGPSfix = false;
    bool result = true;

    if (!gpsIsHealthy() || !STATE(GPS_FIX_HOME)) {
        return false;
    }

    //  Things that should run at a low refresh rate >> ~1hz
    const uint32_t dTime = currentTimeUs - previousTimeUs;
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

/*
    Determine what phase we are in, determine if all criteria are met to move to the next phase
*/
void updateGPSRescueState(void)
{
    static uint16_t newDescentDistanceM;
    static float_t lineSlope;
    static float_t lineOffsetM;
    static int32_t newSpeed;

    if (!FLIGHT_MODE(GPS_RESCUE_MODE)) {
        rescueStop();
    } else if (FLIGHT_MODE(GPS_RESCUE_MODE) && rescueState.phase == RESCUE_IDLE) {
        rescueStart();
        rescueAttainPosition(); // Initialize
        performSanityChecks(); // Initialize
    }

    rescueState.isFailsafe = failsafeIsActive();

    sensorUpdate();

    rescueState.isAvailable = checkGPSRescueIsAvailable();

    switch (rescueState.phase) {
    case RESCUE_IDLE:
        idleTasks();
        break;
    case RESCUE_INITIALIZE:
        if (hoverThrottle == 0) { //no actual throttle data yet, let's use the default.
            hoverThrottle = gpsRescueConfig()->throttleHover;
        }

        if (!STATE(GPS_FIX_HOME)) {
            setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
            disarm();
        }

        // Minimum distance detection.
        if (rescueState.sensor.distanceToHomeM < gpsRescueConfig()->minRescueDth) {
            rescueState.failure = RESCUE_TOO_CLOSE;
            
            // Never allow rescue mode to engage as a failsafe when too close.
            if (rescueState.isFailsafe) {
                setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
                disarm();
            }
            
            // When not in failsafe mode: leave it up to the sanity check setting.
        }
        
        newSpeed = gpsRescueConfig()->rescueGroundspeed;
        //set new descent distance if actual distance to home is lower 
        if (rescueState.sensor.distanceToHomeM < gpsRescueConfig()->descentDistanceM) {
            newDescentDistanceM = rescueState.sensor.distanceToHomeM - 5;
        } else {
            newDescentDistanceM = gpsRescueConfig()->descentDistanceM;
        }
        
        //Calculate angular coefficient and offset for equation of line from 2 points needed for RESCUE_LANDING_APPROACH
        lineSlope = ((float)gpsRescueConfig()->initialAltitudeM  - gpsRescueConfig()->targetLandingAltitudeM) / (newDescentDistanceM - gpsRescueConfig()->targetLandingDistanceM);
        lineOffsetM = gpsRescueConfig()->initialAltitudeM - lineSlope * newDescentDistanceM;

        rescueState.phase = RESCUE_ATTAIN_ALT;
        FALLTHROUGH;
    case RESCUE_ATTAIN_ALT:
        // Get to a safe altitude at a low velocity ASAP
        if (ABS(rescueState.intent.targetAltitudeCm - rescueState.sensor.currentAltitudeCm) < 1000) {
            rescueState.phase = RESCUE_CROSSTRACK;
        }

        rescueState.intent.targetGroundspeed = 500;
        rescueState.intent.targetAltitudeCm = MAX(gpsRescueConfig()->initialAltitudeM * 100, rescueState.sensor.maxAltitudeCm + 1500);
        rescueState.intent.crosstrack = true;
        rescueState.intent.minAngleDeg = 10;
        rescueState.intent.maxAngleDeg = 15;
        break;
    case RESCUE_CROSSTRACK:
        if (rescueState.sensor.distanceToHomeM <= newDescentDistanceM) {
            rescueState.phase = RESCUE_LANDING_APPROACH;
        }

        // We can assume at this point that we are at or above our RTH height, so we need to try and point to home and tilt while maintaining alt
        // Is our altitude way off?  We should probably kick back to phase RESCUE_ATTAIN_ALT
        rescueState.intent.targetGroundspeed = gpsRescueConfig()->rescueGroundspeed;
        rescueState.intent.targetAltitudeCm = MAX(gpsRescueConfig()->initialAltitudeM * 100, rescueState.sensor.maxAltitudeCm + 1500);
        rescueState.intent.crosstrack = true;
        rescueState.intent.minAngleDeg = 15;
        rescueState.intent.maxAngleDeg = gpsRescueConfig()->angle;
        break;
    case RESCUE_LANDING_APPROACH:
        // We are getting close to home in the XY plane, get Z where it needs to be to move to landing phase
        if (rescueState.sensor.distanceToHomeM <= gpsRescueConfig()->targetLandingDistanceM && rescueState.sensor.currentAltitudeCm <= gpsRescueConfig()->targetLandingAltitudeM * 100) {
            rescueState.phase = RESCUE_LANDING;
        }

        // Only allow new altitude and new speed to be equal or lower than the current values (to prevent parabolic movement on overshoot)
        const int32_t newAlt = (lineSlope * rescueState.sensor.distanceToHomeM + lineOffsetM) * 100;
        
        // Start to decrease proportionally the quad's speed when the distance to home is less or equal than GPS_RESCUE_SLOWDOWN_DISTANCE_M
        if (rescueState.sensor.distanceToHomeM <= GPS_RESCUE_SLOWDOWN_DISTANCE_M) {
            newSpeed = gpsRescueConfig()->rescueGroundspeed * rescueState.sensor.distanceToHomeM / GPS_RESCUE_SLOWDOWN_DISTANCE_M;
        }

        rescueState.intent.targetAltitudeCm = constrain(newAlt, 100, rescueState.intent.targetAltitudeCm);
        rescueState.intent.targetGroundspeed = constrain(newSpeed, 100, rescueState.intent.targetGroundspeed);
        rescueState.intent.crosstrack = true;
        rescueState.intent.minAngleDeg = 10;
        rescueState.intent.maxAngleDeg = gpsRescueConfig()->angle;
        break;
    case RESCUE_LANDING:
        // We have reached the XYZ envelope to be considered at "home".  We need to land gently and check our accelerometer for abnormal data.
        // At this point, do not let the target altitude go up anymore, so if we overshoot, we dont' move in a parabolic trajectory

        // If we are over 150% of average magnitude, just disarm since we're pretty much home
        if (rescueState.sensor.accMagnitude > rescueState.sensor.accMagnitudeAvg * 1.5) {
            setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
            disarm();
            rescueState.phase = RESCUE_COMPLETE;
        }

        rescueState.intent.targetGroundspeed = 0;
        rescueState.intent.targetAltitudeCm = 0;
        rescueState.intent.crosstrack = true;
        rescueState.intent.minAngleDeg = 0;
        rescueState.intent.maxAngleDeg = 15;
        break;
    case RESCUE_COMPLETE:
        rescueStop();
        break;
    case RESCUE_ABORT:
        setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
        disarm();
        rescueStop();
        break;
    default:
        break;
    }

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

bool gpsRescueDisableMag(void)
{
    return ((!gpsRescueConfig()->useMag || magForceDisable) && (rescueState.phase >= RESCUE_INITIALIZE) && (rescueState.phase <= RESCUE_LANDING));
}
#endif

