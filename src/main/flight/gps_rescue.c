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

#include "drivers/time.h"

#include "io/gps.h"

#include "fc/config.h"
#include "fc/fc_core.h"
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

#define GPS_RESCUE_MAX_YAW_RATE       360  // deg/sec max yaw rate
#define GPS_RESCUE_RATE_SCALE_DEGREES 45   // Scale the commanded yaw rate when the error is less then this angle

PG_REGISTER_WITH_RESET_TEMPLATE(gpsRescueConfig_t, gpsRescueConfig, PG_GPS_RESCUE, 0);

PG_RESET_TEMPLATE(gpsRescueConfig_t, gpsRescueConfig,
    .angle = 32,
    .initialAltitude = 50,
    .descentDistance = 200,
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
    .sanityChecks = 0,
    .minSats = 8
);

static uint16_t rescueThrottle;
static float    rescueYaw;

int32_t       gpsRescueAngle[ANGLE_INDEX_COUNT] = { 0, 0 };
uint16_t      hoverThrottle = 0;
float         averageThrottle = 0.0;
float         altitudeError = 0.0;
uint32_t      throttleSamples = 0;

static bool newGPSData = false;

rescueState_s rescueState;

/*
 If we have new GPS data, update home heading
 if possible and applicable.
*/
void rescueNewGpsData(void)
{
    if (!ARMING_FLAG(ARMED))
	GPS_reset_home_position();
    newGPSData = true;
}

/*
    Determine what phase we are in, determine if all criteria are met to move to the next phase
*/
void updateGPSRescueState(void)
{
    if (!FLIGHT_MODE(GPS_RESCUE_MODE)) {
        rescueStop();
    } else if (FLIGHT_MODE(GPS_RESCUE_MODE) && rescueState.phase == RESCUE_IDLE) {
        rescueStart();
    }

    rescueState.isFailsafe = failsafeIsActive();

    sensorUpdate();

    switch (rescueState.phase) {
    case RESCUE_IDLE:
        idleTasks();
        break;
    case RESCUE_INITIALIZE:
        if (hoverThrottle == 0) { //no actual throttle data yet, let's use the default.
            hoverThrottle = gpsRescueConfig()->throttleHover;
        }

        rescueState.phase = RESCUE_ATTAIN_ALT;
        FALLTHROUGH;
    case RESCUE_ATTAIN_ALT:
        // Get to a safe altitude at a low velocity ASAP
        if (ABS(rescueState.intent.targetAltitude - rescueState.sensor.currentAltitude) < 1000) {
            rescueState.phase = RESCUE_CROSSTRACK;
        }

        rescueState.intent.targetGroundspeed = 500;
        rescueState.intent.targetAltitude = MAX(gpsRescueConfig()->initialAltitude * 100, rescueState.sensor.maxAltitude + 1500);
        rescueState.intent.crosstrack = true;
        rescueState.intent.minAngleDeg = 10;
        rescueState.intent.maxAngleDeg = 15;
        break;
    case RESCUE_CROSSTRACK:
        if (rescueState.sensor.distanceToHome < gpsRescueConfig()->descentDistance) {
            rescueState.phase = RESCUE_LANDING_APPROACH;
        }

        // We can assume at this point that we are at or above our RTH height, so we need to try and point to home and tilt while maintaining alt
        // Is our altitude way off?  We should probably kick back to phase RESCUE_ATTAIN_ALT
        rescueState.intent.targetGroundspeed = gpsRescueConfig()->rescueGroundspeed;
        rescueState.intent.targetAltitude = MAX(gpsRescueConfig()->initialAltitude * 100, rescueState.sensor.maxAltitude + 1500);
        rescueState.intent.crosstrack = true;
        rescueState.intent.minAngleDeg = 15;
        rescueState.intent.maxAngleDeg = gpsRescueConfig()->angle;
        break;
    case RESCUE_LANDING_APPROACH:
        // We are getting close to home in the XY plane, get Z where it needs to be to move to landing phase
        if (rescueState.sensor.distanceToHome < 10 && rescueState.sensor.currentAltitude <= 1000) {
            rescueState.phase = RESCUE_LANDING;
        }

        // Only allow new altitude and new speed to be equal or lower than the current values (to prevent parabolic movement on overshoot)
        int32_t newAlt = gpsRescueConfig()->initialAltitude * 100  * rescueState.sensor.distanceToHome / gpsRescueConfig()->descentDistance;
        int32_t newSpeed = gpsRescueConfig()->rescueGroundspeed * rescueState.sensor.distanceToHome / gpsRescueConfig()->descentDistance;

        rescueState.intent.targetAltitude = constrain(newAlt, 100, rescueState.intent.targetAltitude);
        rescueState.intent.targetGroundspeed = constrain(newSpeed, 100, rescueState.intent.targetGroundspeed);
        rescueState.intent.crosstrack = true;
        rescueState.intent.minAngleDeg = 10;
        rescueState.intent.maxAngleDeg = 20;
        break;
    case RESCUE_LANDING:
        // We have reached the XYZ envelope to be considered at "home".  We need to land gently and check our accelerometer for abnormal data.
        // At this point, do not let the target altitude go up anymore, so if we overshoot, we dont' move in a parabolic trajectory

        // If we are over 120% of average magnitude, just disarm since we're pretty much home
        if (rescueState.sensor.accMagnitude > rescueState.sensor.accMagnitudeAvg * 1.5) {
            setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
            disarm();
            rescueState.phase = RESCUE_COMPLETE;
        }

        rescueState.intent.targetGroundspeed = 0;
        rescueState.intent.targetAltitude = 0;
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

void sensorUpdate()
{
    rescueState.sensor.currentAltitude = getEstimatedAltitude();

    // Calculate altitude velocity
    static uint32_t previousTimeUs;
    static int32_t previousAltitude;

    const uint32_t currentTimeUs = micros();
    const float dTime = currentTimeUs - previousTimeUs;

    if (newGPSData) { // Calculate velocity at lowest common denominator
        rescueState.sensor.distanceToHome = GPS_distanceToHome;
        rescueState.sensor.directionToHome = GPS_directionToHome;
        rescueState.sensor.numSat = gpsSol.numSat;
        rescueState.sensor.groundSpeed = gpsSol.groundSpeed;

        rescueState.sensor.zVelocity = (rescueState.sensor.currentAltitude - previousAltitude) * 1000000.0f / dTime;
        rescueState.sensor.zVelocityAvg = 0.8f * rescueState.sensor.zVelocityAvg + rescueState.sensor.zVelocity * 0.2f;

        rescueState.sensor.accMagnitude = (float) sqrt(sq(acc.accADC[Z]) + sq(acc.accADC[X]) + sq(acc.accADC[Y]) / sq(acc.dev.acc_1G));
        rescueState.sensor.accMagnitudeAvg = (rescueState.sensor.accMagnitudeAvg * 0.8f) + (rescueState.sensor.accMagnitude * 0.2f);

        previousAltitude = rescueState.sensor.currentAltitude;
        previousTimeUs = currentTimeUs;
    }
}

void performSanityChecks()
{
    if (rescueState.phase == RESCUE_IDLE) {
        rescueState.failure = RESCUE_HEALTHY;

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
        rescueState.failure = RESCUE_CRASH_DETECTED;
    }

    //  Things that should run at a low refresh rate (such as flyaway detection, etc)
    //  This runs at ~1hz

    static uint32_t previousTimeUs;

    const uint32_t currentTimeUs = micros();
    const uint32_t dTime = currentTimeUs - previousTimeUs;

    if (dTime < 1000000) { //1hz
        return;
    }

    previousTimeUs = currentTimeUs;

     // Stalled movement detection
    static int8_t gsI = 0;

    gsI = constrain(gsI + (rescueState.sensor.groundSpeed < 150) ? 1 : -1, -10, 10);

    if (gsI == 10) {
        rescueState.failure = RESCUE_CRASH_DETECTED;
    }

    // Minimum sat detection
    static int8_t msI = 0;

    msI = constrain(msI + (rescueState.sensor.numSat < gpsRescueConfig()->minSats) ? 1 : -1, -5, 5);

    if (msI == 5) {
        rescueState.failure = RESCUE_FLYAWAY;
    }

    // Minimum distance detection (100m)
    if (rescueState.sensor.distanceToHome < 100) {
        rescueState.failure = RESCUE_TOO_CLOSE;
    }
}

void rescueStart()
{
    rescueState.phase = RESCUE_INITIALIZE;
}

void rescueStop()
{
    rescueState.phase = RESCUE_IDLE;
}

// Things that need to run regardless of GPS rescue mode being enabled or not
void idleTasks()
{
    // Do not calculate any of the idle task values when we are not flying
    if (!ARMING_FLAG(ARMED)) {
        return;
    }

    // Don't update any rescue flight statistics if we haven't applied a proper altitude offset yet
    if (!isAltitudeOffset()) {
        return;
    }

    gpsRescueAngle[AI_PITCH] = 0;
    gpsRescueAngle[AI_ROLL] = 0;

    // Store the max altitude we see not during RTH so we know our fly-back minimum alt
    rescueState.sensor.maxAltitude = MAX(rescueState.sensor.currentAltitude, rescueState.sensor.maxAltitude);
    // Store the max distance to home during normal flight so we know if a flyaway is happening
    rescueState.sensor.maxDistanceToHome = MAX(rescueState.sensor.distanceToHome, rescueState.sensor.maxDistanceToHome);

    rescueThrottle = rcCommand[THROTTLE];

    //to do: have a default value for hoverThrottle
    
    // FIXME: GPS Rescue throttle handling should take into account min_check as the
    // active throttle is from min_check through PWM_RANGE_MAX. Currently adjusting for this
    // in gpsRescueGetThrottle() but it would be better handled here.

    float ct = getCosTiltAngle();
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

void rescueAttainPosition()
{
    // Point to home if that is in our intent
    if (rescueState.intent.crosstrack) {
        setBearing(rescueState.sensor.directionToHome);
    }

    if (!newGPSData) {
        return;
    }


    /**
        Speed controller
    */
    static float previousSpeedError = 0;
    static int16_t speedIntegral = 0;

    const int16_t speedError = (rescueState.intent.targetGroundspeed - rescueState.sensor.groundSpeed) / 100;
    const int16_t speedDerivative = speedError - previousSpeedError;

    speedIntegral = constrain(speedIntegral + speedError, -100, 100);

    previousSpeedError = speedError;

    int16_t angleAdjustment =  gpsRescueConfig()->velP * speedError + (gpsRescueConfig()->velI * speedIntegral) / 100 +  gpsRescueConfig()->velD * speedDerivative;

    gpsRescueAngle[AI_PITCH] = constrain(gpsRescueAngle[AI_PITCH] + MIN(angleAdjustment, 80), rescueState.intent.minAngleDeg * 100, rescueState.intent.maxAngleDeg * 100);

    float ct = cos(DECIDEGREES_TO_RADIANS(gpsRescueAngle[AI_PITCH] / 10));

    /**
        Altitude controller
    */
    static float previousAltitudeError = 0;
    static int16_t altitudeIntegral = 0;

    const int16_t altitudeError = (rescueState.intent.targetAltitude - rescueState.sensor.currentAltitude) / 100; // Error in meters
    const int16_t altitudeDerivative = altitudeError - previousAltitudeError;

    // Only allow integral windup within +-15m absolute altitude error
    if (ABS(altitudeError) < 25) {
        altitudeIntegral = constrain(altitudeIntegral + altitudeError, -250, 250);
    } else {
        altitudeIntegral = 0;
    }

    previousAltitudeError = altitudeError;

    int16_t altitudeAdjustment = (gpsRescueConfig()->throttleP * altitudeError + (gpsRescueConfig()->throttleI * altitudeIntegral) / 10 *  + gpsRescueConfig()->throttleD * altitudeDerivative) / ct / 20;
    int16_t hoverAdjustment = (hoverThrottle - 1000) / ct;

    rescueThrottle = constrain(1000 + altitudeAdjustment + hoverAdjustment, gpsRescueConfig()->throttleMin, gpsRescueConfig()->throttleMax);

    DEBUG_SET(DEBUG_RTH, 0, rescueThrottle);
    DEBUG_SET(DEBUG_RTH, 1, gpsRescueAngle[AI_PITCH]);
    DEBUG_SET(DEBUG_RTH, 2, altitudeAdjustment);
    DEBUG_SET(DEBUG_RTH, 3, rescueState.failure);
}

// Very similar to maghold function on betaflight/cleanflight
void setBearing(int16_t desiredHeading)
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
#endif

