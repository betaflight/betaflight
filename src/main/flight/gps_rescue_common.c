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

#include "common/maths.h"

#include "config/config.h"
#include "drivers/time.h"

#include "fc/core.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/position.h"

#include "io/gps.h"
#include "sensors/acceleration.h"

#include "gps_rescue_common.h"
#include "gps_rescue.h"

bool newGPSData = false;
float gpsRescueAngle[ANGLE_INDEX_COUNT] = { 0, 0 };
float rescueYaw;

float gpsRescueGetYawRate(void)
{
    return rescueYaw;
}

/*
 If we have new GPS data, update home heading if possible and applicable.
*/
void gpsRescueNewGpsData(void)
{
    newGPSData = true;
}

// Things that need to run when GPS Rescue is enabled, and while armed, but while there is no Rescue in place
void setReturnAltitude(rescueIntentCommon_s *intentCommon, const rescueSensorData_s *sensorData)
{
    // Hold maxAltitude at zero while disarmed, but if set_home_point_once is true, hold maxAlt until power cycled
    if (!ARMING_FLAG(ARMED) && !gpsConfig()->gps_set_home_point_once) {
        intentCommon->maxAltitudeCm = 0.0f;
        return;
    }

    // While armed, but not during the rescue, update the max altitude value
    intentCommon->maxAltitudeCm = fmaxf(getAltitudeCm(), intentCommon->maxAltitudeCm);

    if (newGPSData) {
        // set the target altitude to the current altitude.
        intentCommon->targetAltitudeCm = getAltitudeCm();

        // Intended descent distance for rescues that start outside the minStartDistM distance
        // Set this to the user's intended descent distance, but not more than half the distance to home to ensure some fly home time
        intentCommon->descentDistanceM = fminf(0.5f * sensorData->distanceToHomeM, gpsRescueConfig()->descentDistanceM);

        const float initialClimbCm = gpsRescueConfig()->initialClimbM * 100.0f;
        switch (gpsRescueConfig()->altitudeMode) {
            case GPS_RESCUE_ALT_MODE_FIXED:
                intentCommon->returnAltitudeCm = gpsRescueConfig()->returnAltitudeM * 100.0f;
                break;
            case GPS_RESCUE_ALT_MODE_CURRENT:
                // climb above current altitude, but always return at least initial height above takeoff point, in case current altitude was negative
                intentCommon->returnAltitudeCm = fmaxf(initialClimbCm, getAltitudeCm() + initialClimbCm);
                break;
            case GPS_RESCUE_ALT_MODE_MAX:
            default:
                intentCommon->returnAltitudeCm = intentCommon->maxAltitudeCm + initialClimbCm;
                break;
        }
    }
}

void sensorUpdate(rescueSensorData_s *sensorData)
{
    static float prevDistanceToHomeCm = 0.0f;

    const float altitudeCurrentCm = getAltitudeCm();
    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 2, lrintf(altitudeCurrentCm));
    DEBUG_SET(DEBUG_GPS_RESCUE_HEADING, 0, sensorData->groundSpeedCmS);  // groundspeed cm/s
    DEBUG_SET(DEBUG_GPS_RESCUE_HEADING, 1, gpsSol.groundCourse);                // degrees * 10
    DEBUG_SET(DEBUG_GPS_RESCUE_HEADING, 2, attitude.values.yaw);                // degrees * 10
    DEBUG_SET(DEBUG_GPS_RESCUE_HEADING, 3, sensorData->directionToHome); // computed from current GPS position in relation to home
    sensorData->healthy = gpsIsHealthy();

    sensorData->directionToHome = GPS_directionToHome; // extern value from gps.c using current position relative to home
    sensorData->errorAngle = (attitude.values.yaw - sensorData->directionToHome) / 10.0f;
    // both attitude and direction are in degrees * 10, errorAngle is degrees
    if (sensorData->errorAngle <= -180) {
        sensorData->errorAngle += 360;
    } else if (sensorData->errorAngle > 180) {
        sensorData->errorAngle -= 360;
    }
    sensorData->absErrorAngle = fabsf(sensorData->errorAngle);
    
    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 4, lrintf(attitude.values.yaw));          // estimated heading of the quad (direction nose is pointing in)
    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 5, lrintf(sensorData->directionToHome));  // angle to home derived from GPS location and home position

    if (!newGPSData) {
        return;
        // GPS ground speed, velocity and distance to home will be held at last good values if no new packets
    }

    sensorData->distanceToHomeCm = GPS_distanceToHomeCm;
    sensorData->distanceToHomeM = sensorData->distanceToHomeCm / 100.0f;
    sensorData->groundSpeedCmS = gpsSol.groundSpeed; // cm/s

    sensorData->gpsDataIntervalSeconds = getGpsDataIntervalSeconds();
    // Range from 10ms (100hz) to 1000ms (1Hz). Intended to cover common GPS data rates and exclude unusual values.

    sensorData->velocityToHomeCmS = ((prevDistanceToHomeCm - sensorData->distanceToHomeCm) / sensorData->gpsDataIntervalSeconds);
    // positive = towards home.  First value is useless since prevDistanceToHomeCm was zero.
    prevDistanceToHomeCm = sensorData->distanceToHomeCm;

    DEBUG_SET(DEBUG_ATTITUDE, 4, sensorData->velocityToHomeCmS); // velocity to home

    DEBUG_SET(DEBUG_GPS_RESCUE_VELOCITY, 2, lrintf(sensorData->velocityToHomeCmS));
    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 0, lrintf(sensorData->velocityToHomeCmS));
}

void disarmOnImpact(const rescueIntentCommon_s *intentCommon, void (*rescueStopFunc)(void))
{
    if (acc.accMagnitude > intentCommon->disarmThreshold) {
        setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
        disarm(DISARM_REASON_GPS_RESCUE);
        rescueStopFunc();
    }
}

bool gpsRescueIsConfigured(void)
{
    return failsafeConfig()->failsafe_procedure == FAILSAFE_PROCEDURE_GPS_RESCUE || isModeActivationConditionPresent(BOXGPSRESCUE);
}

bool gpsRescueIsDisabled(void)
// used for OSD warning
{
    return (!STATE(GPS_FIX_HOME));
}

// This function flashes "RESCUE N/A" in the OSD if:
// 1. sensor healthy - GPS data is being received.
// 2. GPS has a 3D fix.
// 3. GPS number of satellites is greater than or equal to the minimum configured satellite count.
// Note 1: cannot arm without the required number of sats
// hence this flashing indicates that after having enough sats, we now have below the minimum and the rescue likely would fail
// Note 2: this function does not take into account the distance from home
// The sanity checks are independent, this just provides the OSD warning
bool checkGPSRescueIsAvailable(void)
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

#endif // USE_GPS_RESCUE
