/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <limits.h>

#include "platform.h"

#include "build/debug.h"

#include "common/maths.h"
#include "common/filter.h"

#include "fc/runtime_config.h"

#include "flight/position.h"
#include "flight/imu.h"
#include "flight/pid.h"

#include "io/gps.h"

#include "scheduler/scheduler.h"

#include "sensors/sensors.h"
#include "sensors/barometer.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

static int32_t estimatedAltitudeCm = 0;                // in cm
#ifdef USE_BARO
    static pt2Filter_t baroDerivativeLpf;
#endif

typedef enum {
    DEFAULT = 0,
    BARO_ONLY,
    GPS_ONLY
} altSource_e;

PG_REGISTER_WITH_RESET_TEMPLATE(positionConfig_t, positionConfig, PG_POSITION, 3);

PG_RESET_TEMPLATE(positionConfig_t, positionConfig,
    .altSource = DEFAULT,
    .altPreferBaro = 100,
);

#ifdef USE_VARIO
static int16_t estimatedVario = 0;                   // in cm/s

int16_t calculateEstimatedVario(float baroAltVelocity)
{
    baroAltVelocity = applyDeadband(baroAltVelocity, 10.0f); // cm/s, so ignore climb rates less than 0.1 m/s
    return constrain(lrintf(baroAltVelocity), -1500, 1500);
}
#endif

#if defined(USE_BARO) || defined(USE_GPS)
static bool altitudeOffsetSetBaro = false;
static bool altitudeOffsetSetGPS = false;

void calculateEstimatedAltitude()
{
    float baroAltCm = 0;
    float gpsAltCm = 0;
    float baroAltVelocity = 0;
    float gpsTrust = 0.3; //conservative default
    bool haveBaroAlt = false;
    bool haveGpsAlt = false;
#if defined(USE_GPS) && defined(USE_VARIO)
    float gpsVertSpeed = 0;
#endif

// *** Set baroAlt once its calibration is complete (cannot arm until it is)
#ifdef USE_BARO
    if (sensors(SENSOR_BARO)) {
        static float lastBaroAltCm = 0;
        static bool initBaroFilter = false;
        if (!initBaroFilter) {
            const float cutoffHz = barometerConfig()->baro_vario_lpf / 100.0f;
            const float sampleTimeS = HZ_TO_INTERVAL(TASK_ALTITUDE_RATE_HZ);
            const float gain = pt2FilterGain(cutoffHz, sampleTimeS);
            pt2FilterInit(&baroDerivativeLpf, gain);
            initBaroFilter = true;
        }
        baroAltCm = baroUpsampleAltitude();
        const float baroAltVelocityRaw = (baroAltCm - lastBaroAltCm) * TASK_ALTITUDE_RATE_HZ; // cm/s
        baroAltVelocity = pt2FilterApply(&baroDerivativeLpf, baroAltVelocityRaw);
        lastBaroAltCm = baroAltCm;
        if (baroIsCalibrated()) {
            haveBaroAlt = true;
        }
    }
#endif

    // *** Check GPS for 3D fix, set haveGpsAlt if we have a 3D fix, and GPS Trust based on hdop, or leave at default of 0.3
#ifdef USE_GPS
    if (sensors(SENSOR_GPS) && STATE(GPS_FIX)) {
        // Need a 3D fix, which requires min 4 sats
        // if not, gpsAltCm remains zero, haveGpsAlt stays false, and gpsTrust is zero.
        gpsAltCm = gpsSol.llh.altCm;
#ifdef USE_VARIO
        gpsVertSpeed = GPS_verticalSpeedInCmS;
#endif
        haveGpsAlt = true; // remains false if no 3D fix

#ifdef USE_GPS
        if (gpsSol.hdop != 0) {
            gpsTrust = 100.0 / gpsSol.hdop;
            // *** TO DO - investigate if we should use vDOP or vACC with UBlox units; this hDOP value is actually pDOP in UBLox code !!!
        }
#endif
        // always use at least 10% of other sources besides gps if available; limit effect of HDOP
        gpsTrust = MIN(gpsTrust, 0.9f);
        // With a 3D fix, GPS trust starts at 0.3
    } else {
        gpsTrust = 0.0f; // don't trust GPS if no sensor or 3D fix
    }
#endif

    // *** Zero Baro Altitude on arming (every time we re-arm, also)
    // note that arming is blocked until baro 'calibration' (baro ground zeroing) is complete, no need to check status of haveBaroAlt
    // this code adds a secondary zeroing to whatever baro altitude value exists on arming
    // since props spin on arming, we want the last value before arming
    // provided that this function is called before significant motor spin-up has occured, this may not be a big problem
#ifdef USE_BARO
    static float baroAltOffsetCm = 0;
    if (ARMING_FLAG(ARMED)) {
        baroAltCm -= baroAltOffsetCm; // use the last offset value from the disarm period
        altitudeOffsetSetBaro = true; // inevitable, but needed if no GPS
    } else {
        baroAltOffsetCm = baroAltCm; // while disarmed, keep capturing current altitude to zero any offset once armed
    }
#endif

    // *** Zero GPS Altitude on every arm or re-arm using most recent disarmed values
    // but do not use GPS if there are not the required satellites

    // note that if GPS Rescue is enabled, a Home fix is required and reset when the user arms, so the checks below aren't needed
    // tryArm() runs code that sets GPS_FIX_HOME, and if the result is no home fix, arming is blocked
    // a check like this is essential for GPS Rescue
    // if an altitude hold function was enabled, the same test would be good for ensuring a reasonable altitude estimate before takeoff

    // If GPS Rescue is not enabled, the user can arm without a Home fix, or indeed without a 3D fix, with GPS active
    // While disarmed, we use the current GPS value, regardless of GPS quality
    // Also while disarmed, we monitor for a 3D fix and at least the required number of satellites
    // If these are achieved before arming, we lock the offset at the gpsAltitude value
    // If we don't, we wait until we get a 3D fix, and then correct the offset using the baro value
    // This won't be very accurate, but all we need is a 3D fix.
    // Note that without the 3D fix, GPS trust will be zero, and on getting a 3D fix, will depend on hDOP
#ifdef USE_GPS
    static float gpsAltOffsetCm = 0;
    if (ARMING_FLAG(ARMED)) {
        gpsAltCm -= gpsAltOffsetCm; // while armed, use the last offset value from the disarm period
        if (!altitudeOffsetSetGPS && haveBaroAlt && haveGpsAlt) {
            // if we get a 3D fix after not getting a decent altitude offset, zero GPS to the Baro reading
            gpsAltOffsetCm = gpsAltCm - baroAltCm; // set GPS to Baro altitude once we get a GPS fix
            altitudeOffsetSetGPS = true;
        }
    } else {
        gpsAltOffsetCm = gpsAltCm; // while disarmed, keep capturing current altitude, to zero any offset once armed
        altitudeOffsetSetGPS = (haveGpsAlt && gpsConfig()->gpsRequiredSats);
        // if no 3D fix and not enough sats, set the GPS offset flag to false, and use baro to correct once we get a fix
    }
#endif

    // *** adjust gpsTrust, favouring Baro increasingly when there is a discrepancy of more than a meter
    // favour GPS if Baro reads negative, this happens due to ground effects
    float gpsTrustModifier = gpsTrust;
    const float absDifferenceM = fabsf(gpsAltCm - baroAltCm) * positionConfig()->altPreferBaro / 10000.0f;
    if (absDifferenceM > 1.0f && baroAltCm > -100.0f) { // significant difference, and baro altitude not negative
        gpsTrustModifier /=  absDifferenceM;
    }
    // eg if discrepancy is 3m and GPS trust was 0.9, it would now be 0.3

    // *** If we have  a GPS with 3D fix and a Baro signal, blend them
    if (haveGpsAlt && haveBaroAlt && positionConfig()->altSource == DEFAULT) {
        if (ARMING_FLAG(ARMED)) {
            estimatedAltitudeCm = gpsAltCm * gpsTrustModifier + baroAltCm * (1 - gpsTrustModifier);
        } else {
            estimatedAltitudeCm = gpsAltCm;
            //absolute GPS altitude is shown before arming, ignoring baro (I have no clue why this is)
        }

#ifdef USE_VARIO
        // baro is a better source for vario, so ignore gpsVertSpeed
        estimatedVario = calculateEstimatedVario(baroAltVelocity);
#endif

    // *** If we have a GPS but no baro, and are in Default or GPS_ONLY modes, use GPS values
    } else if (haveGpsAlt && (positionConfig()->altSource == GPS_ONLY || positionConfig()->altSource == DEFAULT )) {
        estimatedAltitudeCm = gpsAltCm;
#if defined(USE_VARIO) && defined(USE_GPS)
        estimatedVario = gpsVertSpeed;
#endif

    // *** If we have a Baro, and can work with it in Default or Baro Only modes
    } else if (haveBaroAlt && (positionConfig()->altSource == BARO_ONLY || positionConfig()->altSource == DEFAULT)) {
        estimatedAltitudeCm = baroAltCm;

#ifdef USE_VARIO
        estimatedVario = calculateEstimatedVario(baroAltVelocity); // cm/s
#endif
    }

    // Note that if we have no GPS but user chooses GPS Only, or no Baro but user chooses Baro only, then the reported altitude will be zero
    // The latter may cause GPS rescue to fail, but the user should notice an absence of altitude values.

    DEBUG_SET(DEBUG_ALTITUDE, 0, (int32_t)(100 * gpsTrust));
    DEBUG_SET(DEBUG_ALTITUDE, 1, baroAltCm);
    DEBUG_SET(DEBUG_ALTITUDE, 2, gpsAltCm);
    DEBUG_SET(DEBUG_ALTITUDE, 3, estimatedAltitudeCm);
}

bool isAltitudeOffset(void)
{
    return altitudeOffsetSetBaro || altitudeOffsetSetGPS;
}
#endif

int32_t getEstimatedAltitudeCm(void)
{
    return estimatedAltitudeCm;
}

#ifdef USE_VARIO
int16_t getEstimatedVario(void)
{
    return estimatedVario;
}
#endif
