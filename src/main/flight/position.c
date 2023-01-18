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

static float displayAltitudeCm = 0.0f;
static float zeroedAltitudeCm = 0.0f;

#if defined(USE_BARO) || defined(USE_GPS)
static float zeroedAltitudeDerivative = 0.0f;
#endif

static pt2Filter_t altitudeLpf;
static pt2Filter_t altitudeDerivativeLpf;
#ifdef USE_VARIO
static int16_t estimatedVario = 0; // in cm/s
#endif

void positionInit(void)
{
    const float sampleTimeS = HZ_TO_INTERVAL(TASK_ALTITUDE_RATE_HZ);

    const float altitudeCutoffHz = positionConfig()->altitude_lpf / 100.0f;
    const float altitudeGain = pt2FilterGain(altitudeCutoffHz, sampleTimeS);
    pt2FilterInit(&altitudeLpf, altitudeGain);

    const float altitudeDerivativeCutoffHz = positionConfig()->altitude_d_lpf / 100.0f;
    const float altitudeDerivativeGain = pt2FilterGain(altitudeDerivativeCutoffHz, sampleTimeS);
    pt2FilterInit(&altitudeDerivativeLpf, altitudeDerivativeGain);
}

typedef enum {
    DEFAULT = 0,
    BARO_ONLY,
    GPS_ONLY
} altitudeSource_e;

PG_REGISTER_WITH_RESET_TEMPLATE(positionConfig_t, positionConfig, PG_POSITION, 4);

PG_RESET_TEMPLATE(positionConfig_t, positionConfig,
    .altitude_source = DEFAULT,
    .altitude_prefer_baro = 100, // percentage 'trust' of baro data
    .altitude_lpf = 300,
    .altitude_d_lpf = 100,
);

#if defined(USE_BARO) || defined(USE_GPS)
void calculateEstimatedAltitude(void)
{
    static bool wasArmed = false;
    static bool useZeroedGpsAltitude = false; // whether a zero for the GPS altitude value exists
    static float gpsAltCm = 0.0f; // will hold last value on transient loss of 3D fix
    static float gpsAltOffsetCm = 0.0f;
    static float baroAltOffsetCm = 0.0f;
    static float newBaroAltOffsetCm = 0.0f;

    float baroAltCm = 0.0f;
    float gpsTrust = 0.3f; // if no hDOP value, use 0.3
    bool haveBaroAlt = false; // true if baro exists and has been calibrated on power up
    bool haveGpsAlt = false; // true if GPS is connected and while it has a 3D fix, set each run to false

    // *** Get sensor data
#ifdef USE_BARO
    if (sensors(SENSOR_BARO)) {
        baroAltCm = getBaroAltitude();
        haveBaroAlt = true; // false only if there is no sensor on the board, or it has failed
    }
#endif
#ifdef USE_GPS
    if (sensors(SENSOR_GPS) && STATE(GPS_FIX)) {
        // GPS_FIX means a 3D fix, which requires min 4 sats.
        // On loss of 3D fix, gpsAltCm remains at the last value, haveGpsAlt becomes false, and gpsTrust goes to zero.
        gpsAltCm = gpsSol.llh.altCm; // static, so hold last altitude value if 3D fix is lost to prevent fly to moon
        haveGpsAlt = true; // stays false if no 3D fix
        if (gpsSol.dop.hdop != 0) {
            gpsTrust = 100.0f / gpsSol.dop.hdop;
            // *** TO DO - investigate if we should use vDOP or vACC with UBlox units;
        }
        // always use at least 10% of other sources besides gps if available
        gpsTrust = MIN(gpsTrust, 0.9f);
    }
#endif

    //  ***  DISARMED  ***
    if (!ARMING_FLAG(ARMED)) {
        if (wasArmed) { // things to run once, on disarming, after being armed
            useZeroedGpsAltitude = false; // reset, and wait for valid GPS data to zero the GPS signal
            wasArmed = false;
        }

        newBaroAltOffsetCm = 0.2f * baroAltCm + 0.8f * newBaroAltOffsetCm; // smooth some recent baro samples
        displayAltitudeCm = baroAltCm - baroAltOffsetCm; // if no GPS altitude, show un-smoothed Baro altitude in OSD and sensors tab, using most recent offset.

        if (haveGpsAlt) { // watch for valid GPS altitude data to get a zero value from
            gpsAltOffsetCm = gpsAltCm; // update the zero offset value with the most recent valid gps altitude reading
            useZeroedGpsAltitude = true; // we can use this offset to zero the GPS altitude on arming
            if (!(positionConfig()->altitude_source == BARO_ONLY)) {
                displayAltitudeCm = gpsAltCm; // estimatedAltitude shows most recent ASL GPS altitude in OSD and sensors, while disarmed
            }
        }
        zeroedAltitudeCm = 0.0f; // always hold relativeAltitude at zero while disarmed

    //  ***  ARMED  ***
    } else {
        if (!wasArmed) { // things to run once, on arming, after being disarmed
            baroAltOffsetCm = newBaroAltOffsetCm;
            wasArmed = true;
        }

        baroAltCm -= baroAltOffsetCm; // use smoothed baro with most recent zero from disarm period

        if (haveGpsAlt) { // update relativeAltitude with every new gpsAlt value, or hold the previous value until 3D lock recovers
            if (!useZeroedGpsAltitude && haveBaroAlt) { // armed without zero offset, can use baro values to zero later
                gpsAltOffsetCm = gpsAltCm - baroAltCm; // not very accurate
                useZeroedGpsAltitude = true;
            }
            if (useZeroedGpsAltitude) { // normal situation
                zeroedAltitudeCm = gpsAltCm - gpsAltOffsetCm; // now that we have a GPS offset value, we can use it to zero relativeAltitude
            }
        } else {
            gpsTrust = 0.0f;
            // TO DO - smoothly reduce GPS trust, rather than immediately dropping to zero for what could be only a very brief loss of 3D fix 
        }

        // Empirical mixing of GPS and Baro altitudes
        if (useZeroedGpsAltitude && (positionConfig()->altitude_source == DEFAULT || positionConfig()->altitude_source == GPS_ONLY)) {
            if (haveBaroAlt && positionConfig()->altitude_source == DEFAULT) {
                // mix zeroed GPS with Baro altitude data, if Baro data exists if are in default altitude control mode 
                const float absDifferenceM = fabsf(zeroedAltitudeCm - baroAltCm) / 100.0f * positionConfig()->altitude_prefer_baro / 100.0f;
                if (absDifferenceM > 1.0f) { // when there is a large difference, favour Baro
                    gpsTrust /=  absDifferenceM;
                }
                zeroedAltitudeCm = zeroedAltitudeCm * gpsTrust + baroAltCm * (1.0f - gpsTrust);
            }
        } else if (haveBaroAlt && (positionConfig()->altitude_source == DEFAULT || positionConfig()->altitude_source == BARO_ONLY)) {
            zeroedAltitudeCm = baroAltCm; // use Baro if no GPS data, or we want Baro only
        }
    }

      zeroedAltitudeCm = pt2FilterApply(&altitudeLpf, zeroedAltitudeCm);
      // NOTE: this filter must receive 0 as its input, for the whole disarmed time, to ensure correct zeroed values on arming

    if (wasArmed) {
        displayAltitudeCm = zeroedAltitudeCm; // while armed, show filtered relative altitude in OSD / sensors tab
    }

    // *** calculate Vario signal
    static float previousZeroedAltitudeCm = 0.0f;
    zeroedAltitudeDerivative = (zeroedAltitudeCm - previousZeroedAltitudeCm) * TASK_ALTITUDE_RATE_HZ; // cm/s
    previousZeroedAltitudeCm = zeroedAltitudeCm;

    zeroedAltitudeDerivative = pt2FilterApply(&altitudeDerivativeLpf, zeroedAltitudeDerivative);

#ifdef USE_VARIO
    estimatedVario = lrintf(zeroedAltitudeDerivative);
    estimatedVario = applyDeadband(estimatedVario, 10); // ignore climb rates less than 0.1 m/s
    estimatedVario = constrain(estimatedVario, -1500, 1500);
#endif
 
    // *** set debugs
    DEBUG_SET(DEBUG_ALTITUDE, 0, (int32_t)(100 * gpsTrust));
    DEBUG_SET(DEBUG_ALTITUDE, 1, baroAltCm);
    DEBUG_SET(DEBUG_ALTITUDE, 2, gpsAltCm);
#ifdef USE_VARIO
    DEBUG_SET(DEBUG_ALTITUDE, 3, estimatedVario);
#endif
    DEBUG_SET(DEBUG_RTH, 1, displayAltitudeCm);
    DEBUG_SET(DEBUG_BARO, 3, baroAltCm);
}
#endif //defined(USE_BARO) || defined(USE_GPS)

int32_t getEstimatedAltitudeCm(void)
{
    return lrintf(displayAltitudeCm);
}

float getAltitude(void)
{
    return zeroedAltitudeCm;
}

#ifdef USE_VARIO
int16_t getEstimatedVario(void)
{
    return estimatedVario;
}
#endif
