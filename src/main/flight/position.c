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
#include "flight/kalman_filter_1d.h"

#include "io/gps.h"

#include "scheduler/scheduler.h"

#include "sensors/sensors.h"
#include "sensors/barometer.h"

#ifdef USE_RANGEFINDER
#include "sensors/rangefinder.h"
#endif

#include "pg/pg.h"
#include "pg/pg_ids.h"

static float displayAltitudeCm = 0.0f;
static bool altitudeAvailable = false;

static float zeroedGpsAltitudeCm = 0.0f;

static float zeroedFusedAltitudeCm = 0.0f;
static float zeroedFusedAltitudeCmDerivative = 0.0f;

#ifdef USE_VARIO
static int16_t estimatedVario = 0; // in cm/s
#endif

void positionInit(void) {

}

typedef enum {
    DEFAULT = 0,
    BARO_ONLY,
    GPS_ONLY
} altitudeSource_e;

PG_REGISTER_WITH_RESET_TEMPLATE(positionConfig_t, positionConfig, PG_POSITION, 6);

PG_RESET_TEMPLATE(positionConfig_t, positionConfig,
    .altitude_source = DEFAULT,
    .altitude_prefer_baro = 100, // percentage 'trust' of baro data
    .altitude_lpf = 300,
    .altitude_d_lpf = 100,
);

#if defined(USE_BARO) || defined(USE_GPS) || defined(USE_RANGEFINDER)
void calculateEstimatedAltitude(void) {
    static bool wasArmed = false;
    static bool useZeroedGpsAltitude = false; // whether a zero for the GPS altitude value exists
    static float gpsAltCm = 0.0f; // will hold last value on transient loss of 3D fix
    static float gpsAltOffsetCm = 0.0f;
    static float baroAltOffsetCm = 0.0f;
    static float newBaroAltOffsetCm = 0.0f;
    static KalmanFilter kf;
    static bool kfInitDone = false;
    static uint32_t prevGpsTime = 0; // time of last GPS data
    float baroAltCm = 0.0f;
    bool haveBaroAlt = false; // true if baro exists and has been calibrated on power up
    bool haveGpsAlt = false; // true if GPS is connected and while it has a 3D fix, set each run to false
    bool haveRangefinderAlt = false; // true if rangefinder is connected and has a valid reading
    bool gpsHasNewData = false; // true if GPS has new data since last run

    // *** Get sensor data
#ifdef USE_BARO
    static SensorMeasurement baroAltMeasurement = { .value = -1.0f, .variance = 100.0f };

    if (sensors(SENSOR_BARO)) {
        baroAltCm = getBaroAltitude();
        haveBaroAlt = true; // false only if there is no sensor on the board, or it has failed
    }

#endif

#ifdef USE_GPS
    static SensorMeasurement gpsAltMeasurement = { .value = -1.0f, .variance = 10000.0f };
    if (sensors(SENSOR_GPS) && STATE(GPS_FIX)) {
        // GPS_FIX means a 3D fix, which requires min 4 sats.
        // On loss of 3D fix, gpsAltCm remains at the last value, haveGpsAlt becomes false
        gpsAltCm = gpsSol.llh.altCm; // static, so hold last altitude value if 3D fix is lost to prevent fly to moon
        haveGpsAlt = true; // goes false and stays false if no 3D fix
        if (gpsSol.dop.pdop != 0) {
            // pDOP of 1.0 is good.  100 is very bad.  Our gpsSol.dop.pdop values are *100
            // *** TO DO - investigate if we should use vDOP or vACC with UBlox units;
            gpsAltMeasurement.variance = gpsSol.dop.pdop; // best variance is 100, worst 10000
        }
        if (gpsSol.time != prevGpsTime) {
            gpsHasNewData = true;
            prevGpsTime = gpsSol.time;
        }
    }
#endif

#ifdef USE_RANGEFINDER
    haveRangefinderAlt = rangefinderIsHealthy();
    static SensorMeasurement rangefinderMeasurement = { .value = -1.0f, .variance = 1.0f };  // todo: rangefinder init variance
    rangefinderMeasurement.value = (float)rangefinderGetLatestAltitude();
    static float rangefinderOffsetCm = 0.0f;
#endif

    if(!kfInitDone) { //first kf init
        kf_init(&kf, 0.0f, 0.5, 0.5); //to do: init variance
        kfInitDone = true;
    }
    
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
        zeroedGpsAltitudeCm = 0.0f; // always hold relativeAltitude at zero while disarmed
        
        updateBaroVariance(&baroAltMeasurement, baroAltCm); // update the baro variance while disarmed

        DEBUG_SET(DEBUG_ALTITUDE, 2, gpsAltCm / 100.0f); // Absolute altitude ASL in metres, max 32,767m
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
                zeroedGpsAltitudeCm = gpsAltCm - gpsAltOffsetCm; // now that we have a GPS offset value, we can use it to zero relativeAltitude
            }
        }
    
        // apply Kalman filter on available sensors
        kf_update_variance(&kf);

#ifdef USE_RANGEFINDER
        if(haveRangefinderAlt) {
            kf_update(&kf, rangefinderMeasurement);
            // update the offset to shift the zeroedGpsAltitudeCm and the baroAltCm to match the rangefinder zero point
            if (haveBaroAlt) {
                rangefinderOffsetCm = 0.01 * (baroAltCm - rangefinderMeasurement.value)
                                    + 0.99 * rangefinderOffsetCm;
            }
        }
        zeroedGpsAltitudeCm -= rangefinderOffsetCm;
        baroAltCm           -= rangefinderOffsetCm;
#endif

#ifdef USE_BARO
        if (haveBaroAlt) {
            baroAltMeasurement.value = baroAltCm;
            kf_update(&kf, baroAltMeasurement);
        }
#endif

#ifdef USE_GPS
        if (haveGpsAlt && gpsHasNewData) {
            gpsAltMeasurement.value = zeroedGpsAltitudeCm;
            kf_update(&kf, gpsAltMeasurement);
        }
#endif

        zeroedFusedAltitudeCm = kf.estimatedValue;

}

    if (wasArmed) {
        displayAltitudeCm = zeroedFusedAltitudeCm; // while armed, show filtered relative altitude in OSD / sensors tab
    }

    // *** calculate Vario signal
    static float previousZeroedFusedAltitudeCm = 0.0f;
    zeroedFusedAltitudeCmDerivative = (zeroedFusedAltitudeCm - previousZeroedFusedAltitudeCm) * TASK_ALTITUDE_RATE_HZ; // cm/s
    previousZeroedFusedAltitudeCm = zeroedFusedAltitudeCm;

    // zeroedFusedAltitudeCmDerivative = pt2FilterApply(&altitudeDerivativeLpf, zeroedFusedAltitudeCmDerivative); // do we still need this ?

#ifdef USE_VARIO
    estimatedVario = lrintf(zeroedFusedAltitudeCmDerivative);
    estimatedVario = applyDeadband(estimatedVario, 10); // ignore climb rates less than 0.1 m/s
#endif
 
    // *** set debugs
#ifdef USE_BARO
    DEBUG_SET(DEBUG_ALTITUDE, 1, lrintf(baroAltCm / 10.0f)); // Relative altitude above takeoff, to 0.1m, rolls over at 3,276.7m
    DEBUG_SET(DEBUG_ALTITUDE, 5, lrintf(baroAltMeasurement.variance));
#endif

#ifdef USE_GPS
    if (ARMING_FLAG(ARMED)) {
        DEBUG_SET(DEBUG_ALTITUDE, 2, lrintf(zeroedGpsAltitudeCm / 10.0f)); // Relative altitude above takeoff, to 0.1m, rolls over at 3,276.7m
    }
    DEBUG_SET(DEBUG_ALTITUDE, 6, lrintf(gpsAltMeasurement.variance));
#endif

#ifdef USE_RANGEFINDER
    DEBUG_SET(DEBUG_ALTITUDE, 0, lrintf(rangefinderOffsetCm));
    DEBUG_SET(DEBUG_ALTITUDE, 4, lrintf(rangefinderMeasurement.value  / 10.0f));
    DEBUG_SET(DEBUG_ALTITUDE, 7, lrintf(rangefinderMeasurement.variance));
#endif

#ifdef USE_VARIO
    DEBUG_SET(DEBUG_ALTITUDE, 3, estimatedVario);
#endif
    DEBUG_SET(DEBUG_RTH, 1, lrintf(displayAltitudeCm / 10.0f));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 2, lrintf(zeroedFusedAltitudeCm));

    altitudeAvailable = haveGpsAlt || haveBaroAlt || haveRangefinderAlt;
}

#endif //defined(USE_BARO) || defined(USE_GPS) || defined(USE_RANGEFINDER)

float getAltitudeCm(void)
{
    return zeroedFusedAltitudeCm;
}

float getAltitudeDerivative(void)
{
    return zeroedFusedAltitudeCmDerivative; // cm/s
}

bool isAltitudeAvailable(void) {
    return altitudeAvailable;
}

int32_t getEstimatedAltitudeCm(void)
{
    return lrintf(displayAltitudeCm);
}

#ifdef USE_GPS
float getAltitudeAsl(void)
{
    return gpsSol.llh.altCm;
}
#endif

#ifdef USE_VARIO
int16_t getEstimatedVario(void)
{
    return estimatedVario;
}
#endif
