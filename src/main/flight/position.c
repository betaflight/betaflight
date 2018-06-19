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

#include "build/debug.h"

#include "common/maths.h"

#include "fc/runtime_config.h"

#include "flight/position.h"
#include "flight/imu.h"
#include "flight/pid.h"

#include "io/gps.h"

#include "sensors/sensors.h"
#include "sensors/barometer.h"

static int32_t estimatedAltitude = 0;                // in cm

#define BARO_UPDATE_FREQUENCY_40HZ (1000 * 25)


#if defined(USE_BARO) || defined(USE_GPS)
static bool altitudeOffsetSet = false;

void calculateEstimatedAltitude(timeUs_t currentTimeUs)
{
    static timeUs_t previousTimeUs = 0;
    static int32_t baroAltOffset = 0;
    static int32_t gpsAltOffset = 0;

    const uint32_t dTime = currentTimeUs - previousTimeUs;
    if (dTime < BARO_UPDATE_FREQUENCY_40HZ) {
        return;
    }
    previousTimeUs = currentTimeUs;

    int32_t baroAlt = 0;

    int32_t gpsAlt = 0;
    float gpsTrust = 0.3; //conservative default
    bool haveBaroAlt = false;
    bool haveGpsAlt = false;
#ifdef USE_BARO
    if (sensors(SENSOR_BARO)) {
        if (!isBaroCalibrationComplete()) {
            performBaroCalibrationCycle();
        } else {
            baroAlt = baroCalculateAltitude();
            haveBaroAlt = true;
        }
    }
#endif

#ifdef USE_GPS
if (sensors(SENSOR_GPS) && STATE(GPS_FIX)) {
    gpsAlt = gpsSol.llh.alt;
    haveGpsAlt = true;

    if (gpsSol.hdop != 0) {
        gpsTrust = 100.0 / gpsSol.hdop;
    }
    // always use at least 10% of other sources besides gps if available
    gpsTrust = MIN(gpsTrust, 0.9f);
}
#endif

    if (ARMING_FLAG(ARMED) && !altitudeOffsetSet) {
        baroAltOffset = baroAlt;
        gpsAltOffset = gpsAlt;
        altitudeOffsetSet = true;
    } else if (!ARMING_FLAG(ARMED) && altitudeOffsetSet) {
        altitudeOffsetSet = false;
    }
    baroAlt -= baroAltOffset;
    gpsAlt -= gpsAltOffset;
    
    if (haveGpsAlt && haveBaroAlt) {
        estimatedAltitude = gpsAlt * gpsTrust + baroAlt * (1 - gpsTrust);
    } else if (haveGpsAlt) {
        estimatedAltitude = gpsAlt;
    } else if (haveBaroAlt) {
        estimatedAltitude = baroAlt;
    }
    
    DEBUG_SET(DEBUG_ALTITUDE, 0, (int32_t)(100 * gpsTrust));
    DEBUG_SET(DEBUG_ALTITUDE, 1, baroAlt);
    DEBUG_SET(DEBUG_ALTITUDE, 2, gpsAlt);
}

bool isAltitudeOffset(void)
{
    return altitudeOffsetSet;
}
#endif

int32_t getEstimatedAltitude(void)
{
    return estimatedAltitude;
}

// This should be removed or fixed, but it would require changing a lot of other things to get rid of.
int16_t getEstimatedVario(void)
{
    return 0;
}
