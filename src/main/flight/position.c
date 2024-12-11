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
#include "flight/altitude.h"

#include "io/gps.h"

#include "scheduler/scheduler.h"

#include "sensors/sensors.h"
#include "sensors/barometer.h"
#include "sensors/rangefinder.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"


static float displayAltitudeCm = 0.0f;
static bool altitudeAvailable = false;

static float zeroedAltitudeCm = 0.0f;
static float zeroedAltitudeDerivative = 0.0f;

#ifdef USE_VARIO
static int16_t estimatedVario = 0; // in cm/s
#endif

void positionInit(void) {
altSensorFusionInit();
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
    altitudeAvailable = altSensorFusionUpdate();
    if (altitudeAvailable) {
        zeroedAltitudeCm = getAltitudeState()->distCm;
        zeroedAltitudeDerivative = getAltitudeState()->velocityCm;
        displayAltitudeCm = zeroedAltitudeCm;
        estimatedVario = applyDeadband(lrintf(zeroedAltitudeDerivative), 10);
    }
}

#endif //defined(USE_BARO) || defined(USE_GPS) || defined(USE_RANGEFINDER)

float getAltitudeCm(void)
{
    return getAltitudeState()->distCm;
}

float getAltitudeDerivative(void)
{
    return zeroedAltitudeCm;
}

bool isAltitudeAvailable(void) {
    return altitudeAvailable;
}

int32_t getEstimatedAltitudeCm(void)
{
    return zeroedAltitudeDerivative; // cm/s
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
