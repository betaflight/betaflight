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
#include <string.h>

#include "platform.h"

#include "common/utils.h"

#include "config/config.h"
#include "config/feature.h"

#include "fc/runtime_config.h"

#include "flight/pid.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sensors/acceleration.h"
#include "sensors/adcinternal.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/gyro_init.h"
#include "sensors/initialisation.h"
#include "sensors/rangefinder.h"
#include "sensors/sensors.h"


// requestedSensors is not actually used
uint8_t requestedSensors[SENSOR_INDEX_COUNT] = { GYRO_NONE, ACC_NONE, BARO_NONE, MAG_NONE, RANGEFINDER_NONE };
uint8_t detectedSensors[SENSOR_INDEX_COUNT] = { GYRO_NONE, ACC_NONE, BARO_NONE, MAG_NONE, RANGEFINDER_NONE };

void sensorsPreInit(void)
{
    gyroPreInit();

#ifdef USE_MAG
    compassPreInit();
#endif

#ifdef USE_BARO
    baroPreInit();
#endif
}

bool sensorsAutodetect(void)
{

    // gyro must be initialised before accelerometer

    bool gyroDetected = gyroInit();

#ifdef USE_ACC
    if (gyroDetected) {
        accInit(gyro.accSampleRateHz);
    }
#endif

#ifdef USE_MAG
    compassInit();
#endif

#ifdef USE_BARO
    baroInit();
#endif

#ifdef USE_RANGEFINDER
    rangefinderInit();
#endif

#ifdef USE_ADC_INTERNAL
    adcInternalInit();
#endif

    return gyroDetected;
}
