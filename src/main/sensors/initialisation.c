/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "common/utils.h"

#include "config/feature.h"

#include "fc/config.h"
#include "fc/runtime_config.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"
#include "sensors/sonar.h"
#include "sensors/initialisation.h"

uint8_t detectedSensors[SENSOR_INDEX_COUNT] = { GYRO_NONE, ACC_NONE, BARO_NONE, MAG_NONE };


#ifdef SONAR
static bool sonarDetect(void)
{
    if (feature(FEATURE_SONAR)) {
        // the user has set the sonar feature, so assume they have an HC-SR04 plugged in,
        // since there is no way to detect it
        sensorsSet(SENSOR_SONAR);
        return true;
    }
    return false;
}
#endif

bool sensorsAutodetect(const gyroConfig_t *gyroConfig,
        const accelerometerConfig_t *accelerometerConfig,
        const compassConfig_t *compassConfig,
        const barometerConfig_t *barometerConfig,
        const sonarConfig_t *sonarConfig)
{
    // gyro must be initialised before accelerometer
    if (!gyroInit(gyroConfig)) {
        return false;
    }

    accInit(accelerometerConfig, gyro.targetLooptime);

    mag.magneticDeclination = 0.0f; // TODO investigate if this is actually needed if there is no mag sensor or if the value stored in the config should be used.
#ifdef MAG
    if (compassDetect(&mag.dev, compassConfig->mag_hardware)) {
        compassInit(compassConfig);
    }
#else
    UNUSED(compassConfig);
#endif

#ifdef BARO
    baroDetect(&baro.dev, barometerConfig->baro_hardware);
#else
    UNUSED(barometerConfig);
#endif

#ifdef SONAR
    if (sonarDetect()) {
        sonarInit(sonarConfig);
    }
#else
    UNUSED(sonarConfig);
#endif

    if (gyroConfig->gyro_align != ALIGN_DEFAULT) {
        gyro.dev.gyroAlign = gyroConfig->gyro_align;
    }
    if (accelerometerConfig->acc_align != ALIGN_DEFAULT) {
        acc.dev.accAlign = accelerometerConfig->acc_align;
    }
    if (compassConfig->mag_align != ALIGN_DEFAULT) {
        mag.dev.magAlign = compassConfig->mag_align;
    }

    return true;
}
