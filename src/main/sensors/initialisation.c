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
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

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

bool sensorsAutodetect(void)
{
    // gyro must be initialised before accelerometer
    if (!gyroInit()) {
        return false;
    }

    accInit(gyro.targetLooptime);

#ifdef MAG
    compassInit();
#endif

#ifdef BARO
    baroDetect(&baro.dev, barometerConfig()->baro_hardware);
#endif

#ifdef SONAR
    if (sonarDetect()) {
        sonarInit(sonarConfig());
    }
#endif

    return true;
}
