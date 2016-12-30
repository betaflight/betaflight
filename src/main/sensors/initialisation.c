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

#include "platform.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/initialisation.h"
#include "sensors/sonar.h"

uint8_t detectedSensors[SENSOR_INDEX_COUNT] = { GYRO_NONE, ACC_NONE, BARO_NONE, MAG_NONE };


bool sensorsAutodetect(void)
{
    // gyro must be initialised before accelerometer
    if (!gyroInit()) {
        return false;
    }

    accInit();

    mag.magneticDeclination = 0.0f; // TODO investigate if this is actually needed if there is no mag sensor or if the value stored in the config should be used.
#ifdef MAG
    compassInit();
#endif

#ifdef BARO
    baroInit();
#endif

#ifdef SONAR
    sonarInit();
#endif

    return true;
}
