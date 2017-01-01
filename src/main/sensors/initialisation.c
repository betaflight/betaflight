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

#include "config/config_eeprom.h"

#include "drivers/logging.h"

#include "fc/config.h"
#include "fc/runtime_config.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/pitotmeter.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"
#include "sensors/rangefinder.h"
#include "sensors/initialisation.h"

uint8_t requestedSensors[SENSOR_INDEX_COUNT] = { GYRO_AUTODETECT, ACC_NONE, BARO_NONE, MAG_NONE, RANGEFINDER_NONE, PITOT_NONE };
uint8_t detectedSensors[SENSOR_INDEX_COUNT] = { GYRO_NONE, ACC_NONE, BARO_NONE, MAG_NONE, RANGEFINDER_NONE, PITOT_NONE };


bool sensorsAutodetect(
                pitotmeterConfig_t *pitotConfig)
{
    bool eepromUpdatePending = false;

    if (!gyroInit()) {
        return false;
    }

#ifdef ASYNC_GYRO_PROCESSING
     // ACC will be updated at its own rate
    accInit(getAccUpdateRate());
#else
    // acc updated at same frequency in taskMainPidLoop in mw.c
    accInit(gyro.targetLooptime);
#endif

#ifdef BARO
    baroInit();
#endif

#ifdef PITOT
    pitotDetect(&pitot.dev, pitotConfig->pitot_hardware);
#else
    UNUSED(pitotConfig);
#endif

    // FIXME extract to a method to reduce dependencies, maybe move to sensors_compass.c
    mag.magneticDeclination = 0.0f; // TODO investigate if this is actually needed if there is no mag sensor or if the value stored in the config should be used.
#ifdef MAG
    compassInit();
#endif

#ifdef SONAR
    const rangefinderType_e rangefinderType = rangefinderDetect();
    rangefinderInit(rangefinderType);
#endif

    if (accelerometerConfig()->acc_hardware == ACC_AUTODETECT) {
        accelerometerConfig()->acc_hardware = detectedSensors[SENSOR_INDEX_ACC];
        eepromUpdatePending = true;
    }

#ifdef BARO
    if (barometerConfig()->baro_hardware == BARO_AUTODETECT) {
        barometerConfig()->baro_hardware = detectedSensors[SENSOR_INDEX_BARO];
        eepromUpdatePending = true;
    }
#endif

    if (compassConfig()->mag_hardware == MAG_AUTODETECT) {
        compassConfig()->mag_hardware = detectedSensors[SENSOR_INDEX_MAG];
        eepromUpdatePending = true;
    }

    if (pitotConfig->pitot_hardware == PITOT_AUTODETECT) {
        pitotConfig->pitot_hardware = detectedSensors[SENSOR_INDEX_PITOT];
        eepromUpdatePending = true;
    }

    if (eepromUpdatePending) {
        writeEEPROM();
    }

    return true;
}
