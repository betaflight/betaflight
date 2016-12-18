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

#include "config/config.h"
#include "config/config_eeprom.h"

#include "drivers/logging.h"

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


bool sensorsAutodetect(const gyroConfig_t *gyroConfig,
                accelerometerConfig_t *accConfig,
                compassConfig_t *compassConfig,
                barometerConfig_t *baroConfig,
                pitotmeterConfig_t *pitotConfig)
{
    bool eepromUpdatePending = false;

    if (!gyroInit(gyroConfig)) {
        return false;
    }

#ifdef ASYNC_GYRO_PROCESSING
     // ACC will be updated at its own rate
    accInit(accConfig, getAccUpdateRate());
#else
    // acc updated at same frequency in taskMainPidLoop in mw.c
    accInit(accConfig, gyro.targetLooptime);
#endif

#ifdef BARO
    baroDetect(&baro.dev, baroConfig->baro_hardware);
#else
    UNUSED(baroConfig);
#endif

#ifdef PITOT
    pitotDetect(&pitot.dev, pitotConfig->pitot_hardware);
#else
    UNUSED(pitotConfig);
#endif

    // FIXME extract to a method to reduce dependencies, maybe move to sensors_compass.c
    mag.magneticDeclination = 0.0f; // TODO investigate if this is actually needed if there is no mag sensor or if the value stored in the config should be used.
#ifdef MAG
    if (compassDetect(&mag.dev, compassConfig->mag_hardware)) {
        // calculate magnetic declination
        if (!compassInit(compassConfig)) {
            addBootlogEvent2(BOOT_EVENT_MAG_INIT_FAILED, BOOT_EVENT_FLAGS_ERROR);
            sensorsClear(SENSOR_MAG);
        }
    }
#else
    UNUSED(compassConfig);
#endif

#ifdef SONAR
    const rangefinderType_e rangefinderType = rangefinderDetect();
    rangefinderInit(rangefinderType);
#endif
    if (gyroConfig->gyro_align != ALIGN_DEFAULT) {
        gyro.dev.gyroAlign = gyroConfig->gyro_align;
    }
    if (accConfig->acc_align != ALIGN_DEFAULT) {
        acc.dev.accAlign = accConfig->acc_align;
    }
    if (compassConfig->mag_align != ALIGN_DEFAULT) {
        mag.dev.magAlign = compassConfig->mag_align;
    }

    /* Check if sensor autodetection was requested for some sensors and */
    if (accConfig->acc_hardware == ACC_AUTODETECT) {
        accConfig->acc_hardware = detectedSensors[SENSOR_INDEX_ACC];
        eepromUpdatePending = true;
    }

    if (baroConfig->baro_hardware == BARO_AUTODETECT) {
        baroConfig->baro_hardware = detectedSensors[SENSOR_INDEX_BARO];
        eepromUpdatePending = true;
    }

    if (compassConfig->mag_hardware == MAG_AUTODETECT) {
        compassConfig->mag_hardware = detectedSensors[SENSOR_INDEX_MAG];
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
