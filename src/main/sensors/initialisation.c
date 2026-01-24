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
#include "sensors/opticalflow.h"

uint8_t detectedSensors[SENSOR_INDEX_COUNT] = { GYRO_NONE, ACC_NONE, BARO_NONE, MAG_NONE, RANGEFINDER_NONE, OPTICALFLOW_NONE};

// sync with baroSensor_e
const char * const baroSensorNames[BARO_HARDWARE_COUNT] = {
    [BARO_DEFAULT] = "AUTO",
    [BARO_NONE] = "NONE",
    [BARO_BMP085] = "BMP085",
    [BARO_MS5611] = "MS5611",
    [BARO_BMP280] = "BMP280",
    [BARO_LPS] = "LPS",
    [BARO_QMP6988] = "QMP6988",
    [BARO_BMP388] = "BMP388",
    [BARO_DPS310] = "DPS310",
    [BARO_2SMPB_02B] = "2SMPB_02B",
    [BARO_LPS22DF] = "LPS22DF",
    [BARO_VIRTUAL] = "VIRTUAL"
};

// sync with magSensor_e
const char * const magSensorNames[MAG_HARDWARE_COUNT] = {
    [MAG_DEFAULT] = "AUTO",
    [MAG_NONE] = "NONE",
    [MAG_HMC5883] = "HMC5883",
    [MAG_AK8975] = "AK8975",
    [MAG_AK8963] = "AK8963",
    [MAG_QMC5883] = "QMC5883",
    [MAG_LIS2MDL] = "LIS2MDL",
    [MAG_LIS3MDL] = "LIS3MDL",
    [MAG_MPU925X_AK8963] = "MPU925X_AK8963",
    [MAG_IST8310] = "IST8310"
};

// sync with rangefinderType_e
const char * const rangefinderTypeNames[RANGEFINDER_HARDWARE_COUNT] = {
    [RANGEFINDER_NONE] = "NONE",
    [RANGEFINDER_HCSR04] = "HCSR04",
    [RANGEFINDER_TFMINI] = "TFMINI",
    [RANGEFINDER_TF02] = "TF02",
    [RANGEFINDER_MTF01] = "MTF01",
    [RANGEFINDER_MTF02] = "MTF02",
    [RANGEFINDER_MTF01P] = "MTF01P",
    [RANGEFINDER_MTF02P] = "MTF02P",
    [RANGEFINDER_TFNOVA] = "TFNOVA"
};

// sync with opticalflowType_e
const char * const opticalflowTypeNames[OPTICALFLOW_HARDWARE_COUNT] = {
    [OPTICALFLOW_NONE] = "NONE",
    [OPTICALFLOW_MT] = "MT"
};
uint8_t detectedGyros[GYRO_COUNT];

void sensorsPreInit(void)
{
    memset(detectedGyros, 0, sizeof(detectedGyros));

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

#ifdef USE_BARO
    baroInit();
#endif

#ifdef USE_MAG
    compassInit();
#endif

#ifdef USE_RANGEFINDER
    rangefinderInit();
#endif

#ifdef USE_OPTICALFLOW
    opticalflowInit();
#endif

#ifdef USE_ADC_INTERNAL
    adcInternalInit();
#endif

    return gyroDetected;
}
