/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <strings.h>

#include "platform.h"

#include "common/utils.h"

#include "drivers/accgyro/accgyro.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/rangefinder.h"
#include "sensors/opticalflow.h"
#include "sensors/sensors.h"

// Sensor names (used in lookup tables for *_hardware settings and in status command output)
// sync with gyroHardware_e
const char * const lookupTableGyroHardware[GYRO_HARDWARE_COUNT] = {
    [GYRO_NONE] = "NONE",
    [GYRO_DEFAULT] = "AUTO",
    [GYRO_MPU6050] = "MPU6050",
    [GYRO_L3GD20] = "L3GD20",
    [GYRO_MPU6000] = "MPU6000",
    [GYRO_MPU6500] = "MPU6500",
    [GYRO_MPU9250] = "MPU9250",
    [GYRO_ICM20601] = "ICM20601",
    [GYRO_ICM20602] = "ICM20602",
    [GYRO_ICM20608G] = "ICM20608G",
    [GYRO_ICM20649] = "ICM20649",
    [GYRO_ICM20689] = "ICM20689",
    [GYRO_ICM42605] = "ICM42605",
    [GYRO_ICM42688P] = "ICM42688P",
    [GYRO_BMI160] = "BMI160",
    [GYRO_BMI270] = "BMI270",
    [GYRO_LSM6DSO] = "LSM6DSO",
    [GYRO_LSM6DSV16X] = "LSM6DSV16X",
    [GYRO_IIM42653] = "IIM42653",
    [GYRO_ICM45605] = "ICM45605",
    [GYRO_ICM45686] = "ICM45686",
    [GYRO_ICM40609D] = "ICM40609D",
    [GYRO_IIM42652] = "IIM42652",
    [GYRO_LSM6DSK320X] = "LSM6DSK320X",
    [GYRO_ICM42622P] = "ICM42622P",
    [GYRO_ICM42686P] = "ICM42686P",
    [GYRO_VIRTUAL] = "VIRTUAL"
};

// sync with accelerationSensor_e
const char * const lookupTableAccHardware[ACC_HARDWARE_COUNT] = {
    [ACC_DEFAULT] = "AUTO",
    [ACC_NONE] = "NONE",
    [ACC_MPU6050] = "MPU6050",
    [ACC_MPU6000] = "MPU6000",
    [ACC_MPU6500] = "MPU6500",
    [ACC_MPU9250] = "MPU9250",
    [ACC_ICM20601] = "ICM20601",
    [ACC_ICM20602] = "ICM20602",
    [ACC_ICM20608G] = "ICM20608G",
    [ACC_ICM20649] = "ICM20649",
    [ACC_ICM20689] = "ICM20689",
    [ACC_ICM42605] = "ICM42605",
    [ACC_ICM42688P] = "ICM42688P",
    [ACC_BMI160] = "BMI160",
    [ACC_BMI270] = "BMI270",
    [ACC_LSM6DSO] = "LSM6DSO",
    [ACC_LSM6DSV16X] = "LSM6DSV16X",
    [ACC_IIM42653] = "IIM42653",
    [ACC_ICM45605] = "ICM45605",
    [ACC_ICM45686] = "ICM45686",
    [ACC_ICM40609D] = "ICM40609D",
    [ACC_IIM42652] = "IIM42652",
    [ACC_LSM6DSK320X] = "LSM6DSK320X",
    [ACC_ICM42622P] = "ICM42622P",
    [ACC_ICM42686P] = "ICM42686P",
    [ACC_VIRTUAL] = "VIRTUAL"
};

// sync with baroSensor_e
const char * const lookupTableBaroHardware[BARO_HARDWARE_COUNT] = {
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
    [BARO_BMP580] = "BMP580",
    [BARO_BMP581] = "BMP581",
    [BARO_VIRTUAL] = "VIRTUAL"
};

// sync with magSensor_e
const char * const lookupTableMagHardware[MAG_HARDWARE_COUNT] = {
    [MAG_DEFAULT] = "AUTO",
    [MAG_NONE] = "NONE",
    [MAG_HMC5883] = "HMC5883",
    [MAG_AK8975] = "AK8975",
    [MAG_AK8963] = "AK8963",
    [MAG_QMC5883] = "QMC5883",
    [MAG_LIS2MDL] = "LIS2MDL",
    [MAG_LIS3MDL] = "LIS3MDL",
    [MAG_MPU925X_AK8963] = "MPU925X_AK8963",
    [MAG_IST8310] = "IST8310",
    [MAG_MMC560X] = "MMC560X"
};

// sync with rangefinderType_e
const char * const lookupTableRangefinderHardware[RANGEFINDER_HARDWARE_COUNT] = {
    [RANGEFINDER_NONE] = "NONE",
    [RANGEFINDER_HCSR04] = "HCSR04",
    [RANGEFINDER_TFMINI] = "TFMINI",
    [RANGEFINDER_TF02] = "TF02",
    [RANGEFINDER_MTF01] = "MTF01",
    [RANGEFINDER_MTF02] = "MTF02",
    [RANGEFINDER_MTF01P] = "MTF01P",
    [RANGEFINDER_MTF02P] = "MTF02P",
    [RANGEFINDER_TFNOVA] = "TFNOVA",
    [RANGEFINDER_NOOPLOOP_F2] = "NOOPLOOP_F2",
    [RANGEFINDER_NOOPLOOP_F2P] = "NOOPLOOP_F2P",
    [RANGEFINDER_NOOPLOOP_F2PH] = "NOOPLOOP_F2PH",
    [RANGEFINDER_NOOPLOOP_F] = "NOOPLOOP_F",
    [RANGEFINDER_NOOPLOOP_FP] = "NOOPLOOP_FP",
    [RANGEFINDER_NOOPLOOP_F2MINI] = "NOOPLOOP_F2MINI",
    [RANGEFINDER_UPT1] = "UPT1"
};

// sync with opticalflowType_e
const char * const lookupTableOpticalflowHardware[OPTICALFLOW_HARDWARE_COUNT] = {
    [OPTICALFLOW_NONE] = "NONE",
    [OPTICALFLOW_MT] = "MT",
    [OPTICALFLOW_UPT1] = "UPT1"
};

static const char * const sensorTypeNames[] = {
    [SENSOR_INDEX_GYRO] = "gyro",
    [SENSOR_INDEX_ACC] = "acc",
    [SENSOR_INDEX_BARO] = "baro",
    [SENSOR_INDEX_MAG] = "mag",
    [SENSOR_INDEX_RANGEFINDER] = "rangefinder",
    [SENSOR_INDEX_OPTICALFLOW] = "opticalflow",
};
STATIC_ASSERT(SENSOR_INDEX_COUNT == ARRAYLEN(sensorTypeNames), sensorTypeNames_length_mismatch);

static const struct {
    const char * const *names;
    uint8_t count;
} sensorHardwareTables[] = {
    [SENSOR_INDEX_GYRO]         = { lookupTableGyroHardware,         GYRO_HARDWARE_COUNT },
    [SENSOR_INDEX_ACC]          = { lookupTableAccHardware,          ACC_HARDWARE_COUNT },
    [SENSOR_INDEX_BARO]         = { lookupTableBaroHardware,         BARO_HARDWARE_COUNT },
    [SENSOR_INDEX_MAG]          = { lookupTableMagHardware,          MAG_HARDWARE_COUNT },
    [SENSOR_INDEX_RANGEFINDER]  = { lookupTableRangefinderHardware,  RANGEFINDER_HARDWARE_COUNT },
    [SENSOR_INDEX_OPTICALFLOW]  = { lookupTableOpticalflowHardware,  OPTICALFLOW_HARDWARE_COUNT },
};
STATIC_ASSERT(SENSOR_INDEX_COUNT == ARRAYLEN(sensorHardwareTables), sensorHardwareTables_length_mismatch);

const char * const *sensorHardwareNames(sensorIndex_e sensor, int *count)
{
    if (sensor >= SENSOR_INDEX_COUNT) {
        return NULL;
    }
    if (count) {
        *count = sensorHardwareTables[sensor].count;
    }
    return sensorHardwareTables[sensor].names;
}

sensorIndex_e sensorIndexFromName(const char *name)
{
    for (unsigned i = 0; i < SENSOR_INDEX_COUNT; i++) {
        if (strcasecmp(name, sensorTypeNames[i]) == 0) {
            return i;
        }
    }
    return SENSOR_INDEX_COUNT;
}

const char *sensorTypeName(sensorIndex_e sensor)
{
    if (sensor >= SENSOR_INDEX_COUNT) {
        return NULL;
    }
    return sensorTypeNames[sensor];
}
