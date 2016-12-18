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

#pragma once

#include "common/axis.h"
#include "drivers/accgyro.h"
#include "sensors/sensors.h"

// Type of accelerometer used/detected
typedef enum {
    ACC_NONE = 0,
    ACC_AUTODETECT = 1,
    ACC_ADXL345 = 2,
    ACC_MPU6050 = 3,
    ACC_MMA8452 = 4,
    ACC_BMA280 = 5,
    ACC_LSM303DLHC = 6,
    ACC_MPU6000 = 7,
    ACC_MPU6500 = 8,
    ACC_FAKE = 9,
    ACC_MAX = ACC_FAKE
} accelerationSensor_e;

typedef struct acc_s {
    accDev_t dev;
    uint32_t accTargetLooptime;
    int32_t accADC[XYZ_AXIS_COUNT];
} acc_t;

extern acc_t acc;

typedef struct accelerometerConfig_s {
    sensor_align_e acc_align;               // acc alignment
    uint8_t acc_hardware;                   // Which acc hardware to use on boards with more than one device
    flightDynamicsTrims_t accZero;          // Accelerometer offset
    flightDynamicsTrims_t accGain;          // Accelerometer gain to read exactly 1G
} accelerometerConfig_t;

bool accInit(const accelerometerConfig_t *accConfig, uint32_t accTargetLooptime);
bool isAccelerationCalibrationComplete(void);
void accSetCalibrationCycles(uint16_t calibrationCyclesRequired);
void updateAccelerationReadings(void);
union flightDynamicsTrims_u;
void setAccelerationCalibrationValues(union flightDynamicsTrims_u * accZeroToUse, union flightDynamicsTrims_u * accGainToUse);
void setAccelerationFilter(uint8_t initialAccLpfCutHz);
bool isAccelerometerHealthy(void);
