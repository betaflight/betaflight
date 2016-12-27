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

#include "drivers/accgyro.h"
#include "sensors/sensors.h"

// Type of accelerometer used/detected
typedef enum {
    ACC_DEFAULT = 0,
    ACC_NONE = 1,
    ACC_ADXL345 = 2,
    ACC_MPU6050 = 3,
    ACC_MMA8452 = 4,
    ACC_BMA280 = 5,
    ACC_LSM303DLHC = 6,
    ACC_MPU6000 = 7,
    ACC_MPU6500 = 8,
    ACC_ICM20689 = 9,
    ACC_FAKE = 10
} accelerationSensor_e;

typedef struct acc_s {
    accDev_t dev;
    uint32_t accSamplingInterval;
    int32_t accSmooth[XYZ_AXIS_COUNT];
    bool isAccelUpdatedAtLeastOnce;
} acc_t;

extern acc_t acc;

typedef struct rollAndPitchTrims_s {
    int16_t roll;
    int16_t pitch;
} rollAndPitchTrims_t_def;

typedef union rollAndPitchTrims_u {
    int16_t raw[2];
    rollAndPitchTrims_t_def values;
} rollAndPitchTrims_t;


typedef struct accelerometerConfig_s {
    uint16_t acc_lpf_hz;                    // cutoff frequency for the low pass filter used on the acc z-axis for althold in Hz
    sensor_align_e acc_align;               // acc alignment
    uint8_t acc_hardware;                   // Which acc hardware to use on boards with more than one device
    flightDynamicsTrims_t accZero;
    rollAndPitchTrims_t accelerometerTrims;
} accelerometerConfig_t;

bool accInit(const accelerometerConfig_t *accelerometerConfig, uint32_t gyroTargetLooptime);
bool isAccelerationCalibrationComplete(void);
void accSetCalibrationCycles(uint16_t calibrationCyclesRequired);
void resetRollAndPitchTrims(rollAndPitchTrims_t *rollAndPitchTrims);
void accUpdate(rollAndPitchTrims_t *rollAndPitchTrims);
union flightDynamicsTrims_u;
void setAccelerationTrims(union flightDynamicsTrims_u *accelerationTrimsToUse);
void setAccelerationFilter(uint16_t initialAccLpfCutHz);

