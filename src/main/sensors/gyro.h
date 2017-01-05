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
#include "common/axis.h"

typedef enum {
    GYRO_NONE = 0,
    GYRO_DEFAULT,
    GYRO_MPU6050,
    GYRO_L3G4200D,
    GYRO_MPU3050,
    GYRO_L3GD20,
    GYRO_MPU6000,
    GYRO_MPU6500,
    GYRO_MPU9250,
    GYRO_ICM20689,
    GYRO_ICM20608G,
    GYRO_ICM20602,
    GYRO_FAKE
} gyroSensor_e;

typedef struct gyro_s {
    gyroDev_t dev;
    uint32_t targetLooptime;
    float gyroADCf[XYZ_AXIS_COUNT];
} gyro_t;

extern gyro_t gyro;

typedef struct gyroConfig_s {
    sensor_align_e gyro_align;              // gyro alignment
    uint8_t  gyroMovementCalibrationThreshold; // people keep forgetting that moving model while init results in wrong gyro offsets. and then they never reset gyro. so this is now on by default.
    uint8_t  gyro_sync_denom;                  // Gyro sample divider
    uint8_t  gyro_lpf;                         // gyro LPF setting - values are driver specific, in case of invalid number, a reasonable default ~30-40HZ is chosen.
    uint8_t  gyro_soft_lpf_type;
    uint8_t  gyro_soft_lpf_hz;
    bool     gyro_isr_update;
    bool     gyro_use_32khz;
    uint16_t gyro_soft_notch_hz_1;
    uint16_t gyro_soft_notch_cutoff_1;
    uint16_t gyro_soft_notch_hz_2;
    uint16_t gyro_soft_notch_cutoff_2;
} gyroConfig_t;

void gyroSetCalibrationCycles(void);
bool gyroInit(const gyroConfig_t *gyroConfigToUse);
void gyroInitFilters(void);
void gyroUpdate(void);
bool isGyroCalibrationComplete(void);
