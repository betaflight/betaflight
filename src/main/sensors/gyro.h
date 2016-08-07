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
    GYRO_FAKE,
    GYRO_MAX = GYRO_FAKE
} gyroSensor_e;

extern gyro_t gyro;

extern int32_t gyroADC[XYZ_AXIS_COUNT];
extern float gyroADCf[XYZ_AXIS_COUNT];

typedef struct gyroConfig_s {
    uint8_t gyroMovementCalibrationThreshold; // people keep forgetting that moving model while init results in wrong gyro offsets. and then they never reset gyro. so this is now on by default.
} gyroConfig_t;

void gyroUseConfig(const gyroConfig_t *gyroConfigToUse, uint8_t gyro_soft_lpf_hz, uint16_t gyro_soft_notch_hz, uint16_t gyro_soft_notch_cutoff, uint8_t gyro_soft_lpf_type);
void gyroSetCalibrationCycles(void);
void gyroInit(void);
void gyroUpdate(void);
bool isGyroCalibrationComplete(void);

