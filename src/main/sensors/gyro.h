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
    GYRO_FAKE
} gyroSensor_e;

extern gyro_t gyro;
extern sensor_align_e gyroAlign;

extern int32_t gyroADC[XYZ_AXIS_COUNT];

typedef struct gyroConfig_s {
    uint8_t gyroMovementCalibrationThreshold;   // people keep forgetting that moving model while init results in wrong gyro offsets. and then they never reset gyro. so this is now on by default.
    uint8_t gyro_lpf;                           // gyro LPF setting - values are driver specific, in case of invalid number, a reasonable default ~30-40HZ is chosen.
    uint8_t gyro_soft_type;                     // Gyro Filter Type
    uint16_t gyro_soft_lpf_hz;                  // Software based gyro filter in hz
    uint16_t gyro_soft_notch_hz;                // Biquad gyro notch hz
    uint16_t gyro_soft_notch_cutoff;            // Biquad gyro notch low cutoff
} gyroConfig_t;

PG_DECLARE(gyroConfig_t, gyroConfig);

void gyroInit(void);
void gyroUpdate(void);
void gyroSetCalibrationCycles(uint16_t calibrationCyclesRequired);
bool isGyroCalibrationComplete(void);

