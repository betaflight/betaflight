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

#include "platform.h"
#include "common/axis.h"
#include "drivers/exti.h"
#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro_mpu.h"

#ifndef MPU_I2C_INSTANCE
#define MPU_I2C_INSTANCE I2C_DEVICE
#endif

#define GYRO_LPF_256HZ      0
#define GYRO_LPF_188HZ      1
#define GYRO_LPF_98HZ       2
#define GYRO_LPF_42HZ       3
#define GYRO_LPF_20HZ       4
#define GYRO_LPF_10HZ       5
#define GYRO_LPF_5HZ        6
#define GYRO_LPF_NONE       7

typedef enum {
    GYRO_RATE_1_kHz,
    GYRO_RATE_8_kHz,
    GYRO_RATE_32_kHz,
} gyroRateKHz_e;

typedef struct gyroDev_s {
    sensorGyroInitFuncPtr initFn;                       // initialize function
    sensorGyroReadFuncPtr readFn;                       // read 3 axis data function
    sensorGyroReadDataFuncPtr temperatureFn;            // read temperature if available
    sensorGyroInterruptStatusFuncPtr intStatusFn;
    sensorGyroUpdateFuncPtr updateFn;
    extiCallbackRec_t exti;
    busDevice_t bus;
    float scale;                                        // scalefactor
    int16_t gyroADCRaw[XYZ_AXIS_COUNT];
    int16_t gyroZero[XYZ_AXIS_COUNT];
    uint8_t lpf;
    gyroRateKHz_e gyroRateKHz;
    uint8_t mpuDividerDrops;
    volatile bool dataReady;
    sensor_align_e gyroAlign;
    mpuDetectionResult_t mpuDetectionResult;
    const extiConfig_t *mpuIntExtiConfig;
    mpuConfiguration_t mpuConfiguration;
} gyroDev_t;

typedef struct accDev_s {
    sensorAccInitFuncPtr initFn;                        // initialize function
    sensorAccReadFuncPtr readFn;                        // read 3 axis data function
    busDevice_t bus;
    uint16_t acc_1G;
    int16_t ADCRaw[XYZ_AXIS_COUNT];
    char revisionCode;                                  // a revision code for the sensor, if known
    sensor_align_e accAlign;
    mpuDetectionResult_t mpuDetectionResult;
    mpuConfiguration_t mpuConfiguration;
} accDev_t;
