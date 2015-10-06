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

typedef bool (*mpuReadRegisterFunc)(uint8_t reg, uint8_t length, uint8_t* data);
typedef bool (*mpuWriteRegisterFunc)(uint8_t reg, uint8_t data);

typedef struct mpuConfiguration_s {
    uint8_t gyroReadXRegister; // Y and Z must registers follow this, 2 words each
    mpuReadRegisterFunc read;
    mpuWriteRegisterFunc write;
} mpuConfiguration_t;

extern mpuConfiguration_t mpuConfiguration;

enum lpf_e {
    INV_FILTER_256HZ_NOLPF2 = 0,
    INV_FILTER_188HZ,
    INV_FILTER_98HZ,
    INV_FILTER_42HZ,
    INV_FILTER_20HZ,
    INV_FILTER_10HZ,
    INV_FILTER_5HZ,
    INV_FILTER_2100HZ_NOLPF,
    NUM_FILTER
};
enum gyro_fsr_e {
    INV_FSR_250DPS = 0,
    INV_FSR_500DPS,
    INV_FSR_1000DPS,
    INV_FSR_2000DPS,
    NUM_GYRO_FSR
};
enum clock_sel_e {
    INV_CLK_INTERNAL = 0,
    INV_CLK_PLL,
    NUM_CLK
};
enum accel_fsr_e {
    INV_FSR_2G = 0,
    INV_FSR_4G,
    INV_FSR_8G,
    INV_FSR_16G,
    NUM_ACCEL_FSR
};

typedef enum {
    MPU_NONE,
    MPU_3050,
    MPU_60x0,
    MPU_60x0_SPI,
    MPU_65xx_I2C,
    MPU_65xx_SPI
} detectedMPUSensor_e;

typedef enum {
    MPU_HALF_RESOLUTION,
    MPU_FULL_RESOLUTION
} mpu6050Resolution_e;

typedef struct mpuDetectionResult_s {
    detectedMPUSensor_e sensor;
    mpu6050Resolution_e resolution;
} mpuDetectionResult_t;

extern mpuDetectionResult_t mpuDetectionResult;

uint8_t determineMPULPF(uint16_t lpf);
void configureMPUDataReadyInterruptHandling(void);
void mpuIntExtiInit(void);
bool mpuAccRead(int16_t *accData);
bool mpuGyroRead(int16_t *gyroADC);
mpuDetectionResult_t *detectMpu(const extiConfig_t *configToUse);
