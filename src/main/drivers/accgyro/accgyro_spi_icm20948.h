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

#pragma once

#include "drivers/bus.h"

// Sensitivity for ICM20948 (Reciprocal of datasheet LSB/DPS values)
#define ICM20948_GYRO_SCALE_250DPS  (1.0f / 131.0f)
#define ICM20948_GYRO_SCALE_500DPS  (1.0f / 65.5f)
#define ICM20948_GYRO_SCALE_1000DPS (1.0f / 32.8f)
#define ICM20948_GYRO_SCALE_2000DPS (1.0f / 16.4f)

#define ICM20948_BIT_RESET             0x80
#define ICM20948_RA_REG_BANK_SEL       0x7F

// BANK 0
#define ICM20948_RA_WHO_AM_I           0x00
#define ICM20948_RA_LP_CONFIG          0x05
#define ICM20948_RA_PWR_MGMT_1         0x06
#define ICM20948_RA_PWR_MGMT_2         0x07
#define ICM20948_RA_INT_PIN_CFG        0x0F
#define ICM20948_RA_INT_ENABLE_1       0x11
#define ICM20948_RA_GYRO_XOUT_H        0x33
#define ICM20948_RA_ACCEL_XOUT_H       0x2D

// BANK 2
#define ICM20948_RA_GYRO_SMPLRT_DIV    0x00
#define ICM20948_RA_GYRO_CONFIG_1      0x01
#define ICM20948_RA_ACCEL_CONFIG       0x14

enum icm20948_gyro_fsr_e {
    ICM20948_FSR_250DPS = 0,
    ICM20948_FSR_500DPS,
    ICM20948_FSR_1000DPS,
    ICM20948_FSR_2000DPS,
    NUM_ICM20948_GYRO_FSR
};

enum icm20948_accel_fsr_e {
    ICM20948_FSR_2G = 0,
    ICM20948_FSR_4G,
    ICM20948_FSR_8G,
    ICM20948_FSR_16G,
    NUM_ICM20948_ACCEL_FSR
};

void icm20948AccInit(accDev_t *acc);
void icm20948GyroInit(gyroDev_t *gyro);

uint8_t icm20948SpiDetect(const extDevice_t *dev);

bool icm20948SpiAccDetect(accDev_t *acc);
bool icm20948SpiGyroDetect(gyroDev_t *gyro);

bool icm20948GyroReadSPI(gyroDev_t *gyro);
bool icm20948AccRead(accDev_t *acc);
