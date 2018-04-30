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

#pragma once

#include "drivers/bus.h"

#define ICM20649_BIT_RESET                  (0x80)

#define ICM20649_RA_REG_BANK_SEL       0x7F

// BANK 0
#define ICM20649_RA_WHO_AM_I           0x00
#define ICM20649_RA_PWR_MGMT_1         0x06
#define ICM20649_RA_PWR_MGMT_2         0x07
#define ICM20649_RA_INT_PIN_CFG        0x0F
#define ICM20649_RA_INT_ENABLE_1       0x11
#define ICM20649_RA_GYRO_XOUT_H        0x33
#define ICM20649_RA_ACCEL_XOUT_H       0x2D

// BANK 2
#define ICM20649_RA_GYRO_SMPLRT_DIV    0x00
#define ICM20649_RA_GYRO_CONFIG_1      0x01
#define ICM20649_RA_ACCEL_CONFIG       0x14

enum icm20649_gyro_fsr_e {
    ICM20649_FSR_500DPS = 0,
    ICM20649_FSR_1000DPS,
    ICM20649_FSR_2000DPS,
    ICM20649_FSR_4000DPS,
    NUM_ICM20649_GYRO_FSR
};

enum icm20649_accel_fsr_e {
    ICM20649_FSR_4G = 0,
    ICM20649_FSR_8G,
    ICM20649_FSR_16G,
    ICM20649_FSR_30G,
    NUM_ICM20649_ACCEL_FSR
};

void icm20649AccInit(accDev_t *acc);
void icm20649GyroInit(gyroDev_t *gyro);

uint8_t icm20649SpiDetect(const busDevice_t *bus);

bool icm20649SpiAccDetect(accDev_t *acc);
bool icm20649SpiGyroDetect(gyroDev_t *gyro);

bool icm20649GyroReadSPI(gyroDev_t *gyro);
bool icm20649AccRead(accDev_t *acc);
