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

#define MPU6500_RA_RATE_DIV                 (0x19)
#define MPU6500_RA_LPF                      (0x1A)
#define MPU6500_RA_GYRO_CFG                 (0x1B)
#define MPU6500_RA_ACCEL_CFG                (0x1C)
#define MPU6500_RA_ACCEL_XOUT_H             (0x3B)
#define MPU6500_RA_INT_PIN_CFG              (0x37)
#define MPU6500_RA_INT_ENABLE               (0x38)
#define MPU6500_RA_GYRO_XOUT_H              (0x43)
#define MPU6500_RA_SIGNAL_PATH_RST          (0x68)
#define MPU6500_RA_USER_CTRL                (0x6A)
#define MPU6500_RA_PWR_MGMT_1               (0x6B)
#define MPU6500_RA_BANK_SEL                 (0x6D)
#define MPU6500_RA_MEM_RW                   (0x6F)
#define MPU6500_RA_WHOAMI                   (0x75)
#define MPU6500_RA_XA_OFFS_H                (0x77)

#define MPU6500_WHO_AM_I_CONST              (0x70)

#define MPU6500_BIT_RESET                   (0x80)

#pragma once

bool mpu6500AccDetect(acc_t *acc);
bool mpu6500GyroDetect(gyro_t *gyro);

void mpu6500AccInit(void);
void mpu6500GyroInit(uint16_t lpf);
