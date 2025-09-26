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

#include "drivers/io_types.h"

#ifdef USE_MAG_QMC5883P

// QMC5883P I2C Address
#define QMC5883P_I2C_ADDRESS           0x2C

// QMC5883P Register Addresses
#define QMC5883P_REG_ID                0x00
#define QMC5883P_REG_DATA_OUTPUT_X     0x01
#define QMC5883P_REG_DATA_OUTPUT_X_MSB 0x02
#define QMC5883P_REG_DATA_OUTPUT_Y     0x03
#define QMC5883P_REG_DATA_OUTPUT_Y_MSB 0x04
#define QMC5883P_REG_DATA_OUTPUT_Z     0x05
#define QMC5883P_REG_DATA_OUTPUT_Z_MSB 0x06
#define QMC5883P_REG_STATUS            0x09
#define QMC5883P_REG_CONF1             0x0A
#define QMC5883P_REG_CONF2             0x0B

// QMC5883P Chip ID
#define QMC5883P_ID_VAL                0x80

// QMC5883P Configuration Values
// Mode settings for CONF1 register
#define QMC5883P_MODE_STANDBY          0x00
#define QMC5883P_MODE_CONTINUOUS       0x03

// Output Data Rate (ODR) settings for CONF1 register
#define QMC5883P_ODR_10HZ              (0x00 << 2)
#define QMC5883P_ODR_50HZ              (0x01 << 2)
#define QMC5883P_ODR_100HZ             (0x02 << 2)
#define QMC5883P_ODR_200HZ             (0x03 << 2)

// Range settings for CONF1 register
#define QMC5883P_RNG_2G                (0x00 << 4)
#define QMC5883P_RNG_8G                (0x01 << 4)

// Oversampling Ratio 1 (OSR1) settings for CONF1 register
#define QMC5883P_OSR1_8                (0x00 << 6)
#define QMC5883P_OSR1_4                (0x01 << 6)
#define QMC5883P_OSR1_2                (0x02 << 6)
#define QMC5883P_OSR1_1                (0x03 << 6)

// Oversampling Ratio 2 (OSR2) settings for CONF2 register
#define QMC5883P_OSR2_8                0x08
#define QMC5883P_OSR2_4                0x04
#define QMC5883P_OSR2_2                0x02
#define QMC5883P_OSR2_1                0x01

// Status register bits
#define QMC5883P_STATUS_DATA_READY     0x01
#define QMC5883P_STATUS_DATA_OVERRUN   0x02

// Special configuration registers and values
#define QMC5883P_REG_XYZ_UNLOCK        0x29
#define QMC5883P_XYZ_SIGN_CONFIG       0x06

// Unlock register for data overrun recovery (uses Z_MSB register)
#define QMC5883P_REG_DATA_UNLOCK       QMC5883P_REG_DATA_OUTPUT_Z_MSB

// Default configuration for Betaflight
#define QMC5883P_DEFAULT_CONF1         (QMC5883P_MODE_CONTINUOUS | QMC5883P_ODR_100HZ | QMC5883P_RNG_8G | QMC5883P_OSR1_8)
#define QMC5883P_DEFAULT_CONF2         QMC5883P_OSR2_8

// Function declarations
bool qmc5883pDetect(magDev_t *magDev);

#endif