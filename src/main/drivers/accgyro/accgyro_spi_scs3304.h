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
#include "drivers/exti.h"

// SCS3304 registers (not the complete list)
typedef enum {
    SCS3304_REG_INT1_CTRL = 0x0D,  // int pin 1 control
    SCS3304_REG_INT2_CTRL = 0x0E,  // int pin 2 control
    SCS3304_REG_WHO_AM_I = 0x0F,   // chip ID
    SCS3304_REG_CTRL1_XL = 0x10,   // accelerometer control
    SCS3304_REG_CTRL2_G = 0x11,    // gyro control
    SCS3304_REG_CTRL3_C = 0x12,    // control register 3
    SCS3304_REG_CTRL4_C = 0x13,    // control register 4
    SCS3304_REG_CTRL5_C = 0x14,    // control register 5
    SCS3304_REG_CTRL6_C = 0x15,    // control register 6
    SCS3304_REG_CTRL7_G = 0x16,    // control register 7
    SCS3304_REG_CTRL8_XL = 0x17,   // control register 8
    SCS3304_REG_CTRL9_XL = 0x18,   // control register 9
    SCS3304_REG_CTRL10_C = 0x19,   // control register 10
    SCS3304_REG_STATUS = 0x1E,     // status register
    SCS3304_REG_OUT_TEMP_L = 0x20, // temperature LSB
    SCS3304_REG_OUT_TEMP_H = 0x21, // temperature MSB
    SCS3304_REG_OUTX_L_G = 0x22,   // gyro X axis LSB
    SCS3304_REG_OUTX_H_G = 0x23,   // gyro X axis MSB
    SCS3304_REG_OUTY_L_G = 0x24,   // gyro Y axis LSB
    SCS3304_REG_OUTY_H_G = 0x25,   // gyro Y axis MSB
    SCS3304_REG_OUTZ_L_G = 0x26,   // gyro Z axis LSB
    SCS3304_REG_OUTZ_H_G = 0x27,   // gyro Z axis MSB
    SCS3304_REG_OUTX_L_A = 0x28,   // acc X axis LSB
    SCS3304_REG_OUTX_H_A = 0x29,   // acc X axis MSB
    SCS3304_REG_OUTY_L_A = 0x2A,   // acc Y axis LSB
    SCS3304_REG_OUTY_H_A = 0x2B,   // acc Y axis MSB
    SCS3304_REG_OUTZ_L_A = 0x2C,   // acc Z axis LSB
    SCS3304_REG_OUTZ_H_A = 0x2D,   // acc Z axis MSB
} scs3304Register_e;

// Contained in accgyro_spi_scs3304_init.c which is size-optimized
uint8_t scs3304Detect(const extDevice_t *dev);
bool scs3304SpiAccDetect(accDev_t *acc);
bool scs3304SpiGyroDetect(gyroDev_t *gyro);

// Contained in accgyro_spi_scs3304.c which is speed-optimized
void scs3304ExtiHandler(extiCallbackRec_t *cb);
bool scs3304AccRead(accDev_t *acc);
bool scs3304GyroRead(gyroDev_t *gyro);
