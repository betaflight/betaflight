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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_ACC_LSM303AGR

#include "common/axis.h"
#include "common/utils.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_lsm303agr.h"
#include "drivers/bus_i2c.h"
#include "drivers/time.h"

// 7-bit I2C slave address. ST datasheet uses 8-bit form 0x32 (write) / 0x33 (read).
#define LSM303AGR_ACC_ADDR        0x19

#define LSM303AGR_REG_WHO_AM_I    0x0F
#define LSM303AGR_WHO_AM_I_VAL    0x33

#define LSM303AGR_REG_CTRL_REG1   0x20
#define LSM303AGR_REG_CTRL_REG4   0x23
#define LSM303AGR_REG_OUT_X_L     0x28

// Set MSB of register address to enable auto-increment on multi-byte reads.
#define LSM303AGR_AUTOINC         0x80

// CTRL_REG1: ODR=1.344 kHz (HR/normal), LPen=0 (normal mode), Z/Y/X enable.
//   bits[7:4]=0x9 (1.344 kHz), bit3=LPen=0, bits[2:0]=0x7 (XYZ enabled) -> 0x97
#define LSM303AGR_CTRL_REG1_INIT  0x97

// CTRL_REG4: BDU=1, BLE=0 (LSB at lower addr), FS=00 (+/-2g), HR=1 (high resolution).
//   bit7=BDU=1, bits[5:4]=0 (+/-2g), bit3=HR=1 -> 0x88
#define LSM303AGR_CTRL_REG4_INIT  0x88

#ifndef LSM303AGR_I2C_INSTANCE
#define LSM303AGR_I2C_INSTANCE    MPU_I2C_INSTANCE
#endif

static void lsm303agrAccInit(accDev_t *acc)
{
    i2cWrite(LSM303AGR_I2C_INSTANCE, LSM303AGR_ACC_ADDR,
             LSM303AGR_REG_CTRL_REG1, LSM303AGR_CTRL_REG1_INIT);
    delay(10);
    i2cWrite(LSM303AGR_I2C_INSTANCE, LSM303AGR_ACC_ADDR,
             LSM303AGR_REG_CTRL_REG4, LSM303AGR_CTRL_REG4_INIT);
    delay(10);

    // HR mode at +/-2g produces 12-bit data left-justified in int16. Sensitivity
    // is 1 mg/LSB at the 12-bit level, so 1G = 1000 LSB at 12-bit, which becomes
    // 16000 raw int16 counts after the left-shift by 4.
    acc->acc_1G = 16000;
}

static bool lsm303agrAccRead(accDev_t *acc)
{
    uint8_t buf[6];
    if (!i2cRead(LSM303AGR_I2C_INSTANCE, LSM303AGR_ACC_ADDR,
                 LSM303AGR_REG_OUT_X_L | LSM303AGR_AUTOINC, 6, buf)) {
        return false;
    }
    acc->ADCRaw[X] = (int16_t)((buf[1] << 8) | buf[0]);
    acc->ADCRaw[Y] = (int16_t)((buf[3] << 8) | buf[2]);
    acc->ADCRaw[Z] = (int16_t)((buf[5] << 8) | buf[4]);
    return true;
}

bool lsm303agrAccDetect(accDev_t *acc)
{
    uint8_t whoami = 0;
    if (!i2cRead(LSM303AGR_I2C_INSTANCE, LSM303AGR_ACC_ADDR,
                 LSM303AGR_REG_WHO_AM_I, 1, &whoami)) {
        return false;
    }
    if (whoami != LSM303AGR_WHO_AM_I_VAL) {
        return false;
    }

    acc->initFn = lsm303agrAccInit;
    acc->readFn = lsm303agrAccRead;
    return true;
}

#endif // USE_ACC_LSM303AGR
