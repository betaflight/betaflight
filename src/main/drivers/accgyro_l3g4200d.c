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

#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#include "system.h"
#include "bus_i2c.h"

#include "common/maths.h"
#include "common/axis.h"

#include "sensor.h"
#include "accgyro.h"
#include "accgyro_l3g4200d.h"

// L3G4200D, Standard address 0x68
#define L3G4200D_ADDRESS         0x68
#define L3G4200D_ID              0xD3
#define L3G4200D_AUTOINCR        0x80

// Registers
#define L3G4200D_WHO_AM_I        0x0F
#define L3G4200D_CTRL_REG1       0x20
#define L3G4200D_CTRL_REG2       0x21
#define L3G4200D_CTRL_REG3       0x22
#define L3G4200D_CTRL_REG4       0x23
#define L3G4200D_CTRL_REG5       0x24
#define L3G4200D_REFERENCE       0x25
#define L3G4200D_STATUS_REG      0x27
#define L3G4200D_GYRO_OUT        0x28

// Bits
#define L3G4200D_POWER_ON        0x0F
#define L3G4200D_FS_SEL_2000DPS  0xF0
#define L3G4200D_DLPF_32HZ       0x00
#define L3G4200D_DLPF_54HZ       0x40
#define L3G4200D_DLPF_78HZ       0x80
#define L3G4200D_DLPF_93HZ       0xC0

static void l3g4200dInit(uint16_t lpf);
static bool l3g4200dRead(int16_t *gyroADC);

bool l3g4200dDetect(gyro_t *gyro)
{
    uint8_t deviceid;

    delay(25);

    i2cRead(L3G4200D_ADDRESS, L3G4200D_WHO_AM_I, 1, &deviceid);
    if (deviceid != L3G4200D_ID)
        return false;

    gyro->init = l3g4200dInit;
    gyro->read = l3g4200dRead;

    // 14.2857dps/lsb scalefactor
    gyro->scale = 1.0f / 14.2857f;

    return true;
}

static void l3g4200dInit(uint16_t lpf)
{
    bool ack;

    uint8_t mpuLowPassFilter = L3G4200D_DLPF_32HZ;

    switch (lpf) {
        default:
            case 32:
            mpuLowPassFilter = L3G4200D_DLPF_32HZ;
            break;
        case 54:
            mpuLowPassFilter = L3G4200D_DLPF_54HZ;
            break;
        case 78:
            mpuLowPassFilter = L3G4200D_DLPF_78HZ;
            break;
        case 93:
            mpuLowPassFilter = L3G4200D_DLPF_93HZ;
            break;
    }

    delay(100);

    ack = i2cWrite(L3G4200D_ADDRESS, L3G4200D_CTRL_REG4, L3G4200D_FS_SEL_2000DPS);
    if (!ack)
        failureMode(FAILURE_ACC_INIT);

    delay(5);
    i2cWrite(L3G4200D_ADDRESS, L3G4200D_CTRL_REG1, L3G4200D_POWER_ON | mpuLowPassFilter);
}

// Read 3 gyro values into user-provided buffer. No overrun checking is done.
static bool l3g4200dRead(int16_t *gyroADC)
{
    uint8_t buf[6];

    if (!i2cRead(L3G4200D_ADDRESS, L3G4200D_AUTOINCR | L3G4200D_GYRO_OUT, 6, buf)) {
        return false;
    }

    gyroADC[X] = (int16_t)((buf[0] << 8) | buf[1]);
    gyroADC[Y] = (int16_t)((buf[2] << 8) | buf[3]);
    gyroADC[Z] = (int16_t)((buf[4] << 8) | buf[5]);

    return true;
}
