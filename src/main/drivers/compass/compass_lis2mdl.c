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

#include <stdbool.h>
#include <stdint.h>

#include <math.h>

#include "platform.h"

#if defined(USE_MAG_LIS2MDL)

#include "compass.h"
#include "drivers/time.h"
#include "common/axis.h"

#define LIS2MDL_MAG_I2C_ADDRESS     0x1E

// Macros to encode/decode multi-bit values
#define LIS2MDL_ENCODE_BITS(val, mask, shift)   ((val << shift) & mask)
#define LIS2MDL_DECODE_BITS(val, mask, shift)   ((val & mask) >> shift)

#define LIS2MDL_OFFSET_X_REG_L      0x45
#define LIS2MDL_OFFSET_X_REG_H      0x46
#define LIS2MDL_OFFSET_Y_REG_L      0x47
#define LIS2MDL_OFFSET_Y_REG_H      0x48
#define LIS2MDL_OFFSET_Z_REG_L      0x49
#define LIS2MDL_OFFSET_Z_REG_H      0x4A

#define LIS2MDL_REG_WHO_AM_I        0x4F
#define LIS2MDL_DEVICE_ID                   0x40

#define LIS2MDL_CFG_REG_A           0x60
#define LIS2MDL_CFG_REG_A_COMP_TEMP_EN      0x80
#define LIS2MDL_CFG_REG_A_REBOOT            0x40
#define LIS2MDL_CFG_REG_A_SOFT_RST          0x20
#define LIS2MDL_CFG_REG_A_LP                0x10
#define LIS2MDL_CFG_REG_A_ODR_MASK          0x0c
#define LIS2MDL_CFG_REG_A_ODR_SHIFT         2
#define LIS2MDL_CFG_REG_A_ODR_10            0
#define LIS2MDL_CFG_REG_A_ODR_20            1
#define LIS2MDL_CFG_REG_A_ODR_50            2
#define LIS2MDL_CFG_REG_A_ODR_100           3
#define LIS2MDL_CFG_REG_A_MD_MASK           0x03
#define LIS2MDL_CFG_REG_A_MD_SHIFT          0
#define LIS2MDL_CFG_REG_A_MD_CONT           0
#define LIS2MDL_CFG_REG_A_MD_SINGLE         1
#define LIS2MDL_CFG_REG_A_MD_IDLE           3

#define LIS2MDL_CFG_REG_B           0x61
#define LIS2MDL_CFG_REG_B_OFF_CANC_ONE_SHOT 0x10
#define LIS2MDL_CFG_REG_B_INT_ON_DATA_OFF   0x08
#define LIS2MDL_CFG_REG_B_SET_FREQ          0x04
#define LIS2MDL_CFG_REG_B_OFF_CANC          0x02
#define LIS2MDL_CFG_REG_B_LPF               0x01

#define LIS2MDL_CFG_REG_C           0x62
#define LIS2MDL_CFG_REG_C_INT_ON_PIN        0x40
#define LIS2MDL_CFG_REG_C_I2C_DIS           0x20
#define LIS2MDL_CFG_REG_C_BDU               0x10
#define LIS2MDL_CFG_REG_C_BLE               0x08
#define LIS2MDL_CFG_REG_C_4WSPI             0x04
#define LIS2MDL_CFG_REG_C_SELF_TEST         0x02
#define LIS2MDL_CFG_REG_C_DRDY_ON_PIN       0x01

#define LIS2MDL_INT_CTRL_REG        0x63
#define LIS2MDL_INT_CTRL_REG_XIEN           0x80
#define LIS2MDL_INT_CTRL_REG_YIEN           0x40
#define LIS2MDL_INT_CTRL_REG_ZIEN           0x20
#define LIS2MDL_INT_CTRL_REG_IEA            0x04
#define LIS2MDL_INT_CTRL_REG_IEL            0x02
#define LIS2MDL_INT_CTRL_REG_IEN            0x01

#define LIS2MDL_INT_SOURCE_REG      0x64
#define LIS2MDL_INT_SOURCE_REG_P_TH_S_X     0x80
#define LIS2MDL_INT_SOURCE_REG_P_TH_S_Y     0x40
#define LIS2MDL_INT_SOURCE_REG_P_TH_S_Z     0x20
#define LIS2MDL_INT_SOURCE_REG_N_TH_S_X     0x10
#define LIS2MDL_INT_SOURCE_REG_N_TH_S_Y     0x08
#define LIS2MDL_INT_SOURCE_REG_N_TH_S_Z     0x04
#define LIS2MDL_INT_SOURCE_REG_MROI         0x02
#define LIS2MDL_INT_SOURCE_REG_INT          0x01

#define LIS2MDL_INT_THS_L_REG       0x65
#define LIS2MDL_INT_THS_H_REG       0x66

#define LIS2MDL_STATUS_REG          0x67
#define LIS2MDL_STATUS_REG_ZXYOR            0x80
#define LIS2MDL_STATUS_REG_ZOR              0x40
#define LIS2MDL_STATUS_REG_YOR              0x20
#define LIS2MDL_STATUS_REG_XOR              0x10
#define LIS2MDL_STATUS_REG_ZXYDA            0x08
#define LIS2MDL_STATUS_REG_ZDA              0x04
#define LIS2MDL_STATUS_REG_YDA              0x02
#define LIS2MDL_STATUS_REG_XDA              0x01

#define LIS2MDL_OUTX_L_REG          0x68
#define LIS2MDL_OUTX_H_REG          0x69
#define LIS2MDL_OUTY_L_REG          0x6A
#define LIS2MDL_OUTY_H_REG          0x6B
#define LIS2MDL_OUTZ_L_REG          0x6C
#define LIS2MDL_OUTZ_H_REG          0x6D

#define LIS2MDL_TEMP_OUT_L_REG      0x6E
#define LIS2MDL_TEMP_OUT_H_REG      0x6F

static bool lis2mdlRead(magDev_t * mag, int16_t *magData)
{
    static uint8_t buf[6];
    static bool pendingRead = true;

    extDevice_t *dev = &mag->dev;

    if (pendingRead) {
        if (busReadRegisterBufferStart(dev, LIS2MDL_OUTX_L_REG, buf, sizeof(buf))) {
            pendingRead = false;
        }
        return false;
    }

    magData[X] = (int16_t)(buf[1] << 8 | buf[0]);
    magData[Y] = (int16_t)(buf[3] << 8 | buf[2]);
    magData[Z] = (int16_t)(buf[5] << 8 | buf[4]);

    pendingRead = true;

    return true;
}

static bool lis2mdlInit(magDev_t *mag)
{
    extDevice_t *dev = &mag->dev;

    busDeviceRegister(dev);

    busWriteRegister(dev, LIS2MDL_CFG_REG_A,
                     LIS2MDL_CFG_REG_A_COMP_TEMP_EN |
                     LIS2MDL_ENCODE_BITS(LIS2MDL_CFG_REG_A_ODR_100, LIS2MDL_CFG_REG_A_ODR_MASK, LIS2MDL_CFG_REG_A_ODR_SHIFT) |
                     LIS2MDL_ENCODE_BITS(LIS2MDL_CFG_REG_A_MD_CONT, LIS2MDL_CFG_REG_A_MD_MASK, LIS2MDL_CFG_REG_A_MD_SHIFT));

    delay(100);

    return true;
}

bool lis2mdlDetect(magDev_t * mag)
{
    extDevice_t *dev = &mag->dev;

    uint8_t sig = 0;

    if (dev->bus->busType == BUS_TYPE_I2C && dev->busType_u.i2c.address == 0) {
        dev->busType_u.i2c.address = LIS2MDL_MAG_I2C_ADDRESS;
    }

    bool ack = busReadRegisterBuffer(&mag->dev, LIS2MDL_REG_WHO_AM_I, &sig, 1);

    if (!ack || sig != LIS2MDL_DEVICE_ID) {
        return false;
    }

    mag->init = lis2mdlInit;
    mag->read = lis2mdlRead;

    return true;
}
#endif
