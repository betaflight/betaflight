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

// LIS2MDL, IIS2MDC, LSM303AGR and LSM303AH are firmware and pin-to-pin compatible solutions
// https://www.st.com/resource/en/design_tip/dt0131-digital-magnetometer-and-ecompass-efficient-design-tips--stmicroelectronics.pdf

#include "compass.h"
#include "drivers/time.h"
#include "common/axis.h"

#include "compass_lis2mdl.h"

#define LIS2MDL_MAG_I2C_ADDRESS 0x1E

// LIS2MDL Registers
#define LIS2MDL_ADDR_CFG_REG_A  0x60
#define LIS2MDL_ADDR_CFG_REG_B  0x61
#define LIS2MDL_ADDR_CFG_REG_C  0x62
#define LIS2MDL_ADDR_STATUS_REG 0x67
#define LIS2MDL_ADDR_OUTX_L_REG 0x68
#define LIS2MDL_ADDR_WHO_AM_I   0x4F

// LIS2MDL Definitions
#define LIS2MDL_WHO_AM_I         0x40
#define LIS2MDL_STATUS_REG_READY 0x0F
#define CFGA_MD_CONTINUOUS   (0 << 0)
#define CFGA_ODR_100         ((1 << 3) | (1 << 2))
#define CFGA_COMP_TEMP_EN    (1 << 7)
#define CFGB_OFF_CANC        (1 << 1)
#define CFGC_BDU             (1 << 4)

static bool lis2mdlInit(magDev_t *mag)
{
    bool ack = true;
    extDevice_t *dev = &mag->dev;

    busDeviceRegister(dev);

    ack = ack && busWriteRegister(dev, LIS2MDL_ADDR_CFG_REG_A, CFGA_MD_CONTINUOUS | CFGA_ODR_100 | CFGA_COMP_TEMP_EN);
    ack = ack && busWriteRegister(dev, LIS2MDL_ADDR_CFG_REG_B, CFGB_OFF_CANC);
    ack = ack && busWriteRegister(dev, LIS2MDL_ADDR_CFG_REG_C, CFGC_BDU);

    if (!ack) {
        return false;
    }

    mag->magOdrHz = 100;
    return true;
}

static bool lis2mdlRead(magDev_t *mag, int16_t *magData)
{
    static uint8_t buf[6];
    static bool pendingRead = true;

    extDevice_t *dev = &mag->dev;

    if (pendingRead && busReadRegisterBufferStart(dev, LIS2MDL_ADDR_OUTX_L_REG, (uint8_t *)buf, sizeof(buf))) {
        pendingRead = false;
    }
        return false;
    }

    // Sensitivity is +/- 50,000 milligauss, 16bit
    // e.g. gauss = val * (100.f / 65.535f)

    int16_t x = (int16_t)(buf[1] << 8 | buf[0]);
    int16_t y = (int16_t)(buf[3] << 8 | buf[2]);
    int16_t z = (int16_t)(buf[5] << 8 | buf[4]);

    // adapt LIS2MDL left-handed frame to common sensor axis orientation  (match LIS3MDL)
    // pin 1 mark becomes +X -Y
    magData[X] = -x;
    magData[Y] = y;
    magData[Z] = z;

    pendingRead = true;

    return true;
}

bool lis2mdlDetect(magDev_t *mag)
{
    extDevice_t *dev = &mag->dev;

    if (dev->bus->busType == BUS_TYPE_I2C && dev->busType_u.i2c.address == 0) {
        dev->busType_u.i2c.address = LIS2MDL_MAG_I2C_ADDRESS;
    }

    uint8_t whoami;
    bool ack = busReadRegisterBuffer(dev, LIS2MDL_ADDR_WHO_AM_I, &whoami, sizeof(whoami));

    if (ack && whoami == LIS2MDL_WHO_AM_I) {
        mag->init = lis2mdlInit;
        mag->read = lis2mdlRead;
        return true;
    }

    return false;
}
#endif
