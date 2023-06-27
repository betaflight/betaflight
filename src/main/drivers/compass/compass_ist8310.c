/*
 * This file is part of Cleanflight, Betaflight and INAV.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 *
 * Copyright: INAVFLIGHT OU
 */

#include <stdbool.h>
#include <stdint.h>

#include <math.h>

#include "platform.h"

#ifdef USE_MAG_IST8310

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/nvic.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "compass.h"
#include "compass_ist8310.h"

//#define DEBUG_MAG_DATA_READY_INTERRUPT

#define IST8310_MAG_I2C_ADDRESS 0x0C

/* ist8310 Slave Address Select : default address 0x0C
 *        CAD1  |  CAD0   |  Address
 *    ------------------------------
 *         VSS   |   VSS  |  0CH
 *         VSS   |   VDD  |  0DH
 *         VDD   |   VSS  |  0EH
 *         VDD   |   VDD  |  0FH
 * if CAD1 and CAD0 are floating, I2C address will be 0EH
 *
 *
 * CTRL_REGA: Control Register 1
 * Read Write
 * Default value: 0x0A
 * 7:4  0   Reserved.
 * 3:0  DO2-DO0: Operating mode setting
 *        DO3  |  DO2 |  DO1 |  DO0 |   mode
 *    ------------------------------------------------------
 *         0   |   0  |  0   |  0   |   Stand-By mode
 *         0   |   0  |  0   |  1   |   Single measurement mode
 *                                       Others: Reserved
 *
 * CTRL_REGB: Control Register 2
 * Read Write
 * Default value: 0x0B
 * 7:4  0   Reserved.
 * 3    DREN : Data ready enable control:
 *      0: disable
 *      1: enable
 * 2    DRP: DRDY pin polarity control
 *      0: active low
 *      1: active high
 * 1    0   Reserved.
 * 0    SRST: Soft reset, perform Power On Reset (POR) routine
 *      0: no action
 *      1: start immediately POR routine
 *      This bit will be set to zero after POR routine
 */

#define IST8310_REG_DATA 0x03
#define IST8310_REG_WHOAMI 0x00

// I2C Control Register
#define IST8310_REG_CNTRL1 0x0A
#define IST8310_REG_CNTRL2 0x0B
#define IST8310_REG_AVERAGE 0x41
#define IST8310_REG_PDCNTL 0x42

// Parameter
// ODR = Output Data Rate, we use single measure mode for getting more data.
#define IST8310_ODR_SINGLE 0x01
#define IST8310_ODR_10_HZ 0x03
#define IST8310_ODR_20_HZ 0x05
#define IST8310_ODR_50_HZ 0x07
#define IST8310_ODR_100_HZ 0x06

// Device ID (ist8310 -> 0x10)
#define IST8310_CHIP_ID 0x10
#define IST8310_AVG_16  0x24
#define IST8310_PULSE_DURATION_NORMAL 0xC0

#define IST8310_CNTRL2_RESET 0x01
#define IST8310_CNTRL2_DRPOL 0x04
#define IST8310_CNTRL2_DRENA 0x08

static bool ist8310Init(magDev_t *magDev)
{
    extDevice_t *dev = &magDev->dev;
    uint8_t regTemp;

    busDeviceRegister(dev);

    // soft-Reset
    bool ack = busReadRegisterBuffer(dev, IST8310_REG_CNTRL2, &regTemp, 1);
    regTemp |= IST8310_CNTRL2_RESET;
    ack = ack && busWriteRegister(dev, IST8310_REG_CNTRL2, regTemp);
    delay(30);

    // ODR mode
    ack = ack && busWriteRegister(dev, IST8310_REG_CNTRL1, IST8310_ODR_50_HZ);
    delay(5);

    // Init setting : avg16 / pulse mode
    ack = ack && busWriteRegister(dev, IST8310_REG_AVERAGE, IST8310_AVG_16);
    delay(5);
    ack = ack && busWriteRegister(dev, IST8310_REG_PDCNTL, IST8310_PULSE_DURATION_NORMAL);
    delay(5);

    return ack;
}


static bool ist8310Read(magDev_t * magDev, int16_t *magData)
{
    uint8_t buf[6];
    uint8_t LSB2FSV = 3; // 3mG - 14 bit

    extDevice_t *dev = &magDev->dev;

    // write 0x01 to register 0x0A to set single measure mode
    busWriteRegister(dev, IST8310_REG_CNTRL1, IST8310_ODR_SINGLE);
    delay(5);

    // read 6 bytes from register 0x03
    if (!busReadRegisterBuffer(dev, IST8310_REG_DATA, buf, 6)) {
        // set magData to zero for case of failed read
        magData[X] = 0;
        magData[Y] = 0;
        magData[Z] = 0;

        return false;
    }

    // Looks like datasheet is incorrect and we need to invert Y axis to conform to right hand rule
    magData[X] =  (int16_t)(buf[1] << 8 | buf[0]) * LSB2FSV;
    magData[Y] = -(int16_t)(buf[3] << 8 | buf[2]) * LSB2FSV;
    magData[Z] =  (int16_t)(buf[5] << 8 | buf[4]) * LSB2FSV;

    // TODO: do cross axis compensation

    return true;
}

static bool deviceDetect(magDev_t * magDev)
{
    extDevice_t *dev = &magDev->dev;

    uint8_t sig = 0;
    bool ack = busReadRegisterBuffer(dev, IST8310_REG_WHOAMI, &sig, 1);
    ack = busReadRegisterBuffer(dev, IST8310_REG_WHOAMI, &sig, 1);
    ack = busReadRegisterBuffer(dev, IST8310_REG_WHOAMI, &sig, 1);

    if (ack && sig == IST8310_CHIP_ID) {
        // TODO: set device in standby mode
        return true;
    }

    return false;
}

bool ist8310Detect(magDev_t * magDev)
{
    extDevice_t *dev = &magDev->dev;

    if (dev->bus->busType == BUS_TYPE_I2C && dev->busType_u.i2c.address == 0) {
        dev->busType_u.i2c.address = IST8310_MAG_I2C_ADDRESS;
    }

    if (deviceDetect(magDev)) {
        magDev->init = ist8310Init;
        magDev->read = ist8310Read;

        return true;
    }

    return false;
}

#endif
