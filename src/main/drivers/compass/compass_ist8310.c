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

#ifdef USE_MAG_IST8310

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
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
#define IST8310_REG_WAI 0x00
#define IST8310_REG_WAI_VALID 0x10

#define IST8310_REG_STAT1 0x02
#define IST8310_REG_STAT2 0x09

#define IST8310_DRDY_MASK 0x01

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

#define IST8310_AVG_16  0x24
#define IST8310_PULSE_DURATION_NORMAL 0xC0

#define IST8310_CNTRL2_RESET 0x01
#define IST8310_CNTRL2_DRPOL 0x04
#define IST8310_CNTRL2_DRENA 0x08

static bool ist8310Init(magDev_t *magDev)
{
    extDevice_t *dev = &magDev->dev;

    busDeviceRegister(dev);

    // Init setting
    bool ack = busWriteRegister(dev, IST8310_REG_AVERAGE, IST8310_AVG_16);
    delay(6);
    ack = ack && busWriteRegister(dev, IST8310_REG_PDCNTL, IST8310_PULSE_DURATION_NORMAL);
    delay(6); 
    ack = ack && busWriteRegister(dev, IST8310_REG_CNTRL1, IST8310_ODR_SINGLE);

    magDev->magOdrHz = 100;
    // need to check what ODR is actually returned, may be a bit faster than 100Hz
    return ack;
}

static bool ist8310Read(magDev_t * magDev, int16_t *magData)
{
    extDevice_t *dev = &magDev->dev;

    static uint8_t buf[6];
    const int LSB2FSV = 3; // 3mG - 14 bit

    static enum {
        STATE_REQUEST_DATA,
        STATE_FETCH_DATA,
    } state = STATE_REQUEST_DATA;

    switch (state) {
        default:
        case STATE_REQUEST_DATA:
            if (busReadRegisterBufferStart(dev, IST8310_REG_DATA, buf, sizeof(buf))) {
                state = STATE_FETCH_DATA;
            }

            return false;
        case STATE_FETCH_DATA:
            // Looks like datasheet is incorrect and we need to invert Y axis to conform to right hand rule
            magData[X] =  (int16_t)(buf[1] << 8 | buf[0]) * LSB2FSV;
            magData[Y] = -(int16_t)(buf[3] << 8 | buf[2]) * LSB2FSV;
            magData[Z] =  (int16_t)(buf[5] << 8 | buf[4]) * LSB2FSV;

            // Force single measurement mode for next read
            if (busWriteRegisterStart(dev, IST8310_REG_CNTRL1, IST8310_ODR_SINGLE)) {
                state = STATE_REQUEST_DATA;

                return true;
            }

            return false;
    }

    // TODO: do cross axis compensation

    return false;
}

static bool deviceDetect(magDev_t * magDev)
{
    uint8_t result = 0;
    bool ack = busReadRegisterBuffer(&magDev->dev, IST8310_REG_WAI, &result, 1);

    return ack && result == IST8310_REG_WAI_VALID;
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
