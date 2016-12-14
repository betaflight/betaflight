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

#include <math.h>

#include "platform.h"

#ifdef USE_MAG_IST8310

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "system.h"
#include "nvic.h"
#include "io.h"
#include "exti.h"
#include "bus_i2c.h"
#include "light_led.h"

#include "sensor.h"
#include "compass.h"

#include "compass_ist8310.h"

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

#define IST8310_ADDRESS 0x0C
#define IST8310_REG_DATA 0x03
#define IST8310_REG_WHOAMI 0x00

// I2C Contorl Register
#define IST8310_REG_CNTRL1 0x0A
#define IST8310_REG_CNTRL2 0x0B
#define IST8310_REG_AVERAGE 0x41

// Parameter
// ODR = Output Data Rate, we use single measure mode for getting more data.
#define IST8310_ODR_SINGLE 0x01
#define IST8310_ODR_10_HZ 0x03
#define IST8310_ODR_20_HZ 0x05
#define IST8310_ODR_50_HZ 0x07
#define IST8310_ODR_100_HZ 0x06 

// Device ID (ist8310 -> 0x10)
#define IST8310_CHIP_ID 0x10
#define IST8310_AVG_16 0x24

#define IST8310_CNTRL2_RESET 0x01
#define IST8310_CNTRL2_DRPOL 0x04
#define IST8310_CNTRL2_DRENA 0x08

static bool ist8310Init(void)
{
    i2cWrite(MAG_I2C_INSTANCE, IST8310_ADDRESS, IST8310_REG_CNTRL1, IST8310_ODR_50_HZ);
    delay(5);
    i2cWrite(MAG_I2C_INSTANCE, IST8310_ADDRESS, IST8310_REG_AVERAGE, IST8310_AVG_16);
    delay(5);

    return true;
}

static bool ist8310Read(int16_t *magData)
{
    uint8_t buf[6];
    uint8_t LSB2FSV = 3; // 3mG - 14 bit

    // set magData to zero for case of failed read
    magData[X] = 0;
    magData[Y] = 0;
    magData[Z] = 0;

    bool ack = i2cRead(MAG_I2C_INSTANCE, IST8310_ADDRESS, IST8310_REG_DATA, 6, buf);
    if (!ack) {
        return false;
    }

    // need to modify when confirming the pcb direction
    magData[X] = (int16_t)(buf[1] << 8 | buf[0]) * LSB2FSV;
    magData[Y] = (int16_t)(buf[3] << 8 | buf[2]) * LSB2FSV;
    magData[Z] = (int16_t)(buf[5] << 8 | buf[4]) * LSB2FSV;

    return true;
}

#define DETECTION_MAX_RETRY_COUNT   5
bool ist8310Detect(magDev_t *mag)
{
    for (int retryCount = 0; retryCount < DETECTION_MAX_RETRY_COUNT; retryCount++) {
        uint8_t sig = 0;
        bool ack = i2cRead(MAG_I2C_INSTANCE, IST8310_ADDRESS, IST8310_REG_WHOAMI, 1, &sig);
        if (ack && sig == IST8310_CHIP_ID) {
            mag->init = ist8310Init;
            mag->read = ist8310Read;
            return true;
        }
    }

    return false;
}

#endif
