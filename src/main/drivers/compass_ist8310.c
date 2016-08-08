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

#include "debug.h"

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

#include "sensors/sensors.h"

#include "compass_ist8310.h"

//#define DEBUG_MAG_DATA_READY_INTERRUPT

// ist8310, default address 0x1C
// NAZE Target connections
// PB12 connected to MAG_DRDY on rev4 hardware
// PC14 connected to MAG_DRDY on rev5 hardware

/* CTRL_REGA: Control Register 1
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

#define MAG_ADDRESS 0x0C
#define MAG_DATA_REGISTER 0x03
#define MAG_WHOAMI 0x00

#define IST8310_REG_CNTRL1 0x0A
#define IST8310_REG_CNTRL2 0x0B
#define IST8310_AVERAGE 0x41

#define IST8310_SINGLE_MODE 0x01
#define IST8310_ODR_10_HZ 0x03
#define IST8310_ODR_20_HZ 0x05
#define IST8310_ODR_50_HZ 0x07
#define IST8310_ODR_100_HZ 0x06 
#define IST8310_AVG_16_TIME 0x24
#define IST8310_RESET 0x0D
#define IST8310_ID 0x10

 
static const ist8310Config_t *ist8310Config = NULL;

#ifdef USE_MAG_DATA_READY_SIGNAL

static IO_t intIO;
static extiCallbackRec_t ist8310_extiCallbackRec;

void ist8310_extiHandler(extiCallbackRec_t* cb)
{
    UNUSED(cb);
#ifdef DEBUG_MAG_DATA_READY_INTERRUPT
    // Measure the delta between calls to the interrupt handler
    // currently should be around 65/66 milli seconds / 15hz output rate
    static uint32_t lastCalledAt = 0;
    static int32_t callDelta = 0;

    uint32_t now = millis();
    callDelta = now - lastCalledAt;

    //UNUSED(callDelta);
    debug[0] = callDelta;

    lastCalledAt = now;
#endif
}
#endif

static void ist8310ConfigureDataReadyInterruptHandling(void)
{
#ifdef USE_MAG_DATA_READY_SIGNAL

    if (!(ist8310Config->intTag)) {
        return;
    }
    intIO = IOGetByTag(ist8310Config->intTag);
#ifdef ENSURE_MAG_DATA_READY_IS_HIGH
    uint8_t status = IORead(intIO);
    if (!status) {
        return;
    }
#endif

    EXTIHandlerInit(&ist8310_extiCallbackRec, ist8310_extiHandler);
    EXTIConfig(intIO, &ist8310_extiCallbackRec, NVIC_PRIO_MAG_INT_EXTI, EXTI_Trigger_Rising);
    EXTIEnable(intIO, true);
#endif
}

bool ist8310Detect(mag_t* mag, const ist8310Config_t *ist8310ConfigToUse)
{
    bool ack = false;
     uint8_t sig = 0;

    ist8310Config = ist8310ConfigToUse;

    ack = i2cRead(MAG_I2C_INSTANCE, MAG_ADDRESS, MAG_WHOAMI, 1, &sig);
    if (!ack || (sig != IST8310_ID))
        return false;

    mag->init = ist8310Init;
    mag->read = ist8310Read;

    return true;
}

void ist8310Init(void)
{
    int16_t magADC[3];

    i2cWrite(MAG_I2C_INSTANCE, MAG_ADDRESS, IST8310_REG_CNTRL1, IST8310_SINGLE_MODE);
    delay(5);
    i2cWrite(MAG_I2C_INSTANCE, MAG_ADDRESS, IST8310_AVERAGE, IST8310_AVG_16_TIME);
    delay(5);
    ist8310Read(magADC);
    delay(5);
    i2cWrite(MAG_I2C_INSTANCE, MAG_ADDRESS, IST8310_REG_CNTRL1, IST8310_SINGLE_MODE);

    ist8310ConfigureDataReadyInterruptHandling();
}

bool ist8310Read(int16_t *magData)
{
    uint8_t buf[6];
    float LSB2FSV = 3; // 3mG - 14 bit
    bool ack = i2cRead(MAG_I2C_INSTANCE, MAG_ADDRESS, MAG_DATA_REGISTER, 6, buf);
    if (!ack) {
        return false;
    }

    // need to modify when confirming the pcb direction
    magData[X] = -(int16_t)(buf[1] << 8 | buf[0]) * LSB2FSV;
    magData[Y] = (int16_t)(buf[3] << 8 | buf[2]) * LSB2FSV;
    magData[Z] = (int16_t)(buf[5] << 8 | buf[4]) * LSB2FSV;
    i2cWrite(MAG_I2C_INSTANCE, MAG_ADDRESS, IST8310_REG_CNTRL1, IST8310_SINGLE_MODE);
    return true;
}
#endif
