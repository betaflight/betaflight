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
#include <stdlib.h>

#include <math.h>

#include <platform.h>
#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "config/parameter_group.h"

#include "system.h"
#include "nvic.h"
#include "gpio.h"
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

void IST_MAG_DATA_READY_EXTI_Handler(void)
{
    if (EXTI_GetITStatus(ist8310Config->exti_line) == RESET) {
        return;
    }

    EXTI_ClearITPendingBit(ist8310Config->exti_line);

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

static void ist8310ConfigureDataReadyInterruptHandling(void)
{
#ifdef USE_MAG_DATA_READY_SIGNAL

    if (!(ist8310Config->exti_port_source && ist8310Config->exti_pin_source)) {
        return;
    }
#ifdef STM32F10X
    // enable AFIO for EXTI support
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
#endif

#ifdef STM32F303xC
    /* Enable SYSCFG clock otherwise the EXTI irq handlers are not called */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
#endif

#ifdef STM32F10X
    gpioExtiLineConfig(ist8310Config->exti_port_source, ist8310Config->exti_pin_source);
#endif

#ifdef STM32F303xC
    gpioExtiLineConfig(ist8310Config->exti_port_source, ist8310Config->exti_pin_source);
#endif

#ifdef ENSURE_MAG_DATA_READY_IS_HIGH
    uint8_t status = GPIO_ReadInputDataBit(ist8310Config->gpioPort, ist8310Config->gpioPin);
    if (!status) {
        return;
    }
#endif

    registerExtiCallbackHandler(ist8310Config->exti_irqn, IST_MAG_DATA_READY_EXTI_Handler);

    EXTI_ClearITPendingBit(ist8310Config->exti_line);

    EXTI_InitTypeDef EXTIInit;
    EXTIInit.EXTI_Line = ist8310Config->exti_line;
    EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTIInit.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTIInit.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTIInit);

    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = ist8310Config->exti_irqn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_MAG_DATA_READY);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_MAG_DATA_READY);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif
}

bool ist8310Detect(mag_t* mag, const ist8310Config_t *ist8310ConfigToUse)
{
    bool ack = false;
     uint8_t sig = 0;

    ist8310Config = ist8310ConfigToUse;

    ack = i2cRead(MAG_ADDRESS, MAG_WHOAMI, 1, &sig);
    if (!ack || (sig != IST8310_ID))
        return false;

    mag->init = ist8310Init;
    mag->read = ist8310Read;

    return true;
}

void ist8310Init(void)
{
    int16_t magADC[3];
    // int i;
    // int32_t xyz_total[3] = { 0, 0, 0 }; // 32 bit totals so they won't overflow.
    // bool bret = true;           // Error indicator

    gpio_config_t gpio;

    if (ist8310Config) {
#ifdef STM32F303
        if (ist8310Config->gpioAHBPeripherals) {
            RCC_AHBPeriphClockCmd(ist8310Config->gpioAHBPeripherals, ENABLE);
        }
#endif
#ifdef STM32F10X
        if (ist8310Config->gpioAPB2Peripherals) {
            RCC_APB2PeriphClockCmd(ist8310Config->gpioAPB2Peripherals, ENABLE);
        }
#endif
        gpio.pin = ist8310Config->gpioPin;
        gpio.speed = Speed_2MHz;
        gpio.mode = Mode_IN_FLOATING;
        gpioInit(ist8310Config->gpioPort, &gpio);
    }

    // 
    i2cWrite(MAG_ADDRESS, IST8310_REG_CNTRL1, IST8310_SINGLE_MODE);
    delay(5);
    i2cWrite(MAG_ADDRESS, IST8310_AVERAGE, IST8310_AVG_16_TIME);
    delay(5);
    ist8310Read(magADC);
    delay(5);
    i2cWrite(MAG_ADDRESS, IST8310_REG_CNTRL1, IST8310_SINGLE_MODE);
    
    // 20160802 wait for ist8310 hareware pcb for data ready pin on stm32f303cc
    // ist8310ConfigureDataReadyInterruptHandling();
   
}

bool ist8310Read(int16_t *magData)
{
    uint8_t buf[6];
    float LSB2FSV = 3; // 3mG - 14 bit
    bool ack = i2cRead(MAG_ADDRESS, MAG_DATA_REGISTER, 6, buf);
    if (!ack) {
        return false;
    }

    // need to modify when confirming the pcb direction
    magData[X] = -(int16_t)(buf[1] << 8 | buf[0]) * LSB2FSV;
    magData[Y] = (int16_t)(buf[3] << 8 | buf[2]) * LSB2FSV;
    magData[Z] = (int16_t)(buf[5] << 8 | buf[4]) * LSB2FSV;
    i2cWrite(MAG_ADDRESS, IST8310_REG_CNTRL1, IST8310_SINGLE_MODE);
    return true;
}
