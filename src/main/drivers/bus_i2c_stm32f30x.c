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
#include <string.h>

#include <platform.h>

#if defined(USE_I2C) && !defined(SOFT_I2C)

#include "build/debug.h"

#include "drivers/system.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/rcc.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_impl.h"

#define IOCFG_I2C_PU IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_OD, GPIO_PuPd_UP)
#define IOCFG_I2C    IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_OD, GPIO_PuPd_NOPULL)

#define I2C_HIGHSPEED_TIMING  0x00500E30  // 1000 Khz, 72Mhz Clock, Analog Filter Delay ON, Setup 40, Hold 4.
#define I2C_STANDARD_TIMING   0x00E0257A  // 400 Khz, 72Mhz Clock, Analog Filter Delay ON, Rise 100, Fall 10.

#define I2C_GPIO_AF         GPIO_AF_4

static uint32_t i2cTimeout;

static volatile uint16_t i2cErrorCount = 0;

const i2cHardware_t i2cHardware[I2CDEV_COUNT] = {
#ifdef USE_I2C_DEVICE_1
    {
        .device = I2CDEV_1,
        .reg = I2C1,
        .sclPins = { I2CPINDEF(PA15), I2CPINDEF(PB6), I2CPINDEF(PB8) },
        .sdaPins = { I2CPINDEF(PA14), I2CPINDEF(PB7), I2CPINDEF(PB9) },
        .rcc = RCC_APB1(I2C1),
    },
#endif
#ifdef USE_I2C_DEVICE_2
    {
        .device = I2CDEV_2,
        .reg = I2C2,
        .sclPins = { I2CPINDEF(PA9), I2CPINDEF(PF6) },
        .sdaPins = { I2CPINDEF(PA10) },
        .rcc = RCC_APB1(I2C2),
    },
#endif
};

i2cDevice_t i2cDevice[I2CDEV_COUNT];

///////////////////////////////////////////////////////////////////////////////
// I2C TimeoutUserCallback
///////////////////////////////////////////////////////////////////////////////

uint32_t i2cTimeoutUserCallback(void)
{
    i2cErrorCount++;
    return false;
}

void i2cInit(I2CDevice device)
{
    if (device == I2CINVALID || device > I2CDEV_COUNT) {
        return;
    }

    i2cDevice_t *pDev = &i2cDevice[device];
    const i2cHardware_t *hw = pDev->hardware;

    if (!hw) {
        return;
    }

    I2C_TypeDef *I2Cx = pDev->reg;

    IO_t scl = pDev->scl;
    IO_t sda = pDev->sda;

    RCC_ClockCmd(hw->rcc, ENABLE);
    RCC_I2CCLKConfig(I2Cx == I2C2 ? RCC_I2C2CLK_SYSCLK : RCC_I2C1CLK_SYSCLK);

    IOInit(scl, OWNER_I2C_SCL, RESOURCE_INDEX(device));
    IOConfigGPIOAF(scl, pDev->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, GPIO_AF_4);

    IOInit(sda, OWNER_I2C_SDA, RESOURCE_INDEX(device));
    IOConfigGPIOAF(sda, pDev->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, GPIO_AF_4);

    I2C_InitTypeDef i2cInit = {
        .I2C_Mode = I2C_Mode_I2C,
        .I2C_AnalogFilter = I2C_AnalogFilter_Enable,
        .I2C_DigitalFilter = 0x00,
        .I2C_OwnAddress1 = 0x00,
        .I2C_Ack = I2C_Ack_Enable,
        .I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit,
        .I2C_Timing = (pDev->overClock ? I2C_HIGHSPEED_TIMING : I2C_STANDARD_TIMING)
    };

    I2C_Init(I2Cx, &i2cInit);

    I2C_StretchClockCmd(I2Cx, ENABLE);

    I2C_Cmd(I2Cx, ENABLE);
}

uint16_t i2cGetErrorCounter(void)
{
    return i2cErrorCount;
}

bool i2cWrite(I2CDevice device, uint8_t addr_, uint8_t reg, uint8_t data)
{
    if (device == I2CINVALID || device > I2CDEV_COUNT) {
        return false;
    }

    I2C_TypeDef *I2Cx = i2cDevice[device].reg;

    if (!I2Cx) {
        return false;
    }

    addr_ <<= 1;

    /* Test on BUSY Flag */
    i2cTimeout = I2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_BUSY) != RESET) {
        if ((i2cTimeout--) == 0) {
            return i2cTimeoutUserCallback();
        }
    }

    /* Configure slave address, nbytes, reload, end mode and start or stop generation */
    I2C_TransferHandling(I2Cx, addr_, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);

    /* Wait until TXIS flag is set */
    i2cTimeout = I2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_TXIS) == RESET) {
        if ((i2cTimeout--) == 0) {
            return i2cTimeoutUserCallback();
        }
    }

    /* Send Register address */
    I2C_SendData(I2Cx, (uint8_t) reg);

    /* Wait until TCR flag is set */
    i2cTimeout = I2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_TCR) == RESET)
    {
        if ((i2cTimeout--) == 0) {
            return i2cTimeoutUserCallback();
        }
    }

    /* Configure slave address, nbytes, reload, end mode and start or stop generation */
    I2C_TransferHandling(I2Cx, addr_, 1, I2C_AutoEnd_Mode, I2C_No_StartStop);

    /* Wait until TXIS flag is set */
    i2cTimeout = I2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_TXIS) == RESET) {
        if ((i2cTimeout--) == 0) {
            return i2cTimeoutUserCallback();
        }
    }

    /* Write data to TXDR */
    I2C_SendData(I2Cx, data);

    /* Wait until STOPF flag is set */
    i2cTimeout = I2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_STOPF) == RESET) {
        if ((i2cTimeout--) == 0) {
            return i2cTimeoutUserCallback();
        }
    }

    /* Clear STOPF flag */
    I2C_ClearFlag(I2Cx, I2C_ICR_STOPCF);

    return true;
}

bool i2cRead(I2CDevice device, uint8_t addr_, uint8_t reg, uint8_t len, uint8_t* buf)
{
    if (device == I2CINVALID || device > I2CDEV_COUNT) {
        return false;
    }

    I2C_TypeDef *I2Cx = i2cDevice[device].reg;

    if (!I2Cx) {
        return false;
    }

    addr_ <<= 1;

    /* Test on BUSY Flag */
    i2cTimeout = I2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_BUSY) != RESET) {
        if ((i2cTimeout--) == 0) {
            return i2cTimeoutUserCallback();
        }
    }

    /* Configure slave address, nbytes, reload, end mode and start or stop generation */
    I2C_TransferHandling(I2Cx, addr_, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

    /* Wait until TXIS flag is set */
    i2cTimeout = I2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_TXIS) == RESET) {
        if ((i2cTimeout--) == 0) {
            return i2cTimeoutUserCallback();
        }
    }

    /* Send Register address */
    I2C_SendData(I2Cx, (uint8_t) reg);

    /* Wait until TC flag is set */
    i2cTimeout = I2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_TC) == RESET) {
        if ((i2cTimeout--) == 0) {
            return i2cTimeoutUserCallback();
        }
    }

    /* Configure slave address, nbytes, reload, end mode and start or stop generation */
    I2C_TransferHandling(I2Cx, addr_, len, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

    /* Wait until all data are received */
    while (len) {
        /* Wait until RXNE flag is set */
        i2cTimeout = I2C_LONG_TIMEOUT;
        while (I2C_GetFlagStatus(I2Cx, I2C_ISR_RXNE) == RESET) {
            if ((i2cTimeout--) == 0) {
                return i2cTimeoutUserCallback();
            }
        }

        /* Read data from RXDR */
        *buf = I2C_ReceiveData(I2Cx);
        /* Point to the next location where the byte read will be saved */
        buf++;

        /* Decrement the read bytes counter */
        len--;
    }

    /* Wait until STOPF flag is set */
    i2cTimeout = I2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_STOPF) == RESET) {
        if ((i2cTimeout--) == 0) {
            return i2cTimeoutUserCallback();
        }
    }

    /* Clear STOPF flag */
    I2C_ClearFlag(I2Cx, I2C_ICR_STOPCF);

    /* If all operations OK */
    return true;
}

#endif
