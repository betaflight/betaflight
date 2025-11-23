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
#include <string.h>

#include "platform.h"

#if defined(USE_I2C) && !defined(SOFT_I2C)

#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/time.h"
#include "drivers/rcc.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_impl.h"
#include "drivers/bus_i2c_timing.h"
#include "drivers/bus_i2c_utils.h"

#include "pg/pinio.h"

#define IOCFG_I2C_PU IO_CONFIG(GPIO_MODE_MUX , GPIO_DRIVE_STRENGTH_STRONGER, GPIO_OUTPUT_OPEN_DRAIN , GPIO_PULL_UP)
#define IOCFG_I2C    IO_CONFIG(GPIO_MODE_MUX , GPIO_DRIVE_STRENGTH_STRONGER, GPIO_OUTPUT_OPEN_DRAIN , GPIO_PULL_NONE)

const i2cHardware_t i2cHardware[I2CDEV_COUNT] = {
#ifdef USE_I2C_DEVICE_1
    {
        .device = I2CDEV_1,
        .reg = I2C1,
        .sclPins = {
            I2CPINDEF(PA9,  GPIO_MUX_8),
            I2CPINDEF(PB6,  GPIO_MUX_4),
            I2CPINDEF(PB8,  GPIO_MUX_4),
            I2CPINDEF(PC6,  GPIO_MUX_4),
        },
        .sdaPins = {
            I2CPINDEF(PA10, GPIO_MUX_8),
            I2CPINDEF(PB7,  GPIO_MUX_4),
            I2CPINDEF(PB9,  GPIO_MUX_4),
            I2CPINDEF(PC7,  GPIO_MUX_4),
        },
        .rcc = RCC_APB1(I2C1),
        .ev_irq = I2C1_EVT_IRQn,
        .er_irq = I2C1_ERR_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_2
    {
        .device = I2CDEV_2,
        .reg = I2C2,
        .sclPins = {
            I2CPINDEF(PA0,  GPIO_MUX_4),			
            I2CPINDEF(PA11, GPIO_MUX_4),
            I2CPINDEF(PB10, GPIO_MUX_4),
            I2CPINDEF(PD12, GPIO_MUX_4),
            I2CPINDEF(PH2,  GPIO_MUX_4),
        },
        .sdaPins = {
            I2CPINDEF(PA1,  GPIO_MUX_4),
            I2CPINDEF(PA12, GPIO_MUX_4),
            I2CPINDEF(PB3,  GPIO_MUX_4),
            I2CPINDEF(PB9,  GPIO_MUX_7),
            I2CPINDEF(PB11, GPIO_MUX_4),
            I2CPINDEF(PC12, GPIO_MUX_4),
            I2CPINDEF(PD13, GPIO_MUX_4),
            I2CPINDEF(PH3,  GPIO_MUX_4),
        },
        .rcc = RCC_APB1(I2C2),
        .ev_irq = I2C2_EVT_IRQn,
        .er_irq = I2C2_ERR_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_3
    {
        .device = I2CDEV_3,
        .reg = I2C3,
        .sclPins = {
            I2CPINDEF(PA8,  GPIO_MUX_4),
            I2CPINDEF(PB13, GPIO_MUX_7),
            I2CPINDEF(PB15, GPIO_MUX_4),
            I2CPINDEF(PC0,  GPIO_MUX_4),
            I2CPINDEF(PD14, GPIO_MUX_4),
        },
        .sdaPins = {
            I2CPINDEF(PB4,  GPIO_MUX_4),
            I2CPINDEF(PB14, GPIO_MUX_4),
            I2CPINDEF(PC1,  GPIO_MUX_4),
            I2CPINDEF(PC9,  GPIO_MUX_4),
            I2CPINDEF(PD15, GPIO_MUX_4),
        },
        .rcc = RCC_APB1(I2C3),
        .ev_irq = I2C3_EVT_IRQn,
        .er_irq = I2C3_ERR_IRQn,
    },
#endif
};

i2cDevice_t i2cDevice[I2CDEV_COUNT];

void i2cInit(I2CDevice device)
{
    if (device == I2CINVALID) {
        return;
    }

    i2cDevice_t *pDev = &i2cDevice[device];

    const i2cHardware_t *hardware = pDev->hardware;
    const IO_t scl = pDev->scl;
    const IO_t sda = pDev->sda;

    if (!hardware || IOGetOwner(scl) || IOGetOwner(sda)) {
        return;
    }

    IOInit(scl, OWNER_I2C_SCL, RESOURCE_INDEX(device));
    IOInit(sda, OWNER_I2C_SDA, RESOURCE_INDEX(device));

    // Enable i2c RCC
    RCC_ClockCmd(hardware->rcc, ENABLE);

    i2cUnstick(scl, sda);

    // Init pins

    IOConfigGPIOAF(scl, pDev->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, pDev->sclAF);
    IOConfigGPIOAF(sda, pDev->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, pDev->sdaAF);


    // Init I2C peripheral
    i2c_handle_type  *pHandle = &pDev->handle;
    memset(pHandle, 0, sizeof(*pHandle));

    i2c_type *i2cx = (i2c_type *)pDev->hardware->reg;
    pHandle->i2cx = i2cx;

    crm_clocks_freq_type crm_clk_freq;
    crm_clocks_freq_get(&crm_clk_freq);

    uint32_t i2cPclk = crm_clk_freq.apb1_freq;

    uint32_t I2Cx_CLKCTRL = i2cClockTIMINGR(i2cPclk, pDev->clockSpeed, 0);

    i2c_config(pHandle);

    i2c_init(i2cx, 0x0f, I2Cx_CLKCTRL);

    i2c_own_address1_set(i2cx, I2C_ADDRESS_MODE_7BIT, 0x0);

    nvic_irq_enable(hardware->er_irq, NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_ER), NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_ER));
    nvic_irq_enable(hardware->ev_irq, NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_EV), NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_EV));

    i2c_enable(i2cx, TRUE);
}

#endif
