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

#if defined(USE_I2C) && !defined(USE_SOFT_I2C)

#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/time.h"
#include "platform/rcc.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_impl.h"
#include "drivers/bus_i2c_timing.h"
#include "drivers/bus_i2c_utils.h"

#if I2C_TRAIT_HANDLE
#include "platform/bus_i2c_hal.h"
static struct i2cHalHandle_s i2cHalHandles[I2CDEV_COUNT];
#endif

#define IOCFG_I2C_PU IO_CONFIG(GPIO_MODE_AF, GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP)
#define IOCFG_I2C    IO_CONFIG(GPIO_MODE_AF, GPIO_SPEED_FREQ_HIGH, GPIO_NOPULL)

#define GPIO_AF4_I2C GPIO_AF4_I2C1

void Error_Handler(void);

const i2cHardware_t i2cHardware[I2CDEV_COUNT] = {
#ifdef USE_I2C_DEVICE_1
    {
        .device = I2CDEV_1,
        .reg = (i2cResource_t *)I2C1,
        .sclPins = { I2CPINDEF(PB6), I2CPINDEF(PB8) },
        .sdaPins = { I2CPINDEF(PB5), I2CPINDEF(PB7), I2CPINDEF(PB9) },
#if PLATFORM_TRAIT_RCC
        .rcc = RCC_APB1(I2C1),
#endif
        .ev_irq = I2C1_IRQn,
        .er_irq = I2C1_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_2
    {
        .device = I2CDEV_2,
        .reg = (i2cResource_t *)I2C2,
        .sclPins = { I2CPINDEF(PB10), I2CPINDEF(PB14) },
        .sdaPins = { I2CPINDEF(PB11), I2CPINDEF(PB12), I2CPINDEF(PB15) },
#if PLATFORM_TRAIT_RCC
        .rcc = RCC_APB1(I2C2),
#endif
        .ev_irq = I2C2_IRQn,
        .er_irq = I2C2_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_3
    {
        .device = I2CDEV_3,
        .reg = (i2cResource_t *)I2C3,
        .sclPins = { I2CPINDEF(PA8), I2CPINDEF(PC12) },
        .sdaPins = { I2CPINDEF(PC9), I2CPINDEF(PC9), I2CPINDEF(PD2) },
#if PLATFORM_TRAIT_RCC
        .rcc = RCC_APB1(I2C3),
#endif
        .ev_irq = I2C3_IRQn,
        .er_irq = I2C3_IRQn,
    },
#endif
};

i2cDevice_t i2cDevice[I2CDEV_COUNT];


void i2cInit(i2cDevice_e device)
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

    // Enable RCC
    RCC_ClockCmd(hardware->rcc, ENABLE);

    i2cUnstick(scl, sda);

    // Init pins
    IOConfigGPIOAF(scl, pDev->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, GPIO_AF4_I2C);
    IOConfigGPIOAF(sda, pDev->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, GPIO_AF4_I2C);

#if I2C_TRAIT_HANDLE
    // Init I2C peripheral
    pDev->halHandle = &i2cHalHandles[device];

    I2C_HandleTypeDef *pHandle = &pDev->halHandle->hal;

    memset(pHandle, 0, sizeof(*pHandle));

    pHandle->Instance = (I2C_TypeDef *)pDev->hardware->reg;

    pHandle->Init.ClockSpeed = I2C_EX_SPEED_FAST_400K;
    pHandle->Init.AddressingMode = I2C_EX_ADDRESSINGMODE_MASTER_7BIT;
    pHandle->Init.MasterRestartMode = I2C_EX_MASTER_RESTART_ENABLE;
    pHandle->Init.MasterBusClearMode = I2C_EX_MASTER_BUS_CLEAR_ENABLE;
    pHandle->Init.GeneralCallMode = I2C_EX_GENERALCALL_DISABLE;

	if(HAL_I2C_EX_Init(pHandle) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();    
	} 
#endif

    HAL_NVIC_SetPriority(hardware->er_irq, NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_EV), NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_EV));
    HAL_NVIC_EnableIRQ(hardware->er_irq);
}

#endif
