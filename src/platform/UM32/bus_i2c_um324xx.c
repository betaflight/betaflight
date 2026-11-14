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
#include <string.h>

#include "platform.h"

#if defined(USE_I2C) && !defined(USE_SOFT_I2C)

#include "common/maths.h"

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
        .sdaPins = { I2CPINDEF(PA9), I2CPINDEF(PC9), I2CPINDEF(PD2) },
#if PLATFORM_TRAIT_RCC
        .rcc = RCC_APB1(I2C3),
#endif
        .ev_irq = I2C3_IRQn,
        .er_irq = I2C3_IRQn,
    },
#endif
};

i2cDevice_t i2cDevice[I2CDEV_COUNT];

static void i2cConfigureTiming(I2C_HandleTypeDef *pHandle, uint16_t clockSpeed)
{
    // UM324xF RM 26.4.3: SCL period = (LCNT + HCNT) / APB1 clock.
    // Fast-mode Plus supports up to 1 MHz on this controller.
    const uint32_t speed = constrain(clockSpeed, 100, 1000);
    const uint32_t pclk = HAL_RCM_GetPCLK1Freq();
    const uint32_t period = (pclk + speed * 1000 - 1) / (speed * 1000);
    const uint32_t lowNs = speed <= 100 ? 4700 : (speed <= 400 ? 1300 : 500);
    const uint32_t low = ((uint64_t)pclk * lowNs + 999999999) / 1000000000;

    __HAL_I2C_EX_DISABLE(pHandle);
    if (speed <= 100) {
        MODIFY_REG(pHandle->Instance->CR, I2C_EX_CR_SPEED_Msk, I2C_EX_ClockSpeed_Standard);
        pHandle->Instance->SSSCLLCNT = low;
        pHandle->Instance->SSSCLHCNT = period - low;
    } else {
        MODIFY_REG(pHandle->Instance->CR, I2C_EX_CR_SPEED_Msk, I2C_EX_ClockSpeed_Fast);
        pHandle->Instance->FSSCLLCNT = low;
        pHandle->Instance->FSSCLHCNT = period - low;
    }
    __HAL_I2C_EX_ENABLE(pHandle);
}


void i2cInit(i2cDevice_e device)
{
    if ((unsigned)device >= I2CDEV_COUNT) {
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
    IOConfigGPIOAF(scl, pDev->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, GPIO_AF4);
    IOConfigGPIOAF(sda, pDev->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, GPIO_AF4);

#if I2C_TRAIT_HANDLE
    // Init I2C peripheral
    pDev->halHandle = &i2cHalHandles[device];

    I2C_HandleTypeDef *pHandle = &pDev->halHandle->hal;

    memset(pHandle, 0, sizeof(*pHandle));

    pHandle->Instance = (I2C_TypeDef *)pDev->hardware->reg;

    // The SDK takes a speed enum, not a frequency in Hz.
    pHandle->Init.ClockSpeed = pDev->clockSpeed <= 100 ? I2C_EX_SPEED_STAD_100K
        : (pDev->clockSpeed <= 400 ? I2C_EX_SPEED_FAST_400K : I2C_EX_SPEED_HIGH_1M);
    pHandle->Init.AddressingMode = I2C_EX_ADDRESSINGMODE_MASTER_7BIT;
    pHandle->Init.MasterRestartMode = I2C_EX_MASTER_RESTART_ENABLE;
    pHandle->Init.MasterBusClearMode = I2C_EX_MASTER_BUS_CLEAR_ENABLE;
    pHandle->Init.GeneralCallMode = I2C_EX_GENERALCALL_DISABLE;

	if(HAL_I2C_EX_Init(pHandle) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}
    i2cConfigureTiming(pHandle, pDev->clockSpeed);
#endif

    HAL_NVIC_SetPriority(hardware->er_irq, NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_EV), NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_EV));
    HAL_NVIC_EnableIRQ(hardware->er_irq);
}

#endif
