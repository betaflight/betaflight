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

#if defined(USE_I2C) && !defined(USE_SOFT_I2C) && !defined(USE_I3C_AS_I2C)

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/time.h"
#include "platform/io_impl.h"
#include "platform/rcc.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_impl.h"
#include "drivers/bus_i2c_timing.h"
#include "drivers/bus_i2c_utils.h"

#define IOCFG_I2C_PU IO_CONFIG(GPIO_MODE_AF_OD, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLUP)
#define IOCFG_I2C    IO_CONFIG(GPIO_MODE_AF_OD, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL)

#define GPIO_AF4_I2C GPIO_AF4_I2C1

const i2cHardware_t i2cHardware[I2CDEV_COUNT] = {
#if defined(STM32F7)
#ifdef USE_I2C_DEVICE_1
    {
        .device = I2CDEV_1,
        .reg = (i2cResource_t *)I2C1,
        .sclPins = { I2CPINDEF(PB6), I2CPINDEF(PB8) },
        .sdaPins = { I2CPINDEF(PB7), I2CPINDEF(PB9) },
        .rcc = RCC_APB1(I2C1),
        .ev_irq = I2C1_EV_IRQn,
        .er_irq = I2C1_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_2
    {
        .device = I2CDEV_2,
        .reg = (i2cResource_t *)I2C2,
        .sclPins = { I2CPINDEF(PB10), I2CPINDEF(PF1) },
        .sdaPins = { I2CPINDEF(PB11), I2CPINDEF(PF0) },
        .rcc = RCC_APB1(I2C2),
        .ev_irq = I2C2_EV_IRQn,
        .er_irq = I2C2_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_3
    {
        .device = I2CDEV_3,
        .reg = (i2cResource_t *)I2C3,
        .sclPins = { I2CPINDEF(PA8) },
        .sdaPins = { I2CPINDEF(PC9) },
        .rcc = RCC_APB1(I2C3),
        .ev_irq = I2C3_EV_IRQn,
        .er_irq = I2C3_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_4
    {
        .device = I2CDEV_4,
        .reg = (i2cResource_t *)I2C4,
        .sclPins = { I2CPINDEF(PD12), I2CPINDEF(PF14) },
        .sdaPins = { I2CPINDEF(PD13), I2CPINDEF(PF15) },
        .rcc = RCC_APB1(I2C4),
        .ev_irq = I2C4_EV_IRQn,
        .er_irq = I2C4_ER_IRQn,
    },
#endif
#elif defined(STM32H7)
#ifdef USE_I2C_DEVICE_1
    {
        .device = I2CDEV_1,
        .reg = (i2cResource_t *)I2C1,
        .sclPins = { I2CPINDEF(PB6, GPIO_AF4_I2C1), I2CPINDEF(PB8, GPIO_AF4_I2C1) },
        .sdaPins = { I2CPINDEF(PB7, GPIO_AF4_I2C1), I2CPINDEF(PB9, GPIO_AF4_I2C1) },
        .rcc = RCC_APB1L(I2C1),
        .ev_irq = I2C1_EV_IRQn,
        .er_irq = I2C1_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_2
    {
        .device = I2CDEV_2,
        .reg = (i2cResource_t *)I2C2,
        .sclPins = { I2CPINDEF(PB10, GPIO_AF4_I2C2), I2CPINDEF(PF1, GPIO_AF4_I2C2) },
        .sdaPins = { I2CPINDEF(PB11, GPIO_AF4_I2C2), I2CPINDEF(PF0, GPIO_AF4_I2C2) },
        .rcc = RCC_APB1L(I2C2),
        .ev_irq = I2C2_EV_IRQn,
        .er_irq = I2C2_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_3
    {
        .device = I2CDEV_3,
        .reg = (i2cResource_t *)I2C3,
        .sclPins = { I2CPINDEF(PA8, GPIO_AF4_I2C3) },
        .sdaPins = { I2CPINDEF(PC9, GPIO_AF4_I2C3) },
        .rcc = RCC_APB1L(I2C3),
        .ev_irq = I2C3_EV_IRQn,
        .er_irq = I2C3_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_4
    {
        .device = I2CDEV_4,
        .reg = (i2cResource_t *)I2C4,
        .sclPins = { I2CPINDEF(PD12, GPIO_AF4_I2C4), I2CPINDEF(PF14, GPIO_AF4_I2C4), I2CPINDEF(PB6, GPIO_AF6_I2C4), I2CPINDEF(PB8, GPIO_AF6_I2C4) },
        .sdaPins = { I2CPINDEF(PD13, GPIO_AF4_I2C4), I2CPINDEF(PF15, GPIO_AF4_I2C4), I2CPINDEF(PB7, GPIO_AF6_I2C4), I2CPINDEF(PB9, GPIO_AF6_I2C4) },
        .rcc = RCC_APB4(I2C4),
        .ev_irq = I2C4_EV_IRQn,
        .er_irq = I2C4_ER_IRQn,
    },
#endif
#elif defined(STM32G4)
#ifdef USE_I2C_DEVICE_1
    {
        .device = I2CDEV_1,
        .reg = (i2cResource_t *)I2C1,

        // Some boards are overloading SWD pins with I2C1 for maximum pin utilization on 48-pin CE(U) packages.
        // Be carefull when using SWD on these boards if I2C1 pins are defined by default.

        .sclPins = { I2CPINDEF(PA13, GPIO_AF4_I2C1), I2CPINDEF(PA15, GPIO_AF4_I2C1), I2CPINDEF(PB6,  GPIO_AF4_I2C1), I2CPINDEF(PB8,  GPIO_AF4_I2C1), },
        .sdaPins = { I2CPINDEF(PA14, GPIO_AF4_I2C1), I2CPINDEF(PB7,  GPIO_AF4_I2C1), I2CPINDEF(PB9,  GPIO_AF4_I2C1), },
        .rcc = RCC_APB11(I2C1),
        .ev_irq = I2C1_EV_IRQn,
        .er_irq = I2C1_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_2
    {
        .device = I2CDEV_2,
        .reg = (i2cResource_t *)I2C2,
        .sclPins = { I2CPINDEF(PA9,  GPIO_AF4_I2C2) },
        .sdaPins = { I2CPINDEF(PA8, GPIO_AF4_I2C2), I2CPINDEF(PF6, GPIO_AF4_I2C2) },
        .rcc = RCC_APB11(I2C2),
        .ev_irq = I2C2_EV_IRQn,
        .er_irq = I2C2_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_3
    {
        .device = I2CDEV_3,
        .reg = (i2cResource_t *)I2C3,
        .sclPins = { I2CPINDEF(PA10, GPIO_AF2_I2C3), I2CPINDEF(PC8,  GPIO_AF8_I2C3), },
        .sdaPins = { I2CPINDEF(PB5,  GPIO_AF8_I2C3), I2CPINDEF(PC9,  GPIO_AF8_I2C3), I2CPINDEF(PC11, GPIO_AF8_I2C3), },
        .rcc = RCC_APB11(I2C3),
        .ev_irq = I2C3_EV_IRQn,
        .er_irq = I2C3_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_4
    {
        .device = I2CDEV_4,
        .reg = (i2cResource_t *)I2C4,

        // Here, SWDIO(PA13) is overloaded with I2C4_SCL, too.
        // See comment in the I2C1 section above.

        .sclPins = { I2CPINDEF(PA13, GPIO_AF3_I2C4), I2CPINDEF(PB6,  GPIO_AF3_I2C4), I2CPINDEF(PC6,  GPIO_AF8_I2C4), },
        .sdaPins = { I2CPINDEF(PB7,  GPIO_AF4_I2C4), I2CPINDEF(PC7,  GPIO_AF8_I2C4), },
        .rcc = RCC_APB12(I2C4),
        .ev_irq = I2C4_EV_IRQn,
        .er_irq = I2C4_ER_IRQn,
    },
#endif
#elif defined(STM32H5)
#ifdef USE_I2C_DEVICE_1
    {
        .device = I2CDEV_1,
        .reg = (i2cResource_t *)I2C1,
        .sclPins = { I2CPINDEF(PB6, GPIO_AF4_I2C1), I2CPINDEF(PB8, GPIO_AF4_I2C1) },
        .sdaPins = { I2CPINDEF(PB7, GPIO_AF4_I2C1), I2CPINDEF(PB9, GPIO_AF4_I2C1) },
        .rcc = RCC_APB1L(I2C1),
        .ev_irq = I2C1_EV_IRQn,
        .er_irq = I2C1_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_2
    {
        .device = I2CDEV_2,
        .reg = (i2cResource_t *)I2C2,
        .sclPins = { I2CPINDEF(PB10, GPIO_AF4_I2C2), I2CPINDEF(PF1, GPIO_AF4_I2C2) },
        .sdaPins = { I2CPINDEF(PB11, GPIO_AF4_I2C2), I2CPINDEF(PF0, GPIO_AF4_I2C2) },
        .rcc = RCC_APB1L(I2C2),
        .ev_irq = I2C2_EV_IRQn,
        .er_irq = I2C2_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_3
    {
        .device = I2CDEV_3,
        .reg = (i2cResource_t *)I2C3,
        .sclPins = { I2CPINDEF(PA8, GPIO_AF4_I2C3) },
        .sdaPins = { I2CPINDEF(PC9, GPIO_AF4_I2C3) },
        .rcc = RCC_APB3(I2C3),
        .ev_irq = I2C3_EV_IRQn,
        .er_irq = I2C3_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_4
    {
        .device = I2CDEV_4,
        .reg = (i2cResource_t *)I2C4,
        .sclPins = { I2CPINDEF(PD12, GPIO_AF4_I2C4), I2CPINDEF(PF14, GPIO_AF4_I2C4), I2CPINDEF(PB6, GPIO_AF6_I2C4), I2CPINDEF(PB8, GPIO_AF6_I2C4) },
        .sdaPins = { I2CPINDEF(PD13, GPIO_AF4_I2C4), I2CPINDEF(PF15, GPIO_AF4_I2C4), I2CPINDEF(PB7, GPIO_AF6_I2C4), I2CPINDEF(PB9, GPIO_AF6_I2C4) },
        .rcc = RCC_APB3(I2C4),
        .ev_irq = I2C4_EV_IRQn,
        .er_irq = I2C4_ER_IRQn,
    },
#endif
#elif defined(STM32N6)
#ifdef USE_I2C_DEVICE_1
    {
        .device = I2CDEV_1,
        .reg = (i2cResource_t *)I2C1,
        .sclPins = { I2CPINDEF(PE5, GPIO_AF4_I2C1), I2CPINDEF(PH9, GPIO_AF4_I2C1) },
        .sdaPins = { I2CPINDEF(PC1, GPIO_AF4_I2C1), I2CPINDEF(PE6, GPIO_AF4_I2C1) },
        .rcc = RCC_APB1(I2C1),
        .ev_irq = I2C1_EV_IRQn,
        .er_irq = I2C1_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_2
    {
        .device = I2CDEV_2,
        .reg = (i2cResource_t *)I2C2,
        .sclPins = { I2CPINDEF(PB10, GPIO_AF4_I2C2), I2CPINDEF(PD14, GPIO_AF4_I2C2) },
        .sdaPins = { I2CPINDEF(PB11, GPIO_AF4_I2C2), I2CPINDEF(PD4, GPIO_AF4_I2C2), I2CPINDEF(PD15, GPIO_AF4_I2C2) },
        .rcc = RCC_APB1(I2C2),
        .ev_irq = I2C2_EV_IRQn,
        .er_irq = I2C2_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_3
    {
        .device = I2CDEV_3,
        .reg = (i2cResource_t *)I2C3,
        .sclPins = { I2CPINDEF(PA8, GPIO_AF4_I2C3) },
        .sdaPins = { I2CPINDEF(PA9, GPIO_AF4_I2C3), I2CPINDEF(PC9, GPIO_AF4_I2C3) },
        .rcc = RCC_APB1(I2C3),
        .ev_irq = I2C3_EV_IRQn,
        .er_irq = I2C3_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_4
    {
        .device = I2CDEV_4,
        .reg = (i2cResource_t *)I2C4,
        .sclPins = { I2CPINDEF(PC10, GPIO_AF4_I2C4), I2CPINDEF(PE13, GPIO_AF4_I2C4) },
        .sdaPins = { I2CPINDEF(PC11, GPIO_AF4_I2C4), I2CPINDEF(PD11, GPIO_AF4_I2C4), I2CPINDEF(PE14, GPIO_AF4_I2C4) },
        .rcc = RCC_APB4(I2C4),
        .ev_irq = I2C4_EV_IRQn,
        .er_irq = I2C4_ER_IRQn,
    },
#endif
#elif defined(STM32C5)
#ifdef USE_I2C_DEVICE_1
    {
        .device = I2CDEV_1,
        .reg = (i2cResource_t *)I2C1,
        .sclPins = { I2CPINDEF(PB6, GPIO_AF4_I2C1), I2CPINDEF(PB8, GPIO_AF4_I2C1) },
        .sdaPins = { I2CPINDEF(PB7, GPIO_AF4_I2C1), I2CPINDEF(PB9, GPIO_AF4_I2C1) },
        .rcc = RCC_APB1L(I2C1),
        .ev_irq = I2C1_EV_IRQn,
        .er_irq = I2C1_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_2
    {
        .device = I2CDEV_2,
        .reg = (i2cResource_t *)I2C2,
        .sclPins = { I2CPINDEF(PB10, GPIO_AF4_I2C2) },
        .sdaPins = { I2CPINDEF(PB11, GPIO_AF4_I2C2) },
        .rcc = RCC_APB1L(I2C2),
        .ev_irq = I2C2_EV_IRQn,
        .er_irq = I2C2_ER_IRQn,
    },
#endif
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

    // Bus-health diagnostic (recorded for CLI status; init continues regardless).
    uint8_t busHealth = I2C_HEALTH_CHECKED;

    // Usability: engage the internal pull-up. An unloaded, functional line
    // reaches HIGH; if it stays LOW it is being held down (short, stuck device,
    // or a non-functional pin). Pull-strategy agnostic — a board relying on the
    // internal pull-up passes this, so it is never mis-flagged.
    IOConfigGPIO(scl, IOCFG_IPU);
    IOConfigGPIO(sda, IOCFG_IPU);
    delayMicroseconds(10);
    if (!IORead(scl)) {
        busHealth |= I2C_HEALTH_SCL_LOW;
    }
    if (!IORead(sda)) {
        busHealth |= I2C_HEALTH_SDA_LOW;
    }

    // External pull-up presence — only meaningful when not relying on the
    // internal pull-up. Internal pull-down engaged: a real external pull-up
    // overrides it (HIGH), its absence reads LOW. Informational only. Skip a
    // line already held low above: it reads low here regardless, so we cannot
    // conclude anything about its pull-up.
    if (!pDev->pullUp) {
        IOConfigGPIO(scl, IOCFG_IPD);
        IOConfigGPIO(sda, IOCFG_IPD);
        delayMicroseconds(10);
        if (!(busHealth & I2C_HEALTH_SCL_LOW) && !IORead(scl)) {
            busHealth |= I2C_HEALTH_SCL_NOPULL;
        }
        if (!(busHealth & I2C_HEALTH_SDA_LOW) && !IORead(sda)) {
            busHealth |= I2C_HEALTH_SDA_NOPULL;
        }
    }

    i2cReportBusHealth(device, busHealth);

    i2cUnstick(scl, sda);

    // Init pins
#if defined(STM32F7)
    IOConfigGPIOAF(scl, pDev->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, GPIO_AF4_I2C);
    IOConfigGPIOAF(sda, pDev->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, GPIO_AF4_I2C);
#elif defined(STM32H7) || defined(STM32H5) || defined(STM32G4) || defined(STM32N6) || defined(STM32C5)
    IOConfigGPIOAF(scl, pDev->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, pDev->sclAF);
    IOConfigGPIOAF(sda, pDev->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, pDev->sdaAF);
#else
    IOConfigGPIO(scl, IOCFG_AF_OD);
    IOConfigGPIO(sda, IOCFG_AF_OD);
#endif

    // Init I2C peripheral using LL

    I2C_TypeDef *I2Cx = (I2C_TypeDef *)pDev->hardware->reg;

    // Reset the I2C state
    memset(&pDev->state, 0, sizeof(pDev->state));

    LL_I2C_Disable(I2Cx);
    LL_I2C_DeInit(I2Cx);

    // Compute TIMINGR value based on peripheral clock for this device instance

    uint32_t i2cPclk;

#if defined(STM32F7) || defined(STM32G4)
    // F7 Clock source configured in startup/stm32/system_stm32f7xx.c as:
    //   I2C1234 : PCLK1
    // G4 Clock source configured in startup/stm32/system_stm32g4xx.c as:
    //   I2C1234 : PCLK1
    i2cPclk = HAL_RCC_GetPCLK1Freq();
#elif defined(STM32H7)
    // Clock sources configured in startup/stm32/system_stm32h7xx.c as:
    //   I2C123 : D2PCLK1 (rcc_pclk1 for APB1)
    //   I2C4   : D3PCLK1 (rcc_pclk4 for APB4)
    i2cPclk = (I2Cx == I2C4) ? HAL_RCCEx_GetD3PCLK1Freq() : HAL_RCC_GetPCLK1Freq();
#elif defined(STM32H5)
    // H5 Clock sources:
    //   I2C12  : PCLK1 (APB1)
    //   I2C34  : PCLK3 (APB3)
    // Explicitly select the kernel clock source rather than trusting the reset
    // default (I2CxSEL=0). The APB clock enable (APB1L/APB3ENR) only gates
    // register access; SCL is generated from the *kernel* clock, so if anything
    // (bootloader, prior state) left I2CxSEL pointing elsewhere the peripheral's
    // registers respond but the bus never clocks. Pin these to the source the
    // TIMINGR below is computed against.
    if (I2Cx == I2C1) {
        __HAL_RCC_I2C1_CONFIG(RCC_I2C1CLKSOURCE_PCLK1);
    } else if (I2Cx == I2C2) {
        __HAL_RCC_I2C2_CONFIG(RCC_I2C2CLKSOURCE_PCLK1);
    } else if (I2Cx == I2C3) {
        __HAL_RCC_I2C3_CONFIG(RCC_I2C3CLKSOURCE_PCLK3);
    } else if (I2Cx == I2C4) {
        __HAL_RCC_I2C4_CONFIG(RCC_I2C4CLKSOURCE_PCLK3);
    }
    i2cPclk = (I2Cx == I2C3 || I2Cx == I2C4) ? HAL_RCC_GetPCLK3Freq() : HAL_RCC_GetPCLK1Freq();
#elif defined(STM32N6)
    // N6 Clock sources:
    //   I2C123 : PCLK1 (APB1)
    //   I2C4   : PCLK4 (APB4)
    i2cPclk = (I2Cx == I2C4) ? HAL_RCC_GetPCLK4Freq() : HAL_RCC_GetPCLK1Freq();
#elif defined(STM32C5)
    // C5 Clock sources:
    //   I2C12 : PCLK1 (APB1)
    i2cPclk = HAL_RCC_GetPCLK1Freq();
#else
#error Unknown MCU type
#endif

    LL_I2C_InitTypeDef i2cInit;
    LL_I2C_StructInit(&i2cInit);

    i2cInit.PeripheralMode = LL_I2C_MODE_I2C;
    i2cInit.Timing = i2cClockTIMINGR(i2cPclk, pDev->clockSpeed, 0);
    i2cInit.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
    i2cInit.DigitalFilter = 0;
    i2cInit.OwnAddress1 = 0;
    i2cInit.TypeAcknowledge = LL_I2C_ACK;
    i2cInit.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;

    LL_I2C_Init(I2Cx, &i2cInit);
    LL_I2C_Enable(I2Cx);

    // Setup interrupt handlers
    HAL_NVIC_SetPriority(hardware->er_irq, NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_ER), NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_ER));
    HAL_NVIC_EnableIRQ(hardware->er_irq);
    HAL_NVIC_SetPriority(hardware->ev_irq, NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_EV), NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_EV));
    HAL_NVIC_EnableIRQ(hardware->ev_irq);
}

#if defined(STM32H5)
// TEMPORARY (Development Instrumentation) — see bus_i2c.h. Read the live post-init
// registers that determine whether a configured I2C bus can actually clock.
bool i2cGetDebugRegs(i2cDevice_e device, i2cDebugRegs_t *regs)
{
    if (device < 0 || device >= I2CDEV_COUNT || !regs) {
        return false;
    }

    const i2cDevice_t *pDev = &i2cDevice[device];
    I2C_TypeDef *I2Cx = (I2C_TypeDef *)pDev->reg;
    if (!I2Cx || !pDev->scl || !pDev->sda) {
        return false;  // bus not configured
    }

    uint8_t selPos;
    bool busClockOn;
    if (I2Cx == I2C1) {
        selPos = RCC_CCIPR4_I2C1SEL_Pos;
        busClockOn = (RCC->APB1LENR & RCC_APB1LENR_I2C1EN) != 0;
    } else if (I2Cx == I2C2) {
        selPos = RCC_CCIPR4_I2C2SEL_Pos;
        busClockOn = (RCC->APB1LENR & RCC_APB1LENR_I2C2EN) != 0;
    } else if (I2Cx == I2C3) {
        selPos = RCC_CCIPR4_I2C3SEL_Pos;
        busClockOn = (RCC->APB3ENR & RCC_APB3ENR_I2C3EN) != 0;
    } else if (I2Cx == I2C4) {
        selPos = RCC_CCIPR4_I2C4SEL_Pos;
        busClockOn = (RCC->APB3ENR & RCC_APB3ENR_I2C4EN) != 0;
    } else {
        return false;
    }

    regs->clkSel = (RCC->CCIPR4 >> selPos) & 0x3;
    regs->busClockOn = busClockOn;
    regs->peEnabled = (I2Cx->CR1 & I2C_CR1_PE) != 0;
    regs->timingr = I2Cx->TIMINGR;

    GPIO_TypeDef *sclGpio = IO_GPIO(pDev->scl);
    const int sclPin = IO_GPIOPinIdx(pDev->scl);
    regs->sclMode = (sclGpio->MODER >> (sclPin * 2)) & 0x3;
    regs->sclAf = (sclGpio->AFR[sclPin >> 3] >> ((sclPin & 7) * 4)) & 0xF;

    GPIO_TypeDef *sdaGpio = IO_GPIO(pDev->sda);
    const int sdaPin = IO_GPIOPinIdx(pDev->sda);
    regs->sdaMode = (sdaGpio->MODER >> (sdaPin * 2)) & 0x3;
    regs->sdaAf = (sdaGpio->AFR[sdaPin >> 3] >> ((sdaPin & 7) * 4)) & 0xF;

    return true;
}
#endif

#endif
