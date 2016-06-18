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
#include <string.h>

#include <platform.h>
#include "build/build_config.h"
#include "build/debug.h"

#include "drivers/nvic.h"

#include "drivers/system.h"
#include "drivers/gpio.h"
#include "drivers/exti.h"

static const extiConfig_t spRacingF3NEOInterconnect1ExtiConfig = {
    .gpioAHBPeripherals = RCC_AHBPeriph_GPIOC,
    .gpioPort = GPIOC,
    .gpioPin = Pin_4,
    .exti_port_source = EXTI_PortSourceGPIOC,
    .exti_pin_source = EXTI_PinSource4,
    .exti_line = EXTI_Line4,
    .exti_irqn = EXTI4_IRQn
};

static const extiConfig_t spRacingF3NEOInterconnect2ExtiConfig = {
    .gpioAHBPeripherals = RCC_AHBPeriph_GPIOC,
    .gpioPort = GPIOC,
    .gpioPin = Pin_5,
    .exti_port_source = EXTI_PortSourceGPIOC,
    .exti_pin_source = EXTI_PinSource5,
    .exti_line = EXTI_Line5,
    .exti_irqn = EXTI9_5_IRQn
};

void configureBoardInterconnect(const extiConfig_t *extiConfig, extiCallbackHandlerFunc *fn, EXTITrigger_TypeDef trigger)
{
    gpio_config_t gpio;

    if (extiConfig->gpioAHBPeripherals) {
        RCC_AHBPeriphClockCmd(extiConfig->gpioAHBPeripherals, ENABLE);
    }

    gpio.pin = extiConfig->gpioPin;
    gpio.speed = Speed_2MHz;
    gpio.mode = Mode_IN_FLOATING;
    gpioInit(extiConfig->gpioPort, &gpio);

    /* Enable SYSCFG clock otherwise the EXTI irq handlers are not called */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    gpioExtiLineConfig(extiConfig->exti_port_source, extiConfig->exti_pin_source);

    registerExtiCallbackHandler(extiConfig->exti_irqn, fn);

    EXTI_ClearITPendingBit(extiConfig->exti_line);

    EXTI_InitTypeDef EXTIInit;
    EXTIInit.EXTI_Line = extiConfig->exti_line;
    EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTIInit.EXTI_Trigger = trigger;
    EXTIInit.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTIInit);

    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = extiConfig->exti_irqn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_EXT_DATA_READY);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_EXT_DATA_READY);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

typedef struct extDataState_s {
    uint32_t ext1DataReadyCounter;
    uint32_t ext2DataReadyCounter;
} extDataState_t;
static extDataState_t extDataState = { 0, 0 };

void EXT1_READY_EXTI_Handler(void)
{
    if (EXTI_GetITStatus(spRacingF3NEOInterconnect1ExtiConfig.exti_line) == RESET) {
        return;
    }

    EXTI_ClearITPendingBit(spRacingF3NEOInterconnect1ExtiConfig.exti_line);

    extDataState.ext1DataReadyCounter++;
    debug[0] = extDataState.ext1DataReadyCounter;
}


void EXT2_READY_EXTI_Handler(void)
{
    if (EXTI_GetITStatus(spRacingF3NEOInterconnect2ExtiConfig.exti_line) == RESET) {
        return;
    }

    EXTI_ClearITPendingBit(spRacingF3NEOInterconnect2ExtiConfig.exti_line);

    extDataState.ext2DataReadyCounter++;
    debug[1] = extDataState.ext2DataReadyCounter;
}

void configureBoardInterconnects(void)
{
    /*
     * The SP Racing F3 NEO has 2 GPIOs connected to a stack connector.  The GPIOs
     */
    configureBoardInterconnect(&spRacingF3NEOInterconnect1ExtiConfig, EXT1_READY_EXTI_Handler, EXTI_Trigger_Rising);
    configureBoardInterconnect(&spRacingF3NEOInterconnect2ExtiConfig, EXT2_READY_EXTI_Handler, EXTI_Trigger_Rising);
}
