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

#include <platform.h>

#include "drivers/gpio.h"
#include "drivers/transponder_ir.h"
#include "drivers/nvic.h"

#ifndef TRANSPONDER_GPIO
#define USE_TRANSPONDER_ON_DMA1_CHANNEL3
#define TRANSPONDER_GPIO                     GPIOB
#define TRANSPONDER_GPIO_AHB_PERIPHERAL      RCC_AHBPeriph_GPIOB
#define TRANSPONDER_GPIO_AF                  GPIO_AF_1
#define TRANSPONDER_PIN                      GPIO_Pin_8 // TIM16_CH1
#define TRANSPONDER_PIN_SOURCE               GPIO_PinSource8
#define TRANSPONDER_TIMER                    TIM16
#define TRANSPONDER_TIMER_APB2_PERIPHERAL    RCC_APB2Periph_TIM16
#define TRANSPONDER_DMA_CHANNEL              DMA1_Channel3
#define TRANSPONDER_IRQ                      DMA1_Channel3_IRQn
#define TRANSPONDER_DMA_TC_FLAG              DMA1_FLAG_TC3
#define TRANSPONDER_DMA_HANDLER_IDENTIFER    DMA1_CH3_HANDLER
#endif

void transponderIrHardwareInit(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    RCC_AHBPeriphClockCmd(TRANSPONDER_GPIO_AHB_PERIPHERAL, ENABLE);

    GPIO_PinAFConfig(TRANSPONDER_GPIO, TRANSPONDER_PIN_SOURCE,  TRANSPONDER_GPIO_AF);

    /* Configuration alternate function push-pull */
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = TRANSPONDER_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(TRANSPONDER_GPIO, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(TRANSPONDER_TIMER_APB2_PERIPHERAL, ENABLE);

    /* Time base configuration */
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = 156;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TRANSPONDER_TIMER, &TIM_TimeBaseStructure);

    /* PWM1 Mode configuration: Channel1 */
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
#ifdef TRANSPONDER_INVERTED
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
#else
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
#endif
    TIM_OC1Init(TRANSPONDER_TIMER, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TRANSPONDER_TIMER, TIM_OCPreload_Enable);

    TIM_CtrlPWMOutputs(TRANSPONDER_TIMER, ENABLE);

    /* configure DMA */
    /* DMA clock enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    /* DMA1 Channel6 Config */
    DMA_DeInit(TRANSPONDER_DMA_CHANNEL);

    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&TRANSPONDER_TIMER->CCR1;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)transponderIrDMABuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = TRANSPONDER_DMA_BUFFER_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

    DMA_Init(TRANSPONDER_DMA_CHANNEL, &DMA_InitStructure);

    TIM_DMACmd(TRANSPONDER_TIMER, TIM_DMA_CC1, ENABLE);

    DMA_ITConfig(TRANSPONDER_DMA_CHANNEL, DMA_IT_TC, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = TRANSPONDER_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_TRANSPONDER_DMA);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_TRANSPONDER_DMA);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void transponderIrDMAEnable(void)
{
    DMA_SetCurrDataCounter(TRANSPONDER_DMA_CHANNEL, TRANSPONDER_DMA_BUFFER_SIZE);  // load number of bytes to be transferred
    TIM_SetCounter(TRANSPONDER_TIMER, 0);
    TIM_Cmd(TRANSPONDER_TIMER, ENABLE);
    DMA_Cmd(TRANSPONDER_DMA_CHANNEL, ENABLE);
}

void transponderIrDisable(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    DMA_Cmd(TRANSPONDER_DMA_CHANNEL, DISABLE);
    TIM_Cmd(TRANSPONDER_TIMER, DISABLE);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = TRANSPONDER_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(TRANSPONDER_GPIO, &GPIO_InitStructure);
#ifdef TRANSPONDER_INVERTED
    digitalHi(TRANSPONDER_GPIO, TRANSPONDER_PIN);
#else
    digitalLo(TRANSPONDER_GPIO, TRANSPONDER_PIN);
#endif
}

