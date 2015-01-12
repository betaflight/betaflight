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

#include "platform.h"

#include "gpio.h"
#include "nvic.h"

#include "common/color.h"
#include "drivers/light_ws2811strip.h"

#ifndef WS2811_GPIO
#define USE_LED_STRIP_ON_DMA1_CHANNEL3
#define WS2811_GPIO                     GPIOB
#define WS2811_GPIO_AHB_PERIPHERAL      RCC_AHBPeriph_GPIOB
#define WS2811_GPIO_AF                  GPIO_AF_1
#define WS2811_PIN                      GPIO_Pin_8 // TIM16_CH1
#define WS2811_PIN_SOURCE               GPIO_PinSource8
#define WS2811_TIMER                    TIM16
#define WS2811_TIMER_APB2_PERIPHERAL    RCC_APB2Periph_TIM16
#define WS2811_DMA_CHANNEL              DMA1_Channel3
#define WS2811_IRQ                      DMA1_Channel3_IRQn
#endif

void ws2811LedStripHardwareInit(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    uint16_t prescalerValue;

    RCC_AHBPeriphClockCmd(WS2811_GPIO_AHB_PERIPHERAL, ENABLE);

    GPIO_PinAFConfig(WS2811_GPIO, WS2811_PIN_SOURCE,  WS2811_GPIO_AF);

    /* Configuration alternate function push-pull */
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = WS2811_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(WS2811_GPIO, &GPIO_InitStructure);


    RCC_APB2PeriphClockCmd(WS2811_TIMER_APB2_PERIPHERAL, ENABLE);

    /* Compute the prescaler value */
    prescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;
    /* Time base configuration */
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = 29; // 800kHz
    TIM_TimeBaseStructure.TIM_Prescaler = prescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(WS2811_TIMER, &TIM_TimeBaseStructure);

    /* PWM1 Mode configuration */
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(WS2811_TIMER, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(WS2811_TIMER, TIM_OCPreload_Enable);


    TIM_CtrlPWMOutputs(WS2811_TIMER, ENABLE);

    /* configure DMA */
    /* DMA clock enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    /* DMA1 Channel Config */
    DMA_DeInit(WS2811_DMA_CHANNEL);

    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&WS2811_TIMER->CCR1;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ledStripDMABuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = WS2811_DMA_BUFFER_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

    DMA_Init(WS2811_DMA_CHANNEL, &DMA_InitStructure);

    TIM_DMACmd(WS2811_TIMER, TIM_DMA_CC1, ENABLE);

    DMA_ITConfig(WS2811_DMA_CHANNEL, DMA_IT_TC, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = WS2811_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_WS2811_DMA);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_WS2811_DMA);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    setStripColor(&hsv_white);
    ws2811UpdateStrip();
}

#ifdef USE_LED_STRIP_ON_DMA1_CHANNEL3
void DMA1_Channel3_IRQHandler(void)
{
    if (DMA_GetFlagStatus(DMA1_FLAG_TC3)) {
        ws2811LedDataTransferInProgress = 0;
        DMA_Cmd(DMA1_Channel3, DISABLE);            // disable DMA channel
        DMA_ClearFlag(DMA1_FLAG_TC3);               // clear DMA1 Channel transfer complete flag
    }
}
#endif

#ifdef USE_LED_STRIP_ON_DMA1_CHANNEL2
void DMA1_Channel2_IRQHandler(void)
{
    if (DMA_GetFlagStatus(DMA1_FLAG_TC2)) {
        ws2811LedDataTransferInProgress = 0;
        DMA_Cmd(DMA1_Channel2, DISABLE);            // disable DMA channel
        DMA_ClearFlag(DMA1_FLAG_TC2);               // clear DMA1 Channel transfer complete flag
    }
}
#endif

#ifdef USE_LED_STRIP_ON_DMA1_CHANNEL7
void DMA1_Channel7_IRQHandler(void)
{
    if (DMA_GetFlagStatus(DMA1_FLAG_TC7)) {
        ws2811LedDataTransferInProgress = 0;
        DMA_Cmd(DMA1_Channel7, DISABLE);            // disable DMA channel
        DMA_ClearFlag(DMA1_FLAG_TC7);               // clear DMA1 Channel transfer complete flag
    }
}
#endif

void ws2811LedStripDMAEnable(void)
{
    DMA_SetCurrDataCounter(WS2811_DMA_CHANNEL, WS2811_DMA_BUFFER_SIZE);  // load number of bytes to be transferred
    TIM_SetCounter(WS2811_TIMER, 0);
    TIM_Cmd(WS2811_TIMER, ENABLE);
    DMA_Cmd(WS2811_DMA_CHANNEL, ENABLE);
}


