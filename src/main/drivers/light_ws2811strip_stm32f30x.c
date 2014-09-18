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

#include "common/color.h"
#include "drivers/light_ws2811strip.h"

#define WS2811_GPIO   GPIOB
#define WS2811_PIN    Pin_8 // TIM16_CH1
#define WS2811_PERIPHERAL RCC_AHBPeriph_GPIOB


void ws2811LedStripHardwareInit(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    uint16_t prescalerValue;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8,  GPIO_AF_1);

    /* GPIOA Configuration: TIM16 Channel 1 as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = WS2811_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(WS2811_GPIO, &GPIO_InitStructure);


    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16, ENABLE);
    /* Compute the prescaler value */
    prescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;
    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 29; // 800kHz
    TIM_TimeBaseStructure.TIM_Prescaler = prescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM16, &TIM_TimeBaseStructure);

    /* PWM1 Mode configuration: Channel1 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM16, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);


    TIM_CtrlPWMOutputs(TIM16, ENABLE);

    /* configure DMA */
    /* DMA clock enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    /* DMA1 Channel Config */
    DMA_DeInit(DMA1_Channel3);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&TIM16->CCR1;
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

    DMA_Init(DMA1_Channel3, &DMA_InitStructure);

    /* TIM16 CC1 DMA Request enable */
    TIM_DMACmd(TIM16, TIM_DMA_CC1, ENABLE);

    DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    setStripColor(&hsv_white);
    ws2811UpdateStrip();
}

void DMA1_Channel3_IRQHandler(void)
{
    if (DMA_GetFlagStatus(DMA1_FLAG_TC3)) {
        ws2811LedDataTransferInProgress = 0;
        DMA_Cmd(DMA1_Channel3, DISABLE);            // disable DMA channel 6
        DMA_ClearFlag(DMA1_FLAG_TC3);               // clear DMA1 Channel 6 transfer complete flag
    }
}

void ws2811LedStripDMAEnable(void)
{
    DMA_SetCurrDataCounter(DMA1_Channel3, WS2811_DMA_BUFFER_SIZE);  // load number of bytes to be transferred
    TIM_SetCounter(TIM16, 0);
    TIM_Cmd(TIM16, ENABLE);
    DMA_Cmd(DMA1_Channel3, ENABLE);
}


