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

#ifdef LED_STRIP

#include "common/color.h"
#include "light_ws2811strip.h"
#include "nvic.h"
#include "dma.h"
#include "io.h"
#include "system.h"
#include "rcc.h"
#include "timer.h"

#if !defined(WS2811_PIN)
#define WS2811_PIN                      PA0
#define WS2811_TIMER                    TIM5
#define WS2811_DMA_HANDLER_IDENTIFER    DMA1_ST2_HANDLER
#define WS2811_DMA_STREAM               DMA1_Stream2
#define WS2811_DMA_IT                   DMA_IT_TCIF2
#define WS2811_DMA_CHANNEL              DMA_Channel_6
#define WS2811_TIMER_CHANNEL            TIM_Channel_1
#endif

static IO_t ws2811IO = IO_NONE;
static uint16_t timDMASource = 0;
bool ws2811Initialised = false;

static void WS2811_DMA_IRQHandler(dmaChannelDescriptor_t *descriptor)
{
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
        ws2811LedDataTransferInProgress = 0;
        DMA_Cmd(descriptor->stream, DISABLE);
        TIM_DMACmd(WS2811_TIMER, timDMASource, DISABLE);
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
    }
}

void ws2811LedStripHardwareInit(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    uint16_t prescalerValue;

    RCC_ClockCmd(timerRCC(WS2811_TIMER), ENABLE);

    ws2811IO = IOGetByTag(IO_TAG(WS2811_PIN));
    /* GPIOA Configuration: TIM5 Channel 1 as alternate function push-pull */
    IOInit(ws2811IO, OWNER_LED_STRIP, RESOURCE_OUTPUT, 0);
    IOConfigGPIOAF(ws2811IO, IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_UP), timerGPIOAF(WS2811_TIMER));

    // Stop timer
    TIM_Cmd(WS2811_TIMER, DISABLE);

    /* Compute the prescaler value */
    prescalerValue = (uint16_t)(SystemCoreClock / 2 / 84000000) - 1;

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 104; // 800kHz
    TIM_TimeBaseStructure.TIM_Prescaler = prescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(WS2811_TIMER, &TIM_TimeBaseStructure);

    /* PWM1 Mode configuration: Channel1 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse = 0;

    uint32_t channelAddress = 0;
    switch (WS2811_TIMER_CHANNEL) {
        case TIM_Channel_1:
            TIM_OC1Init(WS2811_TIMER, &TIM_OCInitStructure);
            timDMASource = TIM_DMA_CC1;
            channelAddress = (uint32_t)(&WS2811_TIMER->CCR1);
            TIM_OC1PreloadConfig(WS2811_TIMER, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_2:
            TIM_OC2Init(WS2811_TIMER, &TIM_OCInitStructure);
            timDMASource = TIM_DMA_CC2;
            channelAddress = (uint32_t)(&WS2811_TIMER->CCR2);
            TIM_OC2PreloadConfig(WS2811_TIMER, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_3:
            TIM_OC3Init(WS2811_TIMER, &TIM_OCInitStructure);
            timDMASource = TIM_DMA_CC3;
            channelAddress = (uint32_t)(&WS2811_TIMER->CCR3);
            TIM_OC3PreloadConfig(WS2811_TIMER, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_4:
            TIM_OC4Init(WS2811_TIMER, &TIM_OCInitStructure);
            timDMASource = TIM_DMA_CC4;
            channelAddress = (uint32_t)(&WS2811_TIMER->CCR4);
            TIM_OC4PreloadConfig(WS2811_TIMER, TIM_OCPreload_Enable);
            break;
    }

    TIM_CtrlPWMOutputs(WS2811_TIMER, ENABLE);
    TIM_ARRPreloadConfig(WS2811_TIMER, ENABLE);

    TIM_CCxCmd(WS2811_TIMER, WS2811_TIMER_CHANNEL, TIM_CCx_Enable);
    TIM_Cmd(WS2811_TIMER, ENABLE);

    /* configure DMA */
    DMA_Cmd(WS2811_DMA_STREAM, DISABLE);
    DMA_DeInit(WS2811_DMA_STREAM);
    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_Channel = WS2811_DMA_CHANNEL;
    DMA_InitStructure.DMA_PeripheralBaseAddr = channelAddress;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)ledStripDMABuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = WS2811_DMA_BUFFER_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

    DMA_Init(WS2811_DMA_STREAM, &DMA_InitStructure);

    DMA_ITConfig(WS2811_DMA_STREAM, DMA_IT_TC, ENABLE);
    DMA_ClearITPendingBit(WS2811_DMA_STREAM, WS2811_DMA_IT);

    dmaSetHandler(WS2811_DMA_HANDLER_IDENTIFER, WS2811_DMA_IRQHandler, NVIC_PRIO_WS2811_DMA, 0);

    const hsvColor_t hsv_white = {  0, 255, 255};
    ws2811Initialised = true;
    setStripColor(&hsv_white);
    ws2811UpdateStrip();
}

void ws2811LedStripDMAEnable(void)
{
    if (!ws2811Initialised)
        return;

    DMA_SetCurrDataCounter(WS2811_DMA_STREAM, WS2811_DMA_BUFFER_SIZE);  // load number of bytes to be transferred
    TIM_SetCounter(WS2811_TIMER, 0);
    DMA_Cmd(WS2811_DMA_STREAM, ENABLE);
    TIM_DMACmd(WS2811_TIMER, timDMASource, ENABLE);
}

#endif
