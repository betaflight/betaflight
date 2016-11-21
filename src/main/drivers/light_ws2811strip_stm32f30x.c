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

#ifdef LED_STRIP

#include "io.h"
#include "nvic.h"

#include "common/color.h"
#include "light_ws2811strip.h"
#include "dma.h"
#include "rcc.h"
#include "timer.h"

static IO_t ws2811IO = IO_NONE;
bool ws2811Initialised = false;
static DMA_Channel_TypeDef *dmaChannel = NULL;
static TIM_TypeDef *timer = NULL;

static void WS2811_DMA_IRQHandler(dmaChannelDescriptor_t *descriptor)
{
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
        ws2811LedDataTransferInProgress = 0;
        DMA_Cmd(descriptor->channel, DISABLE);
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
    }
}

void ws2811LedStripHardwareInit(ioTag_t ioTag)
{
    if (!ioTag) {
        return;
    }

    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    const timerHardware_t *timerHardware = timerGetByTag(ioTag, TIM_USE_ANY);
    timer = timerHardware->tim;

    if (timerHardware->dmaChannel == NULL) {
        return;
    }

    ws2811IO = IOGetByTag(ioTag);
    IOInit(ws2811IO, OWNER_LED_STRIP, 0);
    IOConfigGPIOAF(ws2811IO, IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_UP), timerHardware->alternateFunction);

    dmaInit(timerHardware->dmaIrqHandler, OWNER_LED_STRIP, 0);
    dmaSetHandler(timerHardware->dmaIrqHandler, WS2811_DMA_IRQHandler, NVIC_PRIO_WS2811_DMA, 0);
    RCC_ClockCmd(timerRCC(timer), ENABLE);

    /* Compute the prescaler value */
    uint16_t prescalerValue = (uint16_t) (SystemCoreClock / WS2811_TIMER_HZ) - 1;

    /* Time base configuration */
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = WS2811_TIMER_PERIOD; // 800kHz
    TIM_TimeBaseStructure.TIM_Prescaler = prescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(timer, &TIM_TimeBaseStructure);

    /* PWM1 Mode configuration */
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    if (timerHardware->output & TIMER_OUTPUT_N_CHANNEL) {
        TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
        TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    } else {
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    }
    TIM_OCInitStructure.TIM_OCPolarity =  (timerHardware->output & TIMER_OUTPUT_INVERTED) ? TIM_OCPolarity_Low : TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC1Init(timer, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(timer, TIM_OCPreload_Enable);

    TIM_CtrlPWMOutputs(timer, ENABLE);

    /* configure DMA */
    /* DMA1 Channel Config */
    dmaChannel = timerHardware->dmaChannel;
    DMA_DeInit(dmaChannel);

    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)timerCCR(timer, timerHardware->channel);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ledStripDMABuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = WS2811_DMA_BUFFER_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

    DMA_Init(dmaChannel, &DMA_InitStructure);

    TIM_DMACmd(timer, timerDmaSource(timerHardware->channel), ENABLE);

    DMA_ITConfig(dmaChannel, DMA_IT_TC, ENABLE);

    ws2811Initialised = true;
}

void ws2811LedStripDMAEnable(void)
{
    if (!ws2811Initialised)
        return;

    DMA_SetCurrDataCounter(dmaChannel, WS2811_DMA_BUFFER_SIZE);  // load number of bytes to be transferred
    TIM_SetCounter(timer, 0);
    TIM_Cmd(timer, ENABLE);
    DMA_Cmd(dmaChannel, ENABLE);
}

#endif
