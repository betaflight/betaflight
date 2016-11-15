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
#include "timer_stm32f4xx.h"
#include "io.h"

static IO_t ws2811IO = IO_NONE;
bool ws2811Initialised = false;
static DMA_Stream_TypeDef *stream = NULL;
static TIM_TypeDef *timer = NULL;

static void WS2811_DMA_IRQHandler(dmaChannelDescriptor_t *descriptor)
{
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
        ws2811LedDataTransferInProgress = 0;
        DMA_Cmd(descriptor->stream, DISABLE);
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

    if (timerHardware->dmaStream == NULL) {
        return;
    }

    RCC_ClockCmd(timerRCC(timer), ENABLE);

    ws2811IO = IOGetByTag(ioTag);
    /* GPIOA Configuration: TIM5 Channel 1 as alternate function push-pull */
    IOInit(ws2811IO, OWNER_LED_STRIP, 0);
    IOConfigGPIOAF(ws2811IO, IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_UP), timerHardware->alternateFunction);

    // Stop timer
    TIM_Cmd(timer, DISABLE);

    /* Compute the prescaler value */
    uint16_t prescalerValue = (uint16_t)(SystemCoreClock / timerClockDivisor(timer) / WS2811_TIMER_HZ) - 1;

    /* Time base configuration */
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = WS2811_TIMER_PERIOD; // 800kHz
    TIM_TimeBaseStructure.TIM_Prescaler = prescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(timer, &TIM_TimeBaseStructure);

    /* PWM1 Mode configuration: Channel1 */
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

    timerOCInit(timer, timerHardware->channel, &TIM_OCInitStructure);
    timerOCPreloadConfig(timer, timerHardware->channel, TIM_OCPreload_Enable);

    TIM_CtrlPWMOutputs(timer, ENABLE);
    TIM_ARRPreloadConfig(timer, ENABLE);

    TIM_CCxCmd(timer, timerHardware->channel, TIM_CCx_Enable);
    TIM_Cmd(timer, ENABLE);

    dmaInit(timerHardware->dmaIrqHandler, OWNER_LED_STRIP, 0);
    dmaSetHandler(timerHardware->dmaIrqHandler, WS2811_DMA_IRQHandler, NVIC_PRIO_WS2811_DMA, 0);

    stream = timerHardware->dmaStream;
    /* configure DMA */
    DMA_Cmd(stream, DISABLE);
    DMA_DeInit(stream);
    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_Channel = timerHardware->dmaChannel;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)timerCCR(timer, timerHardware->channel);
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)ledStripDMABuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = WS2811_DMA_BUFFER_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;

    DMA_Init(stream, &DMA_InitStructure);
    TIM_DMACmd(timer, timerDmaSource(timerHardware->channel), ENABLE);

    DMA_ITConfig(stream, DMA_IT_TC, ENABLE);
    DMA_ClearITPendingBit(stream, dmaFlag_IT_TCIF(stream));

    ws2811Initialised = true;
}

void ws2811LedStripDMAEnable(void)
{
    if (!ws2811Initialised)
        return;

    DMA_SetCurrDataCounter(stream, WS2811_DMA_BUFFER_SIZE);  // load number of bytes to be transferred
    TIM_SetCounter(timer, 0);
    TIM_Cmd(timer, ENABLE);
    DMA_Cmd(stream, ENABLE);
}

#endif
