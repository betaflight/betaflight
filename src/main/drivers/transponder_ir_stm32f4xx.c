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

#include "io.h"
#include "nvic.h"

#include "dma.h"
#include "rcc.h"
#include "timer.h"
#include "timer_stm32f4xx.h"

#include "transponder_ir.h"

static IO_t transponderIO = IO_NONE;
static DMA_Stream_TypeDef *stream = NULL;
static TIM_TypeDef *timer = NULL;

static void TRANSPONDER_DMA_IRQHandler(dmaChannelDescriptor_t* descriptor)
{
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
        transponderIrDataTransferInProgress = 0;
        DMA_Cmd(descriptor->stream, DISABLE);
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
    }
}


void transponderIrHardwareInit(ioTag_t ioTag)
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

    transponderIO = IOGetByTag(ioTag);
    IOInit(transponderIO, OWNER_TRANSPONDER, 0);
    IOConfigGPIOAF(transponderIO, IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_DOWN), timerHardware->alternateFunction);

    dmaInit(timerHardware->dmaIrqHandler, OWNER_TRANSPONDER, 0);
    dmaSetHandler(timerHardware->dmaIrqHandler, TRANSPONDER_DMA_IRQHandler, NVIC_PRIO_TRANSPONDER_DMA, 0);

    RCC_ClockCmd(timerRCC(timer), ENABLE);

    /* Time base configuration */
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = 156;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
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

    /* configure DMA */
    stream = timerHardware->dmaStream;
    DMA_Cmd(stream, DISABLE);
    DMA_DeInit(stream);

    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)timerCCR(timer, timerHardware->channel);
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)transponderIrDMABuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = TRANSPONDER_DMA_BUFFER_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;

    DMA_Init(stream, &DMA_InitStructure);

    TIM_DMACmd(timer, timerDmaSource(timerHardware->channel), ENABLE);

    DMA_ITConfig(stream, DMA_IT_TC, ENABLE);
}

void transponderIrDMAEnable(void)
{
    DMA_SetCurrDataCounter(stream, TRANSPONDER_DMA_BUFFER_SIZE);  // load number of bytes to be transferred
    TIM_SetCounter(timer, 0);
    TIM_Cmd(timer, ENABLE);
    DMA_Cmd(stream, ENABLE);
}

void transponderIrDisable(void)
{
    DMA_Cmd(stream, DISABLE);
    TIM_Cmd(timer, DISABLE);

    IOInit(transponderIO, OWNER_TRANSPONDER, 0);
    IOConfigGPIOAF(transponderIO, IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_DOWN), timerHardware->alternateFunction);

#ifdef TRANSPONDER_INVERTED
    IOHi(transponderIO);
#else
    IOLo(transponderIO);
#endif
}

