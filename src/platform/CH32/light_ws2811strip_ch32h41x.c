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

#ifdef USE_LED_STRIP

#include "build/debug.h"

#include "common/color.h"

#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "platform/dma.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "platform/rcc.h"
#include "drivers/timer.h"
#include "platform/timer.h"

#include "drivers/light_ws2811strip.h"
#include "platform/light_ws2811strip_stm32.h"

static IO_t ws2811IO = IO_NONE;
static dmaResource_t *dmaRef = NULL;
static TIM_TypeDef *timer = NULL;

static inline bool isWs2811Timer32Bit(const TIM_TypeDef *tim)
{
    return tim == TIM9 || tim == TIM10 || tim == TIM11 || tim == TIM12;
}

static void WS2811_DMA_IRQHandler(dmaChannelDescriptor_t *descriptor)
{
#if defined(USE_WS2811_SINGLE_COLOUR)
    static uint32_t counter = 0;
#endif

    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
#if defined(USE_WS2811_SINGLE_COLOUR)
        counter++;
        if (counter == WS2811_LED_STRIP_LENGTH) {
            // Output low for 50us delay
            memset(ledStripDMABuffer, 0, sizeof(ledStripDMABuffer));
            ch32DmaBarrier();
            DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
        } else if (counter == (WS2811_LED_STRIP_LENGTH + WS2811_DELAY_ITERATIONS)) {
            counter = 0;
            xDMA_Cmd(descriptor->ref, DISABLE);
            DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
            ch32DmaBarrier();
            ws2811LedDataTransferInProgress = false;
        } else {
            DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
        }
#else
        xDMA_Cmd(descriptor->ref, DISABLE);
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
        ch32DmaBarrier();
        ws2811LedDataTransferInProgress = false;
#endif
    }
}

bool ws2811LedStripHardwareInit(void)
{
    if (!ledStripIoTag) {
        return false;
    }

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    const timerHardware_t *timerHardware = timerAllocate(ledStripIoTag, OWNER_LED_STRIP, 0);

    if (timerHardware == NULL) {
        return false;
    }

    timer = (TIM_TypeDef *)timerHardware->tim;

#if defined(USE_DMA_SPEC)
    const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByTimer(timerHardware);

    if (dmaSpec == NULL) {
        return false;
    }

    dmaRef = dmaSpec->ref;

#else
    dmaRef = timerHardware->dmaRef;
#endif

    const dmaIdentifier_e dmaId = dmaGetIdentifier(dmaRef);
    if (dmaRef == NULL || !dmaAllocate(dmaId, OWNER_LED_STRIP, 0)) {
        return false;
    }

    ws2811IO = IOGetByTag(ledStripIoTag);
    IOInit(ws2811IO, OWNER_LED_STRIP, 0);

    IOConfigGPIOAF(ws2811IO, IO_CONFIG(DIR_OUT, GPIO_MODE_OUT_AF_PP, GPIO_SPEED_VERY_HIGH,
                                       GPIO_PULL_UP), timerHardware->alternateFunction);

    RCC_ClockCmd(timerRCC(timerHardware->tim), ENABLE);

    // Stop timer
    TIM_Cmd(timer, DISABLE);

    /* Compute the prescaler value */
    uint16_t prescaler = timerGetPrescalerByDesiredMhz(timerHardware->tim, WS2811_TIMER_MHZ);
    uint16_t period = timerGetPeriodByPrescaler(timerHardware->tim, prescaler, WS2811_CARRIER_HZ);
    if (period == 0) {
        return false;
    }

    BIT_COMPARE_1 = period / 3 * 2;
    BIT_COMPARE_0 = period / 3;

    /* Time base configuration */
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = period - 1; // The counter includes both 0 and ARR.
    TIM_TimeBaseStructure.TIM_Prescaler = prescaler;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(timer, &TIM_TimeBaseStructure);

    /* PWM1 Mode configuration */
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;

    if (timerHardware->output & TIMER_OUTPUT_N_CHANNEL) {
        TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
        TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
        TIM_OCInitStructure.TIM_OCNPolarity =
            (timerHardware->output & TIMER_OUTPUT_INVERTED) ? TIM_OCNPolarity_Low : TIM_OCNPolarity_High;
    } else {
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
        TIM_OCInitStructure.TIM_OCPolarity =
            (timerHardware->output & TIMER_OUTPUT_INVERTED) ? TIM_OCPolarity_Low : TIM_OCPolarity_High;
    }

    TIM_OCInitStructure.TIM_Pulse = 0;

    timerOCInit(timer, timerHardware->channel, &TIM_OCInitStructure);
    timerOCPreloadConfig(timer, timerHardware->channel, TIM_OCPreload_Enable);

    TIM_CtrlPWMOutputs(timer, ENABLE);
    TIM_ARRPreloadConfig(timer, ENABLE);

    if (timerHardware->output & TIMER_OUTPUT_N_CHANNEL) {
        TIM_CCxNCmd(timer, timerHardware->channel, TIM_CCxN_Enable);
    } else {
        TIM_CCxCmd(timer, timerHardware->channel, TIM_CCx_Enable);
    }

    TIM_Cmd(timer, ENABLE);

    dmaEnable(dmaId);
    dmaSetHandler(dmaId, WS2811_DMA_IRQHandler, NVIC_PRIO_WS2811_DMA, 0);

    /* configure DMA */
    xDMA_Cmd(dmaRef, DISABLE);
    xDMA_DeInit(dmaRef);
    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)timerCCR(timerHardware->tim, timerHardware->channel);
    DMA_InitStructure.DMA_BufferSize = WS2811_DMA_BUFFER_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;

    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)ledStripDMABuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_PeripheralDataSize = isWs2811Timer32Bit(timer)
        ? DMA_PeripheralDataSize_Word
        : DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

#if defined(USE_WS2811_SINGLE_COLOUR)
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
#else
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
#endif
    xDMA_Init(dmaRef, &DMA_InitStructure);
#if defined(USE_DMA_SPEC)
    dmaMuxEnable(dmaId, dmaSpec->dmaMuxId);
#else
    #warning "USE_DMA_SPEC DETECT FAIL IN LEDSTRIP CH32H41x NEED DMA MUX "
#endif
    dmaChannelDescriptor_t *descriptor = dmaGetDescriptorByIdentifier(dmaId);
    if (descriptor) {
        DMA_CLEAR_FLAG(descriptor, DMA_IT_GLOB | DMA_IT_TCIF | DMA_IT_HTIF | DMA_IT_TEIF);
        ch32DmaBarrier();
        NVIC_ClearPendingIRQ(descriptor->irqN);
    }
    TIM_DMACmd(timer, timerDmaSource(timerHardware->channel), ENABLE);
    xDMA_ITConfig(dmaRef, DMA_IT_TC, ENABLE);
    return true;
}

void ws2811LedStripStartTransfer(void)
{
    TIM_Cmd(timer, DISABLE);
    xDMA_Cmd(dmaRef, DISABLE);
    xDMA_SetCurrDataCounter(dmaRef, WS2811_DMA_BUFFER_SIZE);
    TIM_SetCounter(timer, 0);
    ch32DmaBarrier();
    xDMA_Cmd(dmaRef, ENABLE);
    TIM_Cmd(timer, ENABLE);
}
#endif
