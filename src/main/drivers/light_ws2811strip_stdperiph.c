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

#include "drivers/io.h"
#include "drivers/nvic.h"

#include "common/color.h"
#include "light_ws2811strip.h"
#include "dma.h"
#include "drivers/system.h"
#include "rcc.h"
#include "timer.h"

#if !defined(WS2811_PIN)
#if defined(STM32F4)
    #define WS2811_PIN                      PA0
    #define WS2811_DMA_HANDLER_IDENTIFER    DMA1_ST2_HANDLER
    #define WS2811_DMA_STREAM               DMA1_Stream2
    #define WS2811_DMA_CHANNEL              DMA_Channel_6
#elif defined(STM32F3) || defined(STM32F1)
    #define WS2811_PIN                      PB8 // TIM16_CH1
    #define WS2811_DMA_STREAM               DMA1_Channel3
    #define WS2811_DMA_HANDLER_IDENTIFER    DMA1_CH3_HANDLER
#endif
#endif


static IO_t ws2811IO = IO_NONE;
bool ws2811Initialised = false;
#if defined(STM32F4)
static DMA_Stream_TypeDef *dmaRef = NULL;
#elif defined(STM32F3) || defined(STM32F1)
static DMA_Channel_TypeDef *dmaRef = NULL;
#else
#error "No MCU definition in light_ws2811strip_stdperiph.c"
#endif
static TIM_TypeDef *timer = NULL;

static void WS2811_DMA_IRQHandler(dmaChannelDescriptor_t *descriptor)
{
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
        ws2811LedDataTransferInProgress = 0;
        DMA_Cmd(descriptor->ref, DISABLE);
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
    }
}

void ws2811LedStripHardwareInit(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    const timerHardware_t *timerHardware = timerGetByTag(IO_TAG(WS2811_PIN), TIM_USE_ANY);

    if (timerHardware == NULL) {
        return;
    }

    timer = timerHardware->tim;

    ws2811IO = IOGetByTag(IO_TAG(WS2811_PIN));
    IOInit(ws2811IO, OWNER_LED_STRIP, RESOURCE_OUTPUT, 0);
#ifdef STM32F1
    IOConfigGPIO(ws2811IO, IO_CONFIG(GPIO_Speed_50MHz, GPIO_Mode_AF_PP));
#else
    IOConfigGPIOAF(ws2811IO, IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_UP), timerHardware->alternateFunction);
#endif

    RCC_ClockCmd(timerRCC(timer), ENABLE);

    // Stop timer
    TIM_Cmd(timer, DISABLE);

    /* Compute the prescaler value */
    uint16_t prescaler = timerGetPrescalerByDesiredMhz(timer, WS2811_TIMER_MHZ);
    uint16_t period = timerGetPeriodByPrescaler(timer, prescaler, WS2811_CARRIER_HZ);

    BIT_COMPARE_1 = period / 3 * 2;
    BIT_COMPARE_0 = period / 3;

    /* Time base configuration */
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = period; // 800kHz
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

    // dmaInit(timerHardware->dmaIrqHandler, OWNER_LED_STRIP, 0);
    // dmaSetHandler(timerHardware->dmaIrqHandler, WS2811_DMA_IRQHandler, NVIC_PRIO_WS2811_DMA, 0);
    //dmaRef = timerHardware->dmaRef;
    dmaRef = WS2811_DMA_STREAM;
    dmaInit(WS2811_DMA_HANDLER_IDENTIFER, OWNER_LED_STRIP, 0);
    dmaSetHandler(WS2811_DMA_HANDLER_IDENTIFER, WS2811_DMA_IRQHandler, NVIC_PRIO_WS2811_DMA, 0);

    DMA_DeInit(dmaRef);

    /* configure DMA */
    DMA_Cmd(dmaRef, DISABLE);
    DMA_DeInit(dmaRef);
    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)timerCCR(timer, timerHardware->channel);
    DMA_InitStructure.DMA_BufferSize = WS2811_DMA_BUFFER_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;

#if defined(STM32F4)
    DMA_InitStructure.DMA_Channel = WS2811_DMA_CHANNEL;   //timerHardware->dmaChannel;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)ledStripDMABuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
#elif defined(STM32F3) || defined(STM32F1)
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ledStripDMABuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
#endif
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;

    DMA_Init(dmaRef, &DMA_InitStructure);
    TIM_DMACmd(timer, timerDmaSource(timerHardware->channel), ENABLE);
    DMA_ITConfig(dmaRef, DMA_IT_TC, ENABLE);
    ws2811Initialised = true;
}

void ws2811LedStripDMAEnable(void)
{
    if (!ws2811Initialised)
        return;

    DMA_SetCurrDataCounter(dmaRef, WS2811_DMA_BUFFER_SIZE);  // load number of bytes to be transferred
    TIM_SetCounter(timer, 0);
    TIM_Cmd(timer, ENABLE);
    DMA_Cmd(dmaRef, ENABLE);
}

#endif
