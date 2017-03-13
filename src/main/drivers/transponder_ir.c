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
#include <string.h>

#include <platform.h>

#ifdef TRANSPONDER

#include "dma.h"
#include "nvic.h"
#include "io.h"
#include "rcc.h"
#include "timer.h"

#include "transponder_ir.h"

#define TRANSPONDER_TIMER_MHZ       24
#define TRANSPONDER_CARRIER_HZ      460750

static uint8_t bitToggleOne = 0;
#define BIT_TOGGLE_0 0

#define TRANSPONDER_BITS_PER_BYTE 10 // start + 8 data + stop
#define TRANSPONDER_DATA_LENGTH 6
#define TRANSPONDER_TOGGLES_PER_BIT 11
#define TRANSPONDER_GAP_TOGGLES 1
#define TRANSPONDER_TOGGLES (TRANSPONDER_TOGGLES_PER_BIT + TRANSPONDER_GAP_TOGGLES)

#define TRANSPONDER_DMA_BUFFER_SIZE ((TRANSPONDER_TOGGLES_PER_BIT + 1) * TRANSPONDER_BITS_PER_BYTE * TRANSPONDER_DATA_LENGTH)

/*
 * Implementation note:
 * Using around over 700 bytes for a transponder DMA buffer is a little excessive, likely an alternative implementation that uses a fast
 * ISR to generate the output signal dynamically based on state would be more memory efficient and would likely be more appropriate for
 * other targets.  However this approach requires very little CPU time and is just fire-and-forget.
 *
 * On an STM32F303CC 720 bytes is currently fine and that is the target for which this code was designed for.
 */
#if defined(STM32F3)
uint8_t transponderIrDMABuffer[TRANSPONDER_DMA_BUFFER_SIZE];
#elif defined(STM32F4)
uint32_t transponderIrDMABuffer[TRANSPONDER_DMA_BUFFER_SIZE];
#else
#error "Transponder not supported on this MCU."
#endif

volatile uint8_t transponderIrDataTransferInProgress = 0;


static IO_t transponderIO = IO_NONE;
static TIM_TypeDef *timer = NULL;
#if defined(STM32F3)
static DMA_Channel_TypeDef *dmaRef = NULL;
#elif defined(STM32F4)
static DMA_Stream_TypeDef *dmaRef = NULL;
#else
#error "Transponder not supported on this MCU."
#endif

static void TRANSPONDER_DMA_IRQHandler(dmaChannelDescriptor_t* descriptor)
{
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
        transponderIrDataTransferInProgress = 0;

        DMA_Cmd(descriptor->ref, DISABLE);
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

    if (timerHardware->dmaRef == NULL) {
        return;
    }

    transponderIO = IOGetByTag(ioTag);
    IOInit(transponderIO, OWNER_TRANSPONDER, 0);
    IOConfigGPIOAF(transponderIO, IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_DOWN), timerHardware->alternateFunction);

    dmaInit(timerHardware->dmaIrqHandler, OWNER_TRANSPONDER, 0);
    dmaSetHandler(timerHardware->dmaIrqHandler, TRANSPONDER_DMA_IRQHandler, NVIC_PRIO_TRANSPONDER_DMA, 0);

    RCC_ClockCmd(timerRCC(timer), ENABLE);

    uint16_t prescaler = timerGetPrescalerByDesiredMhz(timer, TRANSPONDER_TIMER_MHZ);
    uint16_t period = timerGetPeriodByPrescaler(timer, prescaler, TRANSPONDER_CARRIER_HZ);

    bitToggleOne = period / 2;
    
    /* Time base configuration */
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = period;
    TIM_TimeBaseStructure.TIM_Prescaler = prescaler;
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

    /* configure DMA */
    dmaRef = timerHardware->dmaRef;
    DMA_Cmd(dmaRef, DISABLE);
    DMA_DeInit(dmaRef);

    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)timerCCR(timer, timerHardware->channel);
#if defined(STM32F3)
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)transponderIrDMABuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
#elif defined(STM32F4)
    DMA_InitStructure.DMA_Channel = timerHardware->dmaChannel;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)transponderIrDMABuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
#endif
    DMA_InitStructure.DMA_BufferSize = TRANSPONDER_DMA_BUFFER_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
#if defined(STM32F3)
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
#elif defined(STM32F4)

    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
#endif
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;

    DMA_Init(dmaRef, &DMA_InitStructure);

    TIM_DMACmd(timer, timerDmaSource(timerHardware->channel), ENABLE);

    DMA_ITConfig(dmaRef, DMA_IT_TC, ENABLE);
}

bool transponderIrInit(void)
{
    memset(&transponderIrDMABuffer, 0, TRANSPONDER_DMA_BUFFER_SIZE);

    ioTag_t ioTag = IO_TAG_NONE;
    for (int i = 0; i < USABLE_TIMER_CHANNEL_COUNT; i++) {
        if (timerHardware[i].usageFlags & TIM_USE_TRANSPONDER) {
            ioTag = timerHardware[i].tag;
            break;
        }
    }

    if (!ioTag) {
        return false;
    }


    transponderIrHardwareInit(ioTag);

    return true;
}

bool isTransponderIrReady(void)
{
    return !transponderIrDataTransferInProgress;
}

static uint16_t dmaBufferOffset;

void updateTransponderDMABuffer(const uint8_t* transponderData)
{
    uint8_t byteIndex;
    uint8_t bitIndex;
    uint8_t toggleIndex;

    for (byteIndex = 0; byteIndex < TRANSPONDER_DATA_LENGTH; byteIndex++) {

        uint8_t byteToSend = *transponderData;
        transponderData++;
        for (bitIndex = 0; bitIndex < TRANSPONDER_BITS_PER_BYTE; bitIndex++)
        {
            bool doToggles = false;
            if (bitIndex == 0) {
                doToggles = true;
            } else if (bitIndex == TRANSPONDER_BITS_PER_BYTE - 1) {
                doToggles = false;
            } else {
                doToggles = byteToSend & (1 << (bitIndex - 1));
            }

            for (toggleIndex = 0; toggleIndex < TRANSPONDER_TOGGLES_PER_BIT; toggleIndex++)
            {
                if (doToggles) {
                    transponderIrDMABuffer[dmaBufferOffset] = bitToggleOne;
                } else {
                    transponderIrDMABuffer[dmaBufferOffset] = BIT_TOGGLE_0;
                }
                dmaBufferOffset++;
            }
            transponderIrDMABuffer[dmaBufferOffset] = BIT_TOGGLE_0;
            dmaBufferOffset++;
        }
    }
}

void transponderIrWaitForTransmitComplete(void)
{
    static uint32_t waitCounter = 0;

    while(transponderIrDataTransferInProgress) {
        waitCounter++;
    }
}

void transponderIrUpdateData(const uint8_t* transponderData)
{
    transponderIrWaitForTransmitComplete();

    updateTransponderDMABuffer(transponderData);
}

void transponderIrDMAEnable(void)
{
    DMA_SetCurrDataCounter(dmaRef, TRANSPONDER_DMA_BUFFER_SIZE);  // load number of bytes to be transferred
    TIM_SetCounter(timer, 0);
    TIM_Cmd(timer, ENABLE);
    DMA_Cmd(dmaRef, ENABLE);
}

void transponderIrDisable(void)
{
    DMA_Cmd(dmaRef, DISABLE);
    TIM_Cmd(timer, DISABLE);

    IOInit(transponderIO, OWNER_TRANSPONDER, 0);
    IOConfigGPIOAF(transponderIO, IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_DOWN), timerHardware->alternateFunction);

#ifdef TRANSPONDER_INVERTED
    IOHi(transponderIO);
#else
    IOLo(transponderIO);
#endif
}

void transponderIrTransmit(void)
{
    transponderIrWaitForTransmitComplete();

    dmaBufferOffset = 0;

    transponderIrDataTransferInProgress = 1;
    transponderIrDMAEnable();
}
#endif
