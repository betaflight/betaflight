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
#include <string.h>
#include <stdint.h>

#include <platform.h>

#include "build/build_config.h"

#include "drivers/dma.h"
#include "drivers/nvic.h"

#define DEFINE_DMA_CHANNEL(d, c, f, i, r) \
    {.dma = d, .channel = c, .handler = NULL, .flagsShift = f, .irqn = i, .rcc = r}

#define DEFINE_DMA_IRQ_HANDLER(d, c, h) \
    void DMA ## d ## _Channel ## c ## _IRQHandler(void) {\
        DMA_IRQHandler(h);\
    } \
    struct dummy


/*
 * DMA descriptors.
 */
dmaChannel_t dmaChannels[] = {
    DEFINE_DMA_CHANNEL(DMA1, DMA1_Channel1,  0, DMA1_Channel1_IRQn, RCC_AHBPeriph_DMA1),
    DEFINE_DMA_CHANNEL(DMA1, DMA1_Channel2,  4, DMA1_Channel2_IRQn, RCC_AHBPeriph_DMA1),
    DEFINE_DMA_CHANNEL(DMA1, DMA1_Channel3,  8, DMA1_Channel3_IRQn, RCC_AHBPeriph_DMA1),
    DEFINE_DMA_CHANNEL(DMA1, DMA1_Channel4, 12, DMA1_Channel4_IRQn, RCC_AHBPeriph_DMA1),
    DEFINE_DMA_CHANNEL(DMA1, DMA1_Channel5, 16, DMA1_Channel5_IRQn, RCC_AHBPeriph_DMA1),
    DEFINE_DMA_CHANNEL(DMA1, DMA1_Channel6, 20, DMA1_Channel6_IRQn, RCC_AHBPeriph_DMA1),
    DEFINE_DMA_CHANNEL(DMA1, DMA1_Channel7, 24, DMA1_Channel7_IRQn, RCC_AHBPeriph_DMA1),
#if defined(STM32F3) || defined(STM32F10X_CL)
    DEFINE_DMA_CHANNEL(DMA2, DMA2_Channel1,  0, DMA2_Channel1_IRQn, RCC_AHBPeriph_DMA2),
    DEFINE_DMA_CHANNEL(DMA2, DMA2_Channel2,  4, DMA2_Channel2_IRQn, RCC_AHBPeriph_DMA2),
    DEFINE_DMA_CHANNEL(DMA2, DMA2_Channel3,  8, DMA2_Channel3_IRQn, RCC_AHBPeriph_DMA2),
    DEFINE_DMA_CHANNEL(DMA2, DMA2_Channel4, 12, DMA2_Channel4_IRQn, RCC_AHBPeriph_DMA2),
    DEFINE_DMA_CHANNEL(DMA2, DMA2_Channel5, 16, DMA2_Channel5_IRQn, RCC_AHBPeriph_DMA2),
#endif
};

void DMA_IRQHandler(dmaChannel_t* channel)
{
    dmaCallbackHandler_t* handler = channel->handler;
    while (handler) {
        handler->fn(channel, handler);
        handler = handler->next;
    }
}


/*
 * DMA IRQ Handlers
 */
DEFINE_DMA_IRQ_HANDLER(1, 1, DMA1Channel1Descriptor);
DEFINE_DMA_IRQ_HANDLER(1, 2, DMA1Channel2Descriptor);
DEFINE_DMA_IRQ_HANDLER(1, 3, DMA1Channel3Descriptor);
DEFINE_DMA_IRQ_HANDLER(1, 4, DMA1Channel4Descriptor);
DEFINE_DMA_IRQ_HANDLER(1, 5, DMA1Channel5Descriptor);
DEFINE_DMA_IRQ_HANDLER(1, 6, DMA1Channel6Descriptor);
DEFINE_DMA_IRQ_HANDLER(1, 7, DMA1Channel7Descriptor);

#if defined(STM32F3) || defined(STM32F10X_CL)
DEFINE_DMA_IRQ_HANDLER(2, 1, DMA2Channel1Descriptor);
DEFINE_DMA_IRQ_HANDLER(2, 2, DMA2Channel2Descriptor);
DEFINE_DMA_IRQ_HANDLER(2, 3, DMA2Channel3Descriptor);
DEFINE_DMA_IRQ_HANDLER(2, 4, DMA2Channel4Descriptor);
DEFINE_DMA_IRQ_HANDLER(2, 5, DMA2Channel5Descriptor);
#endif


void dmaInit(void)
{
    // TODO: Do we need this?
}

void dmaHandlerInit(dmaCallbackHandler_t* handlerRec, dmaCallbackHandlerFunc* handler)
{
    handlerRec->fn = handler;
    handlerRec->next = NULL;
}

// This function initialize DMA interrupt and adds user defined handler to this interrupt
// Note: Interrupt priority will be set only once. Call this function for highest priority handler first
void dmaSetHandler(dmaChannel_t* channel, dmaCallbackHandler_t* handler, uint8_t priority)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    if (!channel->handler) {
        channel->handler = handler;

        RCC_AHBPeriphClockCmd(channel->rcc, ENABLE);

        NVIC_InitStructure.NVIC_IRQChannel = channel->irqn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(priority);
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(priority);
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    } else {
        handler->next = channel->handler;
        channel->handler = handler;
    }
}

