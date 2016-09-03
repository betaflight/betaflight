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

#pragma once

typedef struct dmaCallbackHandler_s dmaCallbackHandler_t;
typedef struct dmaChannel_s dmaChannel_t;

typedef void dmaCallbackHandlerFunc(dmaChannel_t* descriptor, dmaCallbackHandler_t* callbackHandler);

struct dmaCallbackHandler_s {
    dmaCallbackHandlerFunc*  fn;
    dmaCallbackHandler_t*       next;
};

#define DMA1Channel1Descriptor  (&dmaChannels[0])
#define DMA1Channel2Descriptor  (&dmaChannels[1])
#define DMA1Channel3Descriptor  (&dmaChannels[2])
#define DMA1Channel4Descriptor  (&dmaChannels[3])
#define DMA1Channel5Descriptor  (&dmaChannels[4])
#define DMA1Channel6Descriptor  (&dmaChannels[5])
#define DMA1Channel7Descriptor  (&dmaChannels[6])
#if defined(STM32F303xC) || defined(STM32F10X_CL)
#define DMA2Channel1Descriptor  (&dmaChannels[7])
#define DMA2Channel2Descriptor  (&dmaChannels[8])
#define DMA2Channel3Descriptor  (&dmaChannels[9])
#define DMA2Channel4Descriptor  (&dmaChannels[10])
#define DMA2Channel5Descriptor  (&dmaChannels[11])
#endif

struct dmaChannel_s {
    DMA_TypeDef*                dma;
    DMA_Channel_TypeDef*        channel;
    dmaCallbackHandler_t*       handler;
    uint8_t                     flagsShift;
    IRQn_Type                   irqn;
    uint32_t                    rcc;
};

extern dmaChannel_t dmaChannels[];

#define DMA_CLEAR_FLAG(d, flag) d->dma->IFCR |= (flag << d->flagsShift)
#define DMA_GET_FLAG_STATUS(d, flag) (d->dma->ISR & (flag << d->flagsShift))

#define DMA_IT_TCIF                          ((uint32_t)0x00000002)
#define DMA_IT_HTIF                          ((uint32_t)0x00000004)
#define DMA_IT_TEIF                          ((uint32_t)0x00000008)

void dmaInit(void);
void dmaHandlerInit(dmaCallbackHandler_t* handlerRec, dmaCallbackHandlerFunc* handler);
void dmaSetHandler(dmaChannel_t* dmaChannel, dmaCallbackHandler_t* handler, uint8_t priority);

