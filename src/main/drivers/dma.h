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
typedef struct dmaChannelDescriptor_s dmaChannelDescriptor_t;

typedef void dmaCallbackHandlerFuncPtr(dmaChannelDescriptor_t* descriptor, dmaCallbackHandler_t* callbackHandler);

struct dmaCallbackHandler_s {
    dmaCallbackHandlerFuncPtr*  fn;
    dmaCallbackHandler_t*       next;
};


typedef enum {
    DMA1_CH1_HANDLER = 0,
    DMA1_CH2_HANDLER,
    DMA1_CH3_HANDLER,
    DMA1_CH4_HANDLER,
    DMA1_CH5_HANDLER,
    DMA1_CH6_HANDLER,
    DMA1_CH7_HANDLER,
    DMA2_CH1_HANDLER,
    DMA2_CH2_HANDLER,
    DMA2_CH3_HANDLER,
    DMA2_CH4_HANDLER,
    DMA2_CH5_HANDLER,
} dmaHandlerIdentifier_e;

struct dmaChannelDescriptor_s {
    DMA_TypeDef*                dma;
    DMA_Channel_TypeDef*        channel;
    dmaCallbackHandler_t*       handler;
    uint8_t                     flagsShift;
    IRQn_Type                   irqn;
    uint32_t                    rcc;
};

#define DMA_CLEAR_FLAG(d, flag) d->dma->IFCR |= (flag << d->flagsShift)
#define DMA_GET_FLAG_STATUS(d, flag) (d->dma->ISR & (flag << d->flagsShift))

#define DMA_IT_TCIF                          ((uint32_t)0x00000002)
#define DMA_IT_HTIF                          ((uint32_t)0x00000004)
#define DMA_IT_TEIF                          ((uint32_t)0x00000008)

void dmaInit(void);
void dmaHandlerInit(dmaCallbackHandler_t* handlerRec, dmaCallbackHandlerFuncPtr* handler);
void dmaSetHandler(dmaHandlerIdentifier_e identifier, dmaCallbackHandler_t* handler, uint32_t priority);

