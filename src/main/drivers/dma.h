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

#include "resource.h"

struct dmaChannelDescriptor_s;
typedef void (*dmaCallbackHandlerFuncPtr)(struct dmaChannelDescriptor_s *channelDescriptor);

typedef struct dmaChannelDescriptor_s {
    DMA_TypeDef*                dma;
#if defined(STM32F4) || defined(STM32F7)
    DMA_Stream_TypeDef*         ref;
#else
    DMA_Channel_TypeDef*        ref;
#endif
    dmaCallbackHandlerFuncPtr   irqHandlerCallback;
    uint8_t                     flagsShift;
    IRQn_Type                   irqN;
    uint32_t                    rcc;
    uint32_t                    userParam;
    resourceOwner_t             owner;
    uint8_t                     resourceIndex;
} dmaChannelDescriptor_t;

#if defined(STM32F4) || defined(STM32F7)

typedef enum {
    DMA1_ST0_HANDLER = 0,
    DMA1_ST1_HANDLER,
    DMA1_ST2_HANDLER,
    DMA1_ST3_HANDLER,
    DMA1_ST4_HANDLER,
    DMA1_ST5_HANDLER,
    DMA1_ST6_HANDLER,
    DMA1_ST7_HANDLER,
    DMA2_ST0_HANDLER,
    DMA2_ST1_HANDLER,
    DMA2_ST2_HANDLER,
    DMA2_ST3_HANDLER,
    DMA2_ST4_HANDLER,
    DMA2_ST5_HANDLER,
    DMA2_ST6_HANDLER,
    DMA2_ST7_HANDLER,
} dmaHandlerIdentifier_e;

#define DEFINE_DMA_CHANNEL(d, s, f, i, r) {.dma = d, .ref = s, .irqHandlerCallback = NULL, .flagsShift = f, .irqN = i, .rcc = r, .userParam = 0}
#define DEFINE_DMA_IRQ_HANDLER(d, s, i) void DMA ## d ## _Stream ## s ## _IRQHandler(void) {\
                                                                if (dmaDescriptors[i].irqHandlerCallback)\
                                                                    dmaDescriptors[i].irqHandlerCallback(&dmaDescriptors[i]);\
                                                            }

#define DMA_CLEAR_FLAG(d, flag) if (d->flagsShift > 31) d->dma->HIFCR = (flag << (d->flagsShift - 32)); else d->dma->LIFCR = (flag << d->flagsShift)
#define DMA_GET_FLAG_STATUS(d, flag) (d->flagsShift > 31 ? d->dma->HISR & (flag << (d->flagsShift - 32)): d->dma->LISR & (flag << d->flagsShift))


#define DMA_IT_TCIF                         ((uint32_t)0x00000020)
#define DMA_IT_HTIF                         ((uint32_t)0x00000010)
#define DMA_IT_TEIF                         ((uint32_t)0x00000008)
#define DMA_IT_DMEIF                        ((uint32_t)0x00000004)
#define DMA_IT_FEIF                         ((uint32_t)0x00000001)

#else

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

#define DEFINE_DMA_CHANNEL(d, c, f, i, r) {.dma = d, .ref = c, .irqHandlerCallback = NULL, .flagsShift = f, .irqN = i, .rcc = r, .userParam = 0}
#define DEFINE_DMA_IRQ_HANDLER(d, c, i) void DMA ## d ## _Channel ## c ## _IRQHandler(void) {\
                                                                        if (dmaDescriptors[i].irqHandlerCallback)\
                                                                            dmaDescriptors[i].irqHandlerCallback(&dmaDescriptors[i]);\
                                                                    }

#define DMA_CLEAR_FLAG(d, flag) d->dma->IFCR = (flag << d->flagsShift)
#define DMA_GET_FLAG_STATUS(d, flag) (d->dma->ISR & (flag << d->flagsShift))

#define DMA_IT_TCIF                          ((uint32_t)0x00000002)
#define DMA_IT_HTIF                          ((uint32_t)0x00000004)
#define DMA_IT_TEIF                          ((uint32_t)0x00000008)

#endif

#if defined(STM32F7)
//#define HAL_CLEANINVALIDATECACHE(addr, size) (SCB_CleanInvalidateDCache_by_Addr((uint32_t*)((uint32_t)addr & ~0x1f), ((uint32_t)(addr + size + 0x1f) & ~0x1f) - ((uint32_t)addr & ~0x1f)))
//#define HAL_CLEANCACHE(addr, size) (SCB_CleanDCache_by_Addr((uint32_t*)((uint32_t)addr & ~0x1f), ((uint32_t)(addr + size + 0x1f) & ~0x1f) - ((uint32_t)addr & ~0x1f)))
#endif

#if defined(STM32F4) || defined(STM32F7)
dmaHandlerIdentifier_e dmaFindHandlerIdentifier(DMA_Stream_TypeDef* stream);
#else
dmaHandlerIdentifier_e dmaFindHandlerIdentifier(DMA_Channel_TypeDef* channel);
#endif

void dmaInit(dmaHandlerIdentifier_e identifier, resourceOwner_t owner, uint8_t resourceIndex);
void dmaEnableClock(dmaHandlerIdentifier_e identifier);
void dmaSetHandler(dmaHandlerIdentifier_e identifier, dmaCallbackHandlerFuncPtr callback, uint32_t priority, uint32_t userParam);

