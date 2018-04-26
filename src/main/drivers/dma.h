/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "resource.h"

struct dmaChannelDescriptor_s;
typedef void (*dmaCallbackHandlerFuncPtr)(struct dmaChannelDescriptor_s *channelDescriptor);

typedef struct dmaChannelDescriptor_s {
    DMA_TypeDef*                dma;
#if defined(STM32F4) || defined(STM32F7)
    DMA_Stream_TypeDef*         ref;
    uint8_t                     stream;
#else
    DMA_Channel_TypeDef*        ref;
#endif
    dmaCallbackHandlerFuncPtr   irqHandlerCallback;
    uint8_t                     flagsShift;
    IRQn_Type                   irqN;
    uint32_t                    userParam;
    resourceOwner_e             owner;
    uint8_t                     resourceIndex;
    uint32_t                    completeFlag;
} dmaChannelDescriptor_t;

#if defined(STM32F7)
//#define HAL_CLEANINVALIDATECACHE(addr, size) (SCB_CleanInvalidateDCache_by_Addr((uint32_t*)((uint32_t)addr & ~0x1f), ((uint32_t)(addr + size + 0x1f) & ~0x1f) - ((uint32_t)addr & ~0x1f)))
//#define HAL_CLEANCACHE(addr, size) (SCB_CleanDCache_by_Addr((uint32_t*)((uint32_t)addr & ~0x1f), ((uint32_t)(addr + size + 0x1f) & ~0x1f) - ((uint32_t)addr & ~0x1f)))

#endif

#define DMA_IDENTIFIER_TO_INDEX(x) ((x) - 1)

#if defined(STM32F4) || defined(STM32F7)

typedef enum {
    DMA_NONE = 0,
    DMA1_ST0_HANDLER = 1,
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
    DMA_LAST_HANDLER = DMA2_ST7_HANDLER
} dmaIdentifier_e;

#define DMA_DEVICE_NO(x)    ((((x)-1) / 8) + 1)
#define DMA_DEVICE_INDEX(x) ((((x)-1) % 8))
#define DMA_OUTPUT_INDEX    0
#define DMA_OUTPUT_STRING   "DMA%d Stream %d:"
#define DMA_INPUT_STRING    "DMA%d_ST%d"

#define DEFINE_DMA_CHANNEL(d, s, f) { \
    .dma = d, \
    .ref = d ## _Stream ## s, \
    .stream = s, \
    .irqHandlerCallback = NULL, \
    .flagsShift = f, \
    .irqN = d ## _Stream ## s ## _IRQn, \
    .userParam = 0, \
    .owner = 0, \
    .resourceIndex = 0 \
    } 

#define DEFINE_DMA_IRQ_HANDLER(d, s, i) void DMA ## d ## _Stream ## s ## _IRQHandler(void) {\
                                                                const uint8_t index = DMA_IDENTIFIER_TO_INDEX(i); \
                                                                if (dmaDescriptors[index].irqHandlerCallback)\
                                                                    dmaDescriptors[index].irqHandlerCallback(&dmaDescriptors[index]);\
                                                            }

#define DMA_CLEAR_FLAG(d, flag) if (d->flagsShift > 31) d->dma->HIFCR = (flag << (d->flagsShift - 32)); else d->dma->LIFCR = (flag << d->flagsShift)
#define DMA_GET_FLAG_STATUS(d, flag) (d->flagsShift > 31 ? d->dma->HISR & (flag << (d->flagsShift - 32)): d->dma->LISR & (flag << d->flagsShift))


#define DMA_IT_TCIF         ((uint32_t)0x00000020)
#define DMA_IT_HTIF         ((uint32_t)0x00000010)
#define DMA_IT_TEIF         ((uint32_t)0x00000008)
#define DMA_IT_DMEIF        ((uint32_t)0x00000004)
#define DMA_IT_FEIF         ((uint32_t)0x00000001)

dmaIdentifier_e dmaGetIdentifier(const DMA_Stream_TypeDef* stream);
dmaChannelDescriptor_t* dmaGetDmaDescriptor(const DMA_Stream_TypeDef* stream);
DMA_Stream_TypeDef* dmaGetRefByIdentifier(const dmaIdentifier_e identifier);
uint32_t dmaGetChannel(const uint8_t channel);

#else

typedef enum {
    DMA_NONE = 0,
    DMA1_CH1_HANDLER = 1,
    DMA1_CH2_HANDLER,
    DMA1_CH3_HANDLER,
    DMA1_CH4_HANDLER,
    DMA1_CH5_HANDLER,
    DMA1_CH6_HANDLER,
    DMA1_CH7_HANDLER,
#if defined(STM32F3) || defined(STM32F10X_CL)
    DMA2_CH1_HANDLER,
    DMA2_CH2_HANDLER,
    DMA2_CH3_HANDLER,
    DMA2_CH4_HANDLER,
    DMA2_CH5_HANDLER,
    DMA_LAST_HANDLER = DMA2_CH5_HANDLER 
#else 
    DMA_LAST_HANDLER = DMA1_CH7_HANDLER
#endif
} dmaIdentifier_e;

#define DMA_DEVICE_NO(x)    ((((x)-1) / 7) + 1)
#define DMA_DEVICE_INDEX(x) ((((x)-1) % 7) + 1)
#define DMA_OUTPUT_INDEX    0
#define DMA_OUTPUT_STRING   "DMA%d Channel %d:"
#define DMA_INPUT_STRING    "DMA%d_CH%d"

#define DEFINE_DMA_CHANNEL(d, c, f) { \
    .dma = d, \
    .ref = d ## _Channel ## c, \
    .irqHandlerCallback = NULL, \
    .flagsShift = f, \
    .irqN = d ## _Channel ## c ## _IRQn, \
    .userParam = 0, \
    .owner = 0, \
    .resourceIndex = 0 \
    }

#define DEFINE_DMA_IRQ_HANDLER(d, c, i) void DMA ## d ## _Channel ## c ## _IRQHandler(void) {\
                                                                        const uint8_t index = DMA_IDENTIFIER_TO_INDEX(i); \
                                                                        if (dmaDescriptors[index].irqHandlerCallback)\
                                                                            dmaDescriptors[index].irqHandlerCallback(&dmaDescriptors[index]);\
                                                                    }

#define DMA_CLEAR_FLAG(d, flag) d->dma->IFCR = (flag << d->flagsShift)
#define DMA_GET_FLAG_STATUS(d, flag) (d->dma->ISR & (flag << d->flagsShift))

#define DMA_IT_TCIF         ((uint32_t)0x00000002)
#define DMA_IT_HTIF         ((uint32_t)0x00000004)
#define DMA_IT_TEIF         ((uint32_t)0x00000008)

dmaIdentifier_e dmaGetIdentifier(const DMA_Channel_TypeDef* channel);
DMA_Channel_TypeDef* dmaGetRefByIdentifier(const dmaIdentifier_e identifier);

#endif

void dmaInit(dmaIdentifier_e identifier, resourceOwner_e owner, uint8_t resourceIndex);
void dmaSetHandler(dmaIdentifier_e identifier, dmaCallbackHandlerFuncPtr callback, uint32_t priority, uint32_t userParam);

resourceOwner_e dmaGetOwner(dmaIdentifier_e identifier);
uint8_t dmaGetResourceIndex(dmaIdentifier_e identifier);
dmaChannelDescriptor_t* dmaGetDescriptorByIdentifier(const dmaIdentifier_e identifier);
