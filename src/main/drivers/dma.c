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

#include "drivers/nvic.h"
#include "dma.h"

/*
 * DMA descriptors.
 */
static dmaChannelDescriptor_t dmaDescriptors[DMA_LAST_HANDLER] = {
    DEFINE_DMA_CHANNEL(DMA1, 1,  0),
    DEFINE_DMA_CHANNEL(DMA1, 2,  4),
    DEFINE_DMA_CHANNEL(DMA1, 3,  8),
    DEFINE_DMA_CHANNEL(DMA1, 4, 12),
    DEFINE_DMA_CHANNEL(DMA1, 5, 16),
    DEFINE_DMA_CHANNEL(DMA1, 6, 20),
    DEFINE_DMA_CHANNEL(DMA1, 7, 24),
#if defined(STM32F3) || defined(STM32F10X_CL)
    DEFINE_DMA_CHANNEL(DMA2, 1,  0),
    DEFINE_DMA_CHANNEL(DMA2, 2,  4),
    DEFINE_DMA_CHANNEL(DMA2, 3,  8),
    DEFINE_DMA_CHANNEL(DMA2, 4, 12),
    DEFINE_DMA_CHANNEL(DMA2, 5, 16),
#endif
};

/*
 * DMA IRQ Handlers
 */
DEFINE_DMA_IRQ_HANDLER(1, 1, DMA1_CH1_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 2, DMA1_CH2_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 3, DMA1_CH3_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 4, DMA1_CH4_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 5, DMA1_CH5_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 6, DMA1_CH6_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 7, DMA1_CH7_HANDLER)

#if defined(STM32F3) || defined(STM32F10X_CL)
DEFINE_DMA_IRQ_HANDLER(2, 1, DMA2_CH1_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 2, DMA2_CH2_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 3, DMA2_CH3_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 4, DMA2_CH4_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 5, DMA2_CH5_HANDLER)
#endif

#define RETURN_TCIF_FLAG(s, d, n) if (s == DMA ## d ## _Channel ## n) return DMA ## d ## _FLAG_TC ## n

uint32_t dmaFlag_IT_TCIF(const DMA_Channel_TypeDef *channel)
{
    RETURN_TCIF_FLAG(channel, 1, 1);
    RETURN_TCIF_FLAG(channel, 1, 2);
    RETURN_TCIF_FLAG(channel, 1, 3);
    RETURN_TCIF_FLAG(channel, 1, 4);
    RETURN_TCIF_FLAG(channel, 1, 5);
    RETURN_TCIF_FLAG(channel, 1, 6);
    RETURN_TCIF_FLAG(channel, 1, 7);
    RETURN_TCIF_FLAG(channel, 2, 1);
    RETURN_TCIF_FLAG(channel, 2, 2);
    RETURN_TCIF_FLAG(channel, 2, 3);
    RETURN_TCIF_FLAG(channel, 2, 4);
    RETURN_TCIF_FLAG(channel, 2, 5);
    return 0;
}

#define DMA_RCC(x) ((x) == DMA1 ? RCC_AHBPeriph_DMA1 : RCC_AHBPeriph_DMA2)
void dmaInit(dmaIdentifier_e identifier, resourceOwner_e owner, uint8_t resourceIndex)
{
    const int index = DMA_IDENTIFIER_TO_INDEX(identifier);

    RCC_AHBPeriphClockCmd(DMA_RCC(dmaDescriptors[index].dma), ENABLE);
    dmaDescriptors[index].owner = owner;
    dmaDescriptors[index].resourceIndex = resourceIndex;
}

void dmaSetHandler(dmaIdentifier_e identifier, dmaCallbackHandlerFuncPtr callback, uint32_t priority, uint32_t userParam)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    const int index = DMA_IDENTIFIER_TO_INDEX(identifier);
    /* TODO: remove this - enforce the init */
    RCC_AHBPeriphClockCmd(DMA_RCC(dmaDescriptors[index].dma), ENABLE);
    dmaDescriptors[index].irqHandlerCallback = callback;
    dmaDescriptors[index].userParam = userParam;
    dmaDescriptors[index].completeFlag = dmaFlag_IT_TCIF(dmaDescriptors[index].ref);

    NVIC_InitStructure.NVIC_IRQChannel = dmaDescriptors[index].irqN;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(priority);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(priority);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

resourceOwner_e dmaGetOwner(dmaIdentifier_e identifier)
{
    return dmaDescriptors[DMA_IDENTIFIER_TO_INDEX(identifier)].owner;
}

uint8_t dmaGetResourceIndex(dmaIdentifier_e identifier)
{
    return dmaDescriptors[DMA_IDENTIFIER_TO_INDEX(identifier)].resourceIndex;
}

dmaIdentifier_e dmaGetIdentifier(const DMA_Channel_TypeDef* channel)
{
    for (int i = 0; i < DMA_LAST_HANDLER; i++) {
        if (dmaDescriptors[i].ref == channel) {
            return i + 1;
        }
    }
    return 0;
}

DMA_Channel_TypeDef* dmaGetRefByIdentifier(const dmaIdentifier_e identifier)
{
    return dmaDescriptors[DMA_IDENTIFIER_TO_INDEX(identifier)].ref;
}

dmaChannelDescriptor_t* dmaGetDescriptorByIdentifier(const dmaIdentifier_e identifier)
{
    return &dmaDescriptors[DMA_IDENTIFIER_TO_INDEX(identifier)];
}