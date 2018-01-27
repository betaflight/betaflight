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
static dmaChannelDescriptor_t dmaDescriptors[] = {
    DEFINE_DMA_CHANNEL(DMA1, 1,  0, RCC_AHBPeriph_DMA1),
    DEFINE_DMA_CHANNEL(DMA1, 2,  4, RCC_AHBPeriph_DMA1),
    DEFINE_DMA_CHANNEL(DMA1, 3,  8, RCC_AHBPeriph_DMA1),
    DEFINE_DMA_CHANNEL(DMA1, 4, 12, RCC_AHBPeriph_DMA1),
    DEFINE_DMA_CHANNEL(DMA1, 5, 16, RCC_AHBPeriph_DMA1),
    DEFINE_DMA_CHANNEL(DMA1, 6, 20, RCC_AHBPeriph_DMA1),
    DEFINE_DMA_CHANNEL(DMA1, 7, 24, RCC_AHBPeriph_DMA1),
#if defined(STM32F3) || defined(STM32F10X_CL)
    DEFINE_DMA_CHANNEL(DMA2, 1,  0, RCC_AHBPeriph_DMA2),
    DEFINE_DMA_CHANNEL(DMA2, 2,  4, RCC_AHBPeriph_DMA2),
    DEFINE_DMA_CHANNEL(DMA2, 3,  8, RCC_AHBPeriph_DMA2),
    DEFINE_DMA_CHANNEL(DMA2, 4, 12, RCC_AHBPeriph_DMA2),
    DEFINE_DMA_CHANNEL(DMA2, 5, 16, RCC_AHBPeriph_DMA2),
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

void dmaInit(dmaIdentifier_e identifier, resourceOwner_e owner, uint8_t resourceIndex)
{
    const int index = identifier-1;

    RCC_AHBPeriphClockCmd(dmaDescriptors[index].rcc, ENABLE);
    dmaDescriptors[index].owner = owner;
    dmaDescriptors[index].resourceIndex = resourceIndex;
}

void dmaSetHandler(dmaIdentifier_e identifier, dmaCallbackHandlerFuncPtr callback, uint32_t priority, uint32_t userParam)
{
    NVIC_InitTypeDef NVIC_InitStructure;


    const int index = identifier-1;
    /* TODO: remove this - enforce the init */
    RCC_AHBPeriphClockCmd(dmaDescriptors[index].rcc, ENABLE);
    dmaDescriptors[index].irqHandlerCallback = callback;
    dmaDescriptors[index].userParam = userParam;

    NVIC_InitStructure.NVIC_IRQChannel = dmaDescriptors[index].irqN;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(priority);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(priority);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

resourceOwner_e dmaGetOwner(dmaIdentifier_e identifier)
{
    return dmaDescriptors[identifier-1].owner;
}

uint8_t dmaGetResourceIndex(dmaIdentifier_e identifier)
{
    return dmaDescriptors[identifier-1].resourceIndex;
}

dmaIdentifier_e dmaGetIdentifier(const DMA_Channel_TypeDef* channel)
{
    for (int i = 1; i < DMA_MAX_DESCRIPTORS; i++) {
        if (dmaDescriptors[i-1].ref == channel) {
            return i;
        }
    }
    return 0;
}

DMA_Channel_TypeDef* dmaGetRefByIdentifier(const dmaIdentifier_e identifier)
{
    return dmaDescriptors[identifier-1].ref;
}

dmaChannelDescriptor_t* dmaGetDescriptorByIdentifier(const dmaIdentifier_e identifier)
{
    return &dmaDescriptors[identifier-1];
}