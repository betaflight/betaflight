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


void dmaInit(dmaHandlerIdentifier_e identifier, resourceOwner_t owner, uint8_t resourceIndex)
{
    RCC_AHBPeriphClockCmd(dmaDescriptors[identifier].rcc, ENABLE);
    dmaDescriptors[identifier].owner = owner;
    dmaDescriptors[identifier].resourceIndex = resourceIndex;
}

dmaHandlerIdentifier_e dmaFindHandlerIdentifier(DMA_Channel_TypeDef* channel)
{
    dmaHandlerIdentifier_e i;

    for (i = 0; i < (sizeof(dmaDescriptors) / sizeof(dmaDescriptors[0])); i++) {
        if (channel == dmaDescriptors[i].ref) {
            return i;
        }
    }

    // Shouldn't get here
    return 0;
}

void dmaEnableClock(dmaHandlerIdentifier_e identifier)
{
    RCC_AHBPeriphClockCmd(dmaDescriptors[identifier].rcc, ENABLE);
}

void dmaSetHandler(dmaHandlerIdentifier_e identifier, dmaCallbackHandlerFuncPtr callback, uint32_t priority, uint32_t userParam)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    dmaEnableClock(identifier);

    dmaDescriptors[identifier].irqHandlerCallback = callback;
    dmaDescriptors[identifier].userParam = userParam;

    NVIC_InitStructure.NVIC_IRQChannel = dmaDescriptors[identifier].irqN;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(priority);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(priority);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

