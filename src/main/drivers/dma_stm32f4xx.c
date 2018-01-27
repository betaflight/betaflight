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

#include "drivers/nvic.h"
#include "dma.h"
#include "resource.h"

/*
 * DMA descriptors.
 */
static dmaChannelDescriptor_t dmaDescriptors[DMA_MAX_DESCRIPTORS] = {
    DEFINE_DMA_CHANNEL(DMA1, 0,  0, RCC_AHB1Periph_DMA1),
    DEFINE_DMA_CHANNEL(DMA1, 1,  6, RCC_AHB1Periph_DMA1),
    DEFINE_DMA_CHANNEL(DMA1, 2, 16, RCC_AHB1Periph_DMA1),
    DEFINE_DMA_CHANNEL(DMA1, 3, 22, RCC_AHB1Periph_DMA1),
    DEFINE_DMA_CHANNEL(DMA1, 4, 32, RCC_AHB1Periph_DMA1),
    DEFINE_DMA_CHANNEL(DMA1, 5, 38, RCC_AHB1Periph_DMA1),
    DEFINE_DMA_CHANNEL(DMA1, 6, 48, RCC_AHB1Periph_DMA1),
    DEFINE_DMA_CHANNEL(DMA1, 7, 54, RCC_AHB1Periph_DMA1),

    DEFINE_DMA_CHANNEL(DMA2, 0,  0, RCC_AHB1Periph_DMA2),
    DEFINE_DMA_CHANNEL(DMA2, 1,  6, RCC_AHB1Periph_DMA2),
    DEFINE_DMA_CHANNEL(DMA2, 2, 16, RCC_AHB1Periph_DMA2),
    DEFINE_DMA_CHANNEL(DMA2, 3, 22, RCC_AHB1Periph_DMA2),
    DEFINE_DMA_CHANNEL(DMA2, 4, 32, RCC_AHB1Periph_DMA2),
    DEFINE_DMA_CHANNEL(DMA2, 5, 38, RCC_AHB1Periph_DMA2),
    DEFINE_DMA_CHANNEL(DMA2, 6, 48, RCC_AHB1Periph_DMA2),
    DEFINE_DMA_CHANNEL(DMA2, 7, 54, RCC_AHB1Periph_DMA2),
};

/*
 * DMA IRQ Handlers
 */
DEFINE_DMA_IRQ_HANDLER(1, 0, DMA1_ST0_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 1, DMA1_ST1_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 2, DMA1_ST2_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 3, DMA1_ST3_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 4, DMA1_ST4_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 5, DMA1_ST5_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 6, DMA1_ST6_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 7, DMA1_ST7_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 0, DMA2_ST0_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 1, DMA2_ST1_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 2, DMA2_ST2_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 3, DMA2_ST3_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 4, DMA2_ST4_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 5, DMA2_ST5_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 6, DMA2_ST6_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 7, DMA2_ST7_HANDLER)

void dmaInit(dmaIdentifier_e identifier, resourceOwner_e owner, uint8_t resourceIndex)
{
    const int index = identifier-1;
    RCC_AHB1PeriphClockCmd(dmaDescriptors[index].rcc, ENABLE);
    dmaDescriptors[index].owner = owner;
    dmaDescriptors[index].resourceIndex = resourceIndex;
}

void dmaSetHandler(dmaIdentifier_e identifier, dmaCallbackHandlerFuncPtr callback, uint32_t priority, uint32_t userParam)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    const int index = identifier-1;

    RCC_AHB1PeriphClockCmd(dmaDescriptors[index].rcc, ENABLE);
    dmaDescriptors[index].irqHandlerCallback = callback;
    dmaDescriptors[index].userParam = userParam;

    NVIC_InitStructure.NVIC_IRQChannel = dmaDescriptors[index].irqN;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(priority);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(priority);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

#define RETURN_TCIF_FLAG(s, n) if (s == DMA1_Stream ## n || s == DMA2_Stream ## n) return DMA_IT_TCIF ## n

uint32_t dmaFlag_IT_TCIF(const DMA_Stream_TypeDef *stream)
{
    RETURN_TCIF_FLAG(stream, 0);
    RETURN_TCIF_FLAG(stream, 1);
    RETURN_TCIF_FLAG(stream, 2);
    RETURN_TCIF_FLAG(stream, 3);
    RETURN_TCIF_FLAG(stream, 4);
    RETURN_TCIF_FLAG(stream, 5);
    RETURN_TCIF_FLAG(stream, 6);
    RETURN_TCIF_FLAG(stream, 7);
    return 0;
}

resourceOwner_e dmaGetOwner(dmaIdentifier_e identifier)
{
    return dmaDescriptors[identifier-1].owner;
}

uint8_t dmaGetResourceIndex(dmaIdentifier_e identifier)
{
    return dmaDescriptors[identifier-1].resourceIndex;
}

dmaIdentifier_e dmaGetIdentifier(const DMA_Stream_TypeDef* stream)
{
    for (int i = 1; i < DMA_MAX_DESCRIPTORS; i++) {
        if (dmaDescriptors[i-1].ref == stream) {
            return i;
        }
    }
    return 0;
}

dmaChannelDescriptor_t* dmaGetDescriptor(const DMA_Stream_TypeDef* stream)
{
    for (int i = 1; i < DMA_MAX_DESCRIPTORS; i++) {
        if (dmaDescriptors[i-1].ref == stream) {
            return &dmaDescriptors[i-1];
        }
    }
    return NULL;
}

DMA_Stream_TypeDef* dmaGetRefByIdentifier(const dmaIdentifier_e identifier)
{
    return dmaDescriptors[identifier-1].ref;
}

dmaChannelDescriptor_t* dmaGetDescriptorByIdentifier(const dmaIdentifier_e identifier)
{
    return &dmaDescriptors[identifier-1];
}

uint32_t dmaGetChannel(const uint8_t channel)
{
    return ((uint32_t)channel*2)<<24;
}