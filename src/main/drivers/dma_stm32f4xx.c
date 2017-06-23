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

/*
 * DMA descriptors.
 */
static dmaChannelDescriptor_t dmaDescriptors[] = {
    DEFINE_DMA_CHANNEL(DMA1, DMA1_Stream0,  0, DMA1_Stream0_IRQn, RCC_AHB1Periph_DMA1),
    DEFINE_DMA_CHANNEL(DMA1, DMA1_Stream1,  6, DMA1_Stream1_IRQn, RCC_AHB1Periph_DMA1),
    DEFINE_DMA_CHANNEL(DMA1, DMA1_Stream2, 16, DMA1_Stream2_IRQn, RCC_AHB1Periph_DMA1),
    DEFINE_DMA_CHANNEL(DMA1, DMA1_Stream3, 22, DMA1_Stream3_IRQn, RCC_AHB1Periph_DMA1),
    DEFINE_DMA_CHANNEL(DMA1, DMA1_Stream4, 32, DMA1_Stream4_IRQn, RCC_AHB1Periph_DMA1),
    DEFINE_DMA_CHANNEL(DMA1, DMA1_Stream5, 38, DMA1_Stream5_IRQn, RCC_AHB1Periph_DMA1),
    DEFINE_DMA_CHANNEL(DMA1, DMA1_Stream6, 48, DMA1_Stream6_IRQn, RCC_AHB1Periph_DMA1),
    DEFINE_DMA_CHANNEL(DMA1, DMA1_Stream7, 54, DMA1_Stream7_IRQn, RCC_AHB1Periph_DMA1),

    DEFINE_DMA_CHANNEL(DMA2, DMA2_Stream0,  0, DMA2_Stream0_IRQn, RCC_AHB1Periph_DMA2),
    DEFINE_DMA_CHANNEL(DMA2, DMA2_Stream1,  6, DMA2_Stream1_IRQn, RCC_AHB1Periph_DMA2),
    DEFINE_DMA_CHANNEL(DMA2, DMA2_Stream2, 16, DMA2_Stream2_IRQn, RCC_AHB1Periph_DMA2),
    DEFINE_DMA_CHANNEL(DMA2, DMA2_Stream3, 22, DMA2_Stream3_IRQn, RCC_AHB1Periph_DMA2),
    DEFINE_DMA_CHANNEL(DMA2, DMA2_Stream4, 32, DMA2_Stream4_IRQn, RCC_AHB1Periph_DMA2),
    DEFINE_DMA_CHANNEL(DMA2, DMA2_Stream5, 38, DMA2_Stream5_IRQn, RCC_AHB1Periph_DMA2),
    DEFINE_DMA_CHANNEL(DMA2, DMA2_Stream6, 48, DMA2_Stream6_IRQn, RCC_AHB1Periph_DMA2),
    DEFINE_DMA_CHANNEL(DMA2, DMA2_Stream7, 54, DMA2_Stream7_IRQn, RCC_AHB1Periph_DMA2),
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

void dmaInit(dmaHandlerIdentifier_e identifier, resourceOwner_t owner, uint8_t resourceIndex)
{
    RCC_AHB1PeriphClockCmd(dmaDescriptors[identifier].rcc, ENABLE);
    dmaDescriptors[identifier].owner = owner;
    dmaDescriptors[identifier].resourceIndex = resourceIndex;
}

dmaHandlerIdentifier_e dmaFindHandlerIdentifier(DMA_Stream_TypeDef* stream)
{
    dmaHandlerIdentifier_e i;

    for (i = 0; i < (sizeof(dmaDescriptors) / sizeof(dmaDescriptors[0])); i++) {
        if (stream == dmaDescriptors[i].ref) {
            return i;
        }
    }

    // Shouldn't get here
    return 0;
}

void dmaEnableClock(dmaHandlerIdentifier_e identifier)
{
    RCC_AHB1PeriphClockCmd(dmaDescriptors[identifier].rcc, ENABLE);
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

