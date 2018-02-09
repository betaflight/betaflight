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
#include "drivers/dma.h"
#include "resource.h"

/*
 * DMA descriptors.
 */
static dmaChannelDescriptor_t dmaDescriptors[DMA_LAST_HANDLER] = {
    DEFINE_DMA_CHANNEL(DMA1, 0,  0),
    DEFINE_DMA_CHANNEL(DMA1, 1,  6),
    DEFINE_DMA_CHANNEL(DMA1, 2, 16),
    DEFINE_DMA_CHANNEL(DMA1, 3, 22),
    DEFINE_DMA_CHANNEL(DMA1, 4, 32),
    DEFINE_DMA_CHANNEL(DMA1, 5, 38),
    DEFINE_DMA_CHANNEL(DMA1, 6, 48),
    DEFINE_DMA_CHANNEL(DMA1, 7, 54),

    DEFINE_DMA_CHANNEL(DMA2, 0,  0),
    DEFINE_DMA_CHANNEL(DMA2, 1,  6),
    DEFINE_DMA_CHANNEL(DMA2, 2, 16),
    DEFINE_DMA_CHANNEL(DMA2, 3, 22),
    DEFINE_DMA_CHANNEL(DMA2, 4, 32),
    DEFINE_DMA_CHANNEL(DMA2, 5, 38),
    DEFINE_DMA_CHANNEL(DMA2, 6, 48),
    DEFINE_DMA_CHANNEL(DMA2, 7, 54),

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

static void enableDmaClock(int index)
{
    const uint32_t rcc = dmaDescriptors[index].dma == DMA1 ? RCC_AHB1ENR_DMA1EN : RCC_AHB1ENR_DMA2EN;
    do {
        __IO uint32_t tmpreg;
        SET_BIT(RCC->AHB1ENR, rcc);
        /* Delay after an RCC peripheral clock enabling */
        tmpreg = READ_BIT(RCC->AHB1ENR, rcc);
        UNUSED(tmpreg);
    } while (0);
}

void dmaInit(dmaIdentifier_e identifier, resourceOwner_e owner, uint8_t resourceIndex)
{
    const int index = DMA_IDENTIFIER_TO_INDEX(identifier);

    enableDmaClock(index);
    dmaDescriptors[index].owner = owner;
    dmaDescriptors[index].resourceIndex = resourceIndex;
}

void dmaSetHandler(dmaIdentifier_e identifier, dmaCallbackHandlerFuncPtr callback, uint32_t priority, uint32_t userParam)
{
    const int index = DMA_IDENTIFIER_TO_INDEX(identifier);

    enableDmaClock(index);
    dmaDescriptors[index].irqHandlerCallback = callback;
    dmaDescriptors[index].userParam = userParam;

    HAL_NVIC_SetPriority(dmaDescriptors[index].irqN, NVIC_PRIORITY_BASE(priority), NVIC_PRIORITY_SUB(priority));
    HAL_NVIC_EnableIRQ(dmaDescriptors[index].irqN);
}

resourceOwner_e dmaGetOwner(dmaIdentifier_e identifier)
{
    return dmaDescriptors[DMA_IDENTIFIER_TO_INDEX(identifier)].owner;
}

uint8_t dmaGetResourceIndex(dmaIdentifier_e identifier)
{
    return dmaDescriptors[DMA_IDENTIFIER_TO_INDEX(identifier)].resourceIndex;
}

dmaIdentifier_e dmaGetIdentifier(const DMA_Stream_TypeDef* stream)
{
    for (int i = 0; i < DMA_LAST_HANDLER; i++) {
        if (dmaDescriptors[i].ref == stream) {
            return i + 1;
        }
    }
    return 0;
}

DMA_Stream_TypeDef* dmaGetRefByIdentifier(const dmaIdentifier_e identifier)
{
    return dmaDescriptors[DMA_IDENTIFIER_TO_INDEX(identifier)].ref;
}

dmaChannelDescriptor_t* dmaGetDescriptorByIdentifier(const dmaIdentifier_e identifier)
{
    return &dmaDescriptors[DMA_IDENTIFIER_TO_INDEX(identifier)];
}

uint32_t dmaGetChannel(const uint8_t channel)
{
    return ((uint32_t)channel*2)<<24;
}
