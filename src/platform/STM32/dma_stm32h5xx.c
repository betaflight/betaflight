/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_DMA

#include "drivers/nvic.h"
#include "drivers/dma_impl.h"
#include "platform/dma.h"
#include "platform/rcc.h"
#include "drivers/resource.h"

/*
 * DMA descriptors.
 */
dmaChannelDescriptor_t dmaDescriptors[DMA_LAST_HANDLER] = {
    // GPDMA1 channels 0-7
    DEFINE_DMA_CHANNEL(GPDMA1, 0, 0),
    DEFINE_DMA_CHANNEL(GPDMA1, 1, 0),
    DEFINE_DMA_CHANNEL(GPDMA1, 2, 0),
    DEFINE_DMA_CHANNEL(GPDMA1, 3, 0),
    DEFINE_DMA_CHANNEL(GPDMA1, 4, 0),
    DEFINE_DMA_CHANNEL(GPDMA1, 5, 0),
    DEFINE_DMA_CHANNEL(GPDMA1, 6, 0),
    DEFINE_DMA_CHANNEL(GPDMA1, 7, 0),

    // GPDMA2 channels 0-7
    DEFINE_DMA_CHANNEL(GPDMA2, 0, 0),
    DEFINE_DMA_CHANNEL(GPDMA2, 1, 0),
    DEFINE_DMA_CHANNEL(GPDMA2, 2, 0),
    DEFINE_DMA_CHANNEL(GPDMA2, 3, 0),
    DEFINE_DMA_CHANNEL(GPDMA2, 4, 0),
    DEFINE_DMA_CHANNEL(GPDMA2, 5, 0),
    DEFINE_DMA_CHANNEL(GPDMA2, 6, 0),
    DEFINE_DMA_CHANNEL(GPDMA2, 7, 0),
};

/*
 * DMA IRQ Handlers
 */

// GPDMA1 channels 0-7
DEFINE_DMA_IRQ_HANDLER(GPDMA1, 0, GPDMA1_CH0_HANDLER)
DEFINE_DMA_IRQ_HANDLER(GPDMA1, 1, GPDMA1_CH1_HANDLER)
DEFINE_DMA_IRQ_HANDLER(GPDMA1, 2, GPDMA1_CH2_HANDLER)
DEFINE_DMA_IRQ_HANDLER(GPDMA1, 3, GPDMA1_CH3_HANDLER)
DEFINE_DMA_IRQ_HANDLER(GPDMA1, 4, GPDMA1_CH4_HANDLER)
DEFINE_DMA_IRQ_HANDLER(GPDMA1, 5, GPDMA1_CH5_HANDLER)
DEFINE_DMA_IRQ_HANDLER(GPDMA1, 6, GPDMA1_CH6_HANDLER)
DEFINE_DMA_IRQ_HANDLER(GPDMA1, 7, GPDMA1_CH7_HANDLER)

// GPDMA2 channels 0-7
DEFINE_DMA_IRQ_HANDLER(GPDMA2, 0, GPDMA2_CH0_HANDLER)
DEFINE_DMA_IRQ_HANDLER(GPDMA2, 1, GPDMA2_CH1_HANDLER)
DEFINE_DMA_IRQ_HANDLER(GPDMA2, 2, GPDMA2_CH2_HANDLER)
DEFINE_DMA_IRQ_HANDLER(GPDMA2, 3, GPDMA2_CH3_HANDLER)
DEFINE_DMA_IRQ_HANDLER(GPDMA2, 4, GPDMA2_CH4_HANDLER)
DEFINE_DMA_IRQ_HANDLER(GPDMA2, 5, GPDMA2_CH5_HANDLER)
DEFINE_DMA_IRQ_HANDLER(GPDMA2, 6, GPDMA2_CH6_HANDLER)
DEFINE_DMA_IRQ_HANDLER(GPDMA2, 7, GPDMA2_CH7_HANDLER)

static void enableDmaClock(int index)
{
    if (dmaDescriptors[index].dma == GPDMA2) {
        RCC_ClockCmd(RCC_AHB1(GPDMA2), ENABLE);
    } else {
        RCC_ClockCmd(RCC_AHB1(GPDMA1), ENABLE);
    }
}

void dmaEnable(dmaIdentifier_e identifier)
{
    const int index = DMA_IDENTIFIER_TO_INDEX(identifier);

    enableDmaClock(index);
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

int dmaGetHandlerCount(void)
{
    return DMA_LAST_HANDLER;
}

int dmaGetDeviceNumber(dmaIdentifier_e identifier)
{
    return DMA_DEVICE_NO(identifier);
}

int dmaGetDeviceIndex(dmaIdentifier_e identifier)
{
    return DMA_DEVICE_INDEX(identifier);
}

const char *dmaGetDisplayString(void)
{
    return DMA_OUTPUT_STRING;
}

uint32_t dmaGetDataLength(dmaResource_t *ref)
{
    return LL_EX_DMA_GetDataLength((DMA_ARCH_TYPE *)ref);
}

#endif
