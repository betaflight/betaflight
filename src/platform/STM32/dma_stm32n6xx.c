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
    // HPDMA1 channels 0-15
    DEFINE_DMA_CHANNEL(HPDMA1,  0, 0),
    DEFINE_DMA_CHANNEL(HPDMA1,  1, 0),
    DEFINE_DMA_CHANNEL(HPDMA1,  2, 0),
    DEFINE_DMA_CHANNEL(HPDMA1,  3, 0),
    DEFINE_DMA_CHANNEL(HPDMA1,  4, 0),
    DEFINE_DMA_CHANNEL(HPDMA1,  5, 0),
    DEFINE_DMA_CHANNEL(HPDMA1,  6, 0),
    DEFINE_DMA_CHANNEL(HPDMA1,  7, 0),
    DEFINE_DMA_CHANNEL(HPDMA1,  8, 0),
    DEFINE_DMA_CHANNEL(HPDMA1,  9, 0),
    DEFINE_DMA_CHANNEL(HPDMA1, 10, 0),
    DEFINE_DMA_CHANNEL(HPDMA1, 11, 0),
    DEFINE_DMA_CHANNEL(HPDMA1, 12, 0),
    DEFINE_DMA_CHANNEL(HPDMA1, 13, 0),
    DEFINE_DMA_CHANNEL(HPDMA1, 14, 0),
    DEFINE_DMA_CHANNEL(HPDMA1, 15, 0),

    // GPDMA1 channels 0-15
    DEFINE_DMA_CHANNEL(GPDMA1,  0, 0),
    DEFINE_DMA_CHANNEL(GPDMA1,  1, 0),
    DEFINE_DMA_CHANNEL(GPDMA1,  2, 0),
    DEFINE_DMA_CHANNEL(GPDMA1,  3, 0),
    DEFINE_DMA_CHANNEL(GPDMA1,  4, 0),
    DEFINE_DMA_CHANNEL(GPDMA1,  5, 0),
    DEFINE_DMA_CHANNEL(GPDMA1,  6, 0),
    DEFINE_DMA_CHANNEL(GPDMA1,  7, 0),
    DEFINE_DMA_CHANNEL(GPDMA1,  8, 0),
    DEFINE_DMA_CHANNEL(GPDMA1,  9, 0),
    DEFINE_DMA_CHANNEL(GPDMA1, 10, 0),
    DEFINE_DMA_CHANNEL(GPDMA1, 11, 0),
    DEFINE_DMA_CHANNEL(GPDMA1, 12, 0),
    DEFINE_DMA_CHANNEL(GPDMA1, 13, 0),
    DEFINE_DMA_CHANNEL(GPDMA1, 14, 0),
    DEFINE_DMA_CHANNEL(GPDMA1, 15, 0),
};

/*
 * DMA IRQ Handlers
 */

// HPDMA1 channels 0-15
DEFINE_DMA_IRQ_HANDLER(HPDMA1,  0, HPDMA1_CH0_HANDLER)
DEFINE_DMA_IRQ_HANDLER(HPDMA1,  1, HPDMA1_CH1_HANDLER)
DEFINE_DMA_IRQ_HANDLER(HPDMA1,  2, HPDMA1_CH2_HANDLER)
DEFINE_DMA_IRQ_HANDLER(HPDMA1,  3, HPDMA1_CH3_HANDLER)
DEFINE_DMA_IRQ_HANDLER(HPDMA1,  4, HPDMA1_CH4_HANDLER)
DEFINE_DMA_IRQ_HANDLER(HPDMA1,  5, HPDMA1_CH5_HANDLER)
DEFINE_DMA_IRQ_HANDLER(HPDMA1,  6, HPDMA1_CH6_HANDLER)
DEFINE_DMA_IRQ_HANDLER(HPDMA1,  7, HPDMA1_CH7_HANDLER)
DEFINE_DMA_IRQ_HANDLER(HPDMA1,  8, HPDMA1_CH8_HANDLER)
DEFINE_DMA_IRQ_HANDLER(HPDMA1,  9, HPDMA1_CH9_HANDLER)
DEFINE_DMA_IRQ_HANDLER(HPDMA1, 10, HPDMA1_CH10_HANDLER)
DEFINE_DMA_IRQ_HANDLER(HPDMA1, 11, HPDMA1_CH11_HANDLER)
DEFINE_DMA_IRQ_HANDLER(HPDMA1, 12, HPDMA1_CH12_HANDLER)
DEFINE_DMA_IRQ_HANDLER(HPDMA1, 13, HPDMA1_CH13_HANDLER)
DEFINE_DMA_IRQ_HANDLER(HPDMA1, 14, HPDMA1_CH14_HANDLER)
DEFINE_DMA_IRQ_HANDLER(HPDMA1, 15, HPDMA1_CH15_HANDLER)

// GPDMA1 channels 0-15
DEFINE_DMA_IRQ_HANDLER(GPDMA1,  0, GPDMA1_CH0_HANDLER)
DEFINE_DMA_IRQ_HANDLER(GPDMA1,  1, GPDMA1_CH1_HANDLER)
DEFINE_DMA_IRQ_HANDLER(GPDMA1,  2, GPDMA1_CH2_HANDLER)
DEFINE_DMA_IRQ_HANDLER(GPDMA1,  3, GPDMA1_CH3_HANDLER)
DEFINE_DMA_IRQ_HANDLER(GPDMA1,  4, GPDMA1_CH4_HANDLER)
DEFINE_DMA_IRQ_HANDLER(GPDMA1,  5, GPDMA1_CH5_HANDLER)
DEFINE_DMA_IRQ_HANDLER(GPDMA1,  6, GPDMA1_CH6_HANDLER)
DEFINE_DMA_IRQ_HANDLER(GPDMA1,  7, GPDMA1_CH7_HANDLER)
DEFINE_DMA_IRQ_HANDLER(GPDMA1,  8, GPDMA1_CH8_HANDLER)
DEFINE_DMA_IRQ_HANDLER(GPDMA1,  9, GPDMA1_CH9_HANDLER)
DEFINE_DMA_IRQ_HANDLER(GPDMA1, 10, GPDMA1_CH10_HANDLER)
DEFINE_DMA_IRQ_HANDLER(GPDMA1, 11, GPDMA1_CH11_HANDLER)
DEFINE_DMA_IRQ_HANDLER(GPDMA1, 12, GPDMA1_CH12_HANDLER)
DEFINE_DMA_IRQ_HANDLER(GPDMA1, 13, GPDMA1_CH13_HANDLER)
DEFINE_DMA_IRQ_HANDLER(GPDMA1, 14, GPDMA1_CH14_HANDLER)
DEFINE_DMA_IRQ_HANDLER(GPDMA1, 15, GPDMA1_CH15_HANDLER)

static void enableDmaClock(int index)
{
    if (dmaDescriptors[index].dma == HPDMA1) {
        RCC_ClockCmd(RCC_AHB5(HPDMA1), ENABLE);
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
