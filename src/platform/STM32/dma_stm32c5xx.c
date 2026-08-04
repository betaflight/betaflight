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
    // LPDMA1 channels 0-7
    DEFINE_DMA_CHANNEL(LPDMA1, 0, 0),
    DEFINE_DMA_CHANNEL(LPDMA1, 1, 0),
    DEFINE_DMA_CHANNEL(LPDMA1, 2, 0),
    DEFINE_DMA_CHANNEL(LPDMA1, 3, 0),
    DEFINE_DMA_CHANNEL(LPDMA1, 4, 0),
    DEFINE_DMA_CHANNEL(LPDMA1, 5, 0),
    DEFINE_DMA_CHANNEL(LPDMA1, 6, 0),
    DEFINE_DMA_CHANNEL(LPDMA1, 7, 0),

    // LPDMA2 channels 0-7
    DEFINE_DMA_CHANNEL(LPDMA2, 0, 0),
    DEFINE_DMA_CHANNEL(LPDMA2, 1, 0),
    DEFINE_DMA_CHANNEL(LPDMA2, 2, 0),
    DEFINE_DMA_CHANNEL(LPDMA2, 3, 0),
#if !defined(STM32C562xx)
    // STM32C562 LPDMA2 only has channels 0-3
    DEFINE_DMA_CHANNEL(LPDMA2, 4, 0),
    DEFINE_DMA_CHANNEL(LPDMA2, 5, 0),
    DEFINE_DMA_CHANNEL(LPDMA2, 6, 0),
    DEFINE_DMA_CHANNEL(LPDMA2, 7, 0),
#endif
};

/*
 * DMA IRQ Handlers
 */

// LPDMA1 channels 0-7
DEFINE_DMA_IRQ_HANDLER(LPDMA1, 0, LPDMA1_CH0_HANDLER)
DEFINE_DMA_IRQ_HANDLER(LPDMA1, 1, LPDMA1_CH1_HANDLER)
DEFINE_DMA_IRQ_HANDLER(LPDMA1, 2, LPDMA1_CH2_HANDLER)
DEFINE_DMA_IRQ_HANDLER(LPDMA1, 3, LPDMA1_CH3_HANDLER)
DEFINE_DMA_IRQ_HANDLER(LPDMA1, 4, LPDMA1_CH4_HANDLER)
DEFINE_DMA_IRQ_HANDLER(LPDMA1, 5, LPDMA1_CH5_HANDLER)
DEFINE_DMA_IRQ_HANDLER(LPDMA1, 6, LPDMA1_CH6_HANDLER)
DEFINE_DMA_IRQ_HANDLER(LPDMA1, 7, LPDMA1_CH7_HANDLER)

// LPDMA2 channels 0-7
DEFINE_DMA_IRQ_HANDLER(LPDMA2, 0, LPDMA2_CH0_HANDLER)
DEFINE_DMA_IRQ_HANDLER(LPDMA2, 1, LPDMA2_CH1_HANDLER)
DEFINE_DMA_IRQ_HANDLER(LPDMA2, 2, LPDMA2_CH2_HANDLER)
DEFINE_DMA_IRQ_HANDLER(LPDMA2, 3, LPDMA2_CH3_HANDLER)
#if !defined(STM32C562xx)
DEFINE_DMA_IRQ_HANDLER(LPDMA2, 4, LPDMA2_CH4_HANDLER)
DEFINE_DMA_IRQ_HANDLER(LPDMA2, 5, LPDMA2_CH5_HANDLER)
DEFINE_DMA_IRQ_HANDLER(LPDMA2, 6, LPDMA2_CH6_HANDLER)
DEFINE_DMA_IRQ_HANDLER(LPDMA2, 7, LPDMA2_CH7_HANDLER)
#endif

static void enableDmaClock(int index)
{
    if (dmaDescriptors[index].dma == LPDMA2) {
        RCC_ClockCmd(RCC_AHB1(LPDMA2), ENABLE);
    } else {
        RCC_ClockCmd(RCC_AHB1(LPDMA1), ENABLE);
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
