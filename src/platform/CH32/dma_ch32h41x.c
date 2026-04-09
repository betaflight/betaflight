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
/*
 * porting for ch32h41x by Temperslee
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
    DEFINE_DMA_CHANNEL(DMA1, 1,  0),
    DEFINE_DMA_CHANNEL(DMA1, 2,  4),
    DEFINE_DMA_CHANNEL(DMA1, 3,  8),
    DEFINE_DMA_CHANNEL(DMA1, 4, 12),
    DEFINE_DMA_CHANNEL(DMA1, 5, 16),
    DEFINE_DMA_CHANNEL(DMA1, 6, 20),
    DEFINE_DMA_CHANNEL(DMA1, 7, 24),
    DEFINE_DMA_CHANNEL(DMA1, 8, 28),
    DEFINE_DMA_CHANNEL(DMA2, 1,  0),
    DEFINE_DMA_CHANNEL(DMA2, 2,  4),
    DEFINE_DMA_CHANNEL(DMA2, 3,  8),
    DEFINE_DMA_CHANNEL(DMA2, 4, 12),
    DEFINE_DMA_CHANNEL(DMA2, 5, 16),
    DEFINE_DMA_CHANNEL(DMA2, 6, 20),
    DEFINE_DMA_CHANNEL(DMA2, 7, 24),
    DEFINE_DMA_CHANNEL(DMA2, 8, 28),
};

/*
 * DMA IRQ Handlers
 * if use hardware push&pop,key woard "WCH-Interrupt-fast" is needed 
 */
__FAST_INTERRUPT DEFINE_DMA_IRQ_HANDLER(1, 1, DMA1_CH1_HANDLER)
__FAST_INTERRUPT DEFINE_DMA_IRQ_HANDLER(1, 2, DMA1_CH2_HANDLER)
__FAST_INTERRUPT DEFINE_DMA_IRQ_HANDLER(1, 3, DMA1_CH3_HANDLER)
__FAST_INTERRUPT DEFINE_DMA_IRQ_HANDLER(1, 4, DMA1_CH4_HANDLER)
__FAST_INTERRUPT DEFINE_DMA_IRQ_HANDLER(1, 5, DMA1_CH5_HANDLER)
__FAST_INTERRUPT DEFINE_DMA_IRQ_HANDLER(1, 6, DMA1_CH6_HANDLER)
__FAST_INTERRUPT DEFINE_DMA_IRQ_HANDLER(1, 7, DMA1_CH7_HANDLER)
__FAST_INTERRUPT DEFINE_DMA_IRQ_HANDLER(1, 8, DMA1_CH8_HANDLER)
__FAST_INTERRUPT DEFINE_DMA_IRQ_HANDLER(2, 1, DMA2_CH1_HANDLER)
__FAST_INTERRUPT DEFINE_DMA_IRQ_HANDLER(2, 2, DMA2_CH2_HANDLER)
__FAST_INTERRUPT DEFINE_DMA_IRQ_HANDLER(2, 3, DMA2_CH3_HANDLER)
__FAST_INTERRUPT DEFINE_DMA_IRQ_HANDLER(2, 4, DMA2_CH4_HANDLER)
__FAST_INTERRUPT DEFINE_DMA_IRQ_HANDLER(2, 5, DMA2_CH5_HANDLER)
__FAST_INTERRUPT DEFINE_DMA_IRQ_HANDLER(2, 6, DMA2_CH6_HANDLER)
__FAST_INTERRUPT DEFINE_DMA_IRQ_HANDLER(2, 7, DMA2_CH7_HANDLER)
__FAST_INTERRUPT DEFINE_DMA_IRQ_HANDLER(2, 8, DMA2_CH8_HANDLER)


static void enableDmaClock(int index)
{
    RCC_ClockCmd(dmaDescriptors[index].dma == DMA1 ?  RCC_HB(DMA1) : RCC_HB(DMA2), ENABLE);
}

void dmaEnable(dmaIdentifier_e identifier)
{
    const int index = DMA_IDENTIFIER_TO_INDEX(identifier);

    enableDmaClock(index);
}

void dmaMuxEnable(dmaIdentifier_e identifier, uint32_t  dmaMuxId)
{
    const int index = DMA_IDENTIFIER_TO_INDEX(identifier);
    // dmamux_init(dmaDescriptors[index].dmamux, dmaMuxId);
    DMA_MuxChannelConfig(index, dmaMuxId);
}

void dmaSetHandler(dmaIdentifier_e identifier, dmaCallbackHandlerFuncPtr callback, uint32_t priority, uint32_t userParam)
{
    const int index = DMA_IDENTIFIER_TO_INDEX(identifier);

    enableDmaClock(index);
    dmaDescriptors[index].irqHandlerCallback = callback;
    dmaDescriptors[index].userParam = userParam;

    // NVIC_SetPriority(dmaDescriptors[index].irqN, NVIC_PRIORITY_BASE(priority), NVIC_PRIORITY_SUB(priority));
    NVIC_SetPriority(dmaDescriptors[index].irqN, priority);
    NVIC_EnableIRQ(dmaDescriptors[index].irqN);
}
#endif
