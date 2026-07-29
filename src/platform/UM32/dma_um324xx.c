/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
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
    DEFINE_DMA_CHANNEL(DMA1, 0, 0),
    DEFINE_DMA_CHANNEL(DMA1, 1, 1),
    DEFINE_DMA_CHANNEL(DMA1, 2, 2),
    DEFINE_DMA_CHANNEL(DMA1, 3, 3),
    DEFINE_DMA_CHANNEL(DMA1, 4, 4),
    DEFINE_DMA_CHANNEL(DMA1, 5, 5),
    DEFINE_DMA_CHANNEL(DMA1, 6, 6),
    DEFINE_DMA_CHANNEL(DMA1, 7, 7),

    DEFINE_DMA_CHANNEL(DMA2, 0, 0),
    DEFINE_DMA_CHANNEL(DMA2, 1, 1),
    DEFINE_DMA_CHANNEL(DMA2, 2, 2),
    DEFINE_DMA_CHANNEL(DMA2, 3, 3),
    DEFINE_DMA_CHANNEL(DMA2, 4, 4),
    DEFINE_DMA_CHANNEL(DMA2, 5, 5),
    DEFINE_DMA_CHANNEL(DMA2, 6, 6),
    DEFINE_DMA_CHANNEL(DMA2, 7, 7),
};

/*
 * DMA IRQ Handlers
 */
DEFINE_DMA_IRQ_HANDLER(DMA1, 0, DMA1_ST0_HANDLER)
DEFINE_DMA_IRQ_HANDLER(DMA1, 1, DMA1_ST1_HANDLER)
DEFINE_DMA_IRQ_HANDLER(DMA1, 2, DMA1_ST2_HANDLER)
DEFINE_DMA_IRQ_HANDLER(DMA1, 3, DMA1_ST3_HANDLER)
DEFINE_DMA_IRQ_HANDLER(DMA1, 4, DMA1_ST4_HANDLER)
DEFINE_DMA_IRQ_HANDLER(DMA1, 5, DMA1_ST5_HANDLER)
DEFINE_DMA_IRQ_HANDLER(DMA1, 6, DMA1_ST6_HANDLER)
DEFINE_DMA_IRQ_HANDLER(DMA1, 7, DMA1_ST7_HANDLER)
DEFINE_DMA_IRQ_HANDLER(DMA2, 0, DMA2_ST0_HANDLER)
DEFINE_DMA_IRQ_HANDLER(DMA2, 1, DMA2_ST1_HANDLER)
DEFINE_DMA_IRQ_HANDLER(DMA2, 2, DMA2_ST2_HANDLER)
DEFINE_DMA_IRQ_HANDLER(DMA2, 3, DMA2_ST3_HANDLER)
DEFINE_DMA_IRQ_HANDLER(DMA2, 4, DMA2_ST4_HANDLER)
DEFINE_DMA_IRQ_HANDLER(DMA2, 5, DMA2_ST5_HANDLER)
DEFINE_DMA_IRQ_HANDLER(DMA2, 6, DMA2_ST6_HANDLER)
DEFINE_DMA_IRQ_HANDLER(DMA2, 7, DMA2_ST7_HANDLER)



void dmaEnable(dmaIdentifier_e identifier)
{
    const int index = DMA_IDENTIFIER_TO_INDEX(identifier);
    RCC_ClockCmd(dmaDescriptors[index].dma == DMA1 ?  RCC_AHB0(DMA1) : RCC_AHB0(DMA2), ENABLE);
}

void dmaSetHandler(dmaIdentifier_e identifier, dmaCallbackHandlerFuncPtr callback, uint32_t priority, uint32_t userParam)
{
    const int index = DMA_IDENTIFIER_TO_INDEX(identifier);

    RCC_ClockCmd(dmaDescriptors[index].dma == DMA1 ?  RCC_AHB0(DMA1) : RCC_AHB0(DMA2), ENABLE);
    dmaDescriptors[index].irqHandlerCallback = callback;
    dmaDescriptors[index].userParam = userParam;

    HAL_NVIC_SetPriority(dmaDescriptors[index].irqN, NVIC_PRIORITY_BASE(priority), NVIC_PRIORITY_SUB(priority));
    HAL_NVIC_EnableIRQ(dmaDescriptors[index].irqN);
}

uint32_t dmaGetInstance(dmaIdentifier_e identifier)
{
    const int index = DMA_IDENTIFIER_TO_INDEX(identifier);
    return (uint32_t)(dmaDescriptors[index].ref) & 0xFFFF0000;
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
    return xLL_EX_DMA_GetDataLength((DMA_ARCH_TYPE *)ref);
}
#endif
