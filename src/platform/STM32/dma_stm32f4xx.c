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
#include "drivers/dma.h"
#include "drivers/resource.h"

/*
 * DMA descriptors.
 */
dmaChannelDescriptor_t dmaDescriptors[DMA_LAST_HANDLER] = {
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

#define DMA_RCC(x) ((x) == DMA1 ? RCC_AHB1Periph_DMA1 : RCC_AHB1Periph_DMA2)
void dmaEnable(dmaIdentifier_e identifier)
{
    const int index = DMA_IDENTIFIER_TO_INDEX(identifier);
    RCC_AHB1PeriphClockCmd(DMA_RCC(dmaDescriptors[index].dma), ENABLE);
}

#define RETURN_TCIF_FLAG(s, n) if (s == DMA1_Stream ## n || s == DMA2_Stream ## n) return DMA_IT_TCIF ## n

uint32_t dmaFlag_IT_TCIF(const dmaResource_t *stream)
{
    RETURN_TCIF_FLAG((DMA_ARCH_TYPE *)stream, 0);
    RETURN_TCIF_FLAG((DMA_ARCH_TYPE *)stream, 1);
    RETURN_TCIF_FLAG((DMA_ARCH_TYPE *)stream, 2);
    RETURN_TCIF_FLAG((DMA_ARCH_TYPE *)stream, 3);
    RETURN_TCIF_FLAG((DMA_ARCH_TYPE *)stream, 4);
    RETURN_TCIF_FLAG((DMA_ARCH_TYPE *)stream, 5);
    RETURN_TCIF_FLAG((DMA_ARCH_TYPE *)stream, 6);
    RETURN_TCIF_FLAG((DMA_ARCH_TYPE *)stream, 7);
    return 0;
}

void dmaSetHandler(dmaIdentifier_e identifier, dmaCallbackHandlerFuncPtr callback, uint32_t priority, uint32_t userParam)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    const int index = DMA_IDENTIFIER_TO_INDEX(identifier);

    RCC_AHB1PeriphClockCmd(DMA_RCC(dmaDescriptors[index].dma), ENABLE);
    dmaDescriptors[index].irqHandlerCallback = callback;
    dmaDescriptors[index].userParam = userParam;
    dmaDescriptors[index].completeFlag = dmaFlag_IT_TCIF(dmaDescriptors[index].ref);

    NVIC_InitStructure.NVIC_IRQChannel = dmaDescriptors[index].irqN;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(priority);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(priority);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
#endif
