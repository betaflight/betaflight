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
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_DMA

#include "drivers/dma.h"
#include "drivers/dma_impl.h"
#include "drivers/nvic.h"
#include "platform/dma.h"
#include "platform/rcc.h"

// X32M7 DMA counts UP (bytes transferred). serial_uart.c expects DOWN-counting
// (remaining bytes). Cache blockSize so dmaGetDataLength can return blockSize - transferred.
static uint16_t dmaBlockSizeCache[DMA_LAST_HANDLER];

void dmaX32StoreBlockSize(void *ref, uint32_t size)
{
    const uint32_t index = dmaX32MuxChannelFromResource((DMA_ARCH_TYPE *)ref);
    if (index < DMA_LAST_HANDLER) {
        dmaBlockSizeCache[index] = (uint16_t)size;
    }
}

// RX DMA channel buffer registration for DCACHE invalidation.
// TX channels are never registered (addr stays 0), so dmaGetDataLength skips them.
typedef struct {
    uint32_t addr;
    uint32_t size;
} dmaRxBuf_t;

static dmaRxBuf_t dmaRxBufCache[DMA_LAST_HANDLER];

void dmaX32RegisterRxBuf(void *ref, void *buf, uint32_t size)
{
    const uint32_t index = dmaX32MuxChannelFromResource((DMA_ARCH_TYPE *)ref);
    if (index < DMA_LAST_HANDLER) {
        dmaRxBufCache[index].addr = (uint32_t)buf;
        dmaRxBufCache[index].size = size;
    }
}
dmaChannelDescriptor_t dmaDescriptors[DMA_LAST_HANDLER] = {
    DEFINE_DMA_CHANNEL(DMA1, 0, 0),
    DEFINE_DMA_CHANNEL(DMA1, 1, 0),
    DEFINE_DMA_CHANNEL(DMA1, 2, 0),
    DEFINE_DMA_CHANNEL(DMA1, 3, 0),
    DEFINE_DMA_CHANNEL(DMA1, 4, 0),
    DEFINE_DMA_CHANNEL(DMA1, 5, 0),
    DEFINE_DMA_CHANNEL(DMA1, 6, 0),
    DEFINE_DMA_CHANNEL(DMA1, 7, 0),

    DEFINE_DMA_CHANNEL(DMA2, 0, 0),
    DEFINE_DMA_CHANNEL(DMA2, 1, 0),
    DEFINE_DMA_CHANNEL(DMA2, 2, 0),
    DEFINE_DMA_CHANNEL(DMA2, 3, 0),
    DEFINE_DMA_CHANNEL(DMA2, 4, 0),
    DEFINE_DMA_CHANNEL(DMA2, 5, 0),
    DEFINE_DMA_CHANNEL(DMA2, 6, 0),
    DEFINE_DMA_CHANNEL(DMA2, 7, 0),

    DEFINE_DMA_CHANNEL(DMA3, 0, 0),
    DEFINE_DMA_CHANNEL(DMA3, 1, 0),
    DEFINE_DMA_CHANNEL(DMA3, 2, 0),
    DEFINE_DMA_CHANNEL(DMA3, 3, 0),
    DEFINE_DMA_CHANNEL(DMA3, 4, 0),
    DEFINE_DMA_CHANNEL(DMA3, 5, 0),
    DEFINE_DMA_CHANNEL(DMA3, 6, 0),
    DEFINE_DMA_CHANNEL(DMA3, 7, 0),
};

DEFINE_DMA_IRQ_HANDLER(1, 0, DMA1_CH0_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 1, DMA1_CH1_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 2, DMA1_CH2_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 3, DMA1_CH3_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 4, DMA1_CH4_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 5, DMA1_CH5_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 6, DMA1_CH6_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 7, DMA1_CH7_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 0, DMA2_CH0_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 1, DMA2_CH1_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 2, DMA2_CH2_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 3, DMA2_CH3_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 4, DMA2_CH4_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 5, DMA2_CH5_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 6, DMA2_CH6_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 7, DMA2_CH7_HANDLER)
DEFINE_DMA_IRQ_HANDLER(3, 0, DMA3_CH0_HANDLER)
DEFINE_DMA_IRQ_HANDLER(3, 1, DMA3_CH1_HANDLER)
DEFINE_DMA_IRQ_HANDLER(3, 2, DMA3_CH2_HANDLER)
DEFINE_DMA_IRQ_HANDLER(3, 3, DMA3_CH3_HANDLER)
DEFINE_DMA_IRQ_HANDLER(3, 4, DMA3_CH4_HANDLER)
DEFINE_DMA_IRQ_HANDLER(3, 5, DMA3_CH5_HANDLER)
DEFINE_DMA_IRQ_HANDLER(3, 6, DMA3_CH6_HANDLER)
DEFINE_DMA_IRQ_HANDLER(3, 7, DMA3_CH7_HANDLER)

static void enableDmaClock(const dmaChannelDescriptor_t *descriptor)
{
    RCC_ClockCmd(RCC_AHB1_1(DMAMUX1), ENABLE);

    if (descriptor->dma == DMA1) {
        RCC_ClockCmd(RCC_AHB1_3(DMA1), ENABLE);
    } else if (descriptor->dma == DMA2) {
        RCC_ClockCmd(RCC_AHB1_3(DMA2), ENABLE);
    } else if (descriptor->dma == DMA3) {
        RCC_ClockCmd(RCC_AHB1_3(DMA3), ENABLE);
    }

    DMA_ControllerCmd(descriptor->dma, ENABLE);
}

void dmaEnable(dmaIdentifier_e identifier)
{
    const int index = DMA_IDENTIFIER_TO_INDEX(identifier);
    enableDmaClock(&dmaDescriptors[index]);
}

void dmaMuxEnable(dmaIdentifier_e identifier, uint32_t dmaMuxId)
{
    const int index = DMA_IDENTIFIER_TO_INDEX(identifier);
    const dmaChannelDescriptor_t *descriptor = &dmaDescriptors[index];

    enableDmaClock(descriptor);
    DMAMUX_SetRequestID(DMAMUX1_ID, dmaX32MuxChannelFromResource((DMA_ARCH_TYPE *)descriptor->ref), dmaMuxId);
}

void dmaSetHandler(dmaIdentifier_e identifier, dmaCallbackHandlerFuncPtr callback, uint32_t priority, uint32_t userParam)
{
    const int index = DMA_IDENTIFIER_TO_INDEX(identifier);
    dmaChannelDescriptor_t *descriptor = &dmaDescriptors[index];

    enableDmaClock(descriptor);
    descriptor->irqHandlerCallback = callback;
    descriptor->userParam = userParam;

    NVIC_InitType nvicInit = {
        .NVIC_IRQChannel = (uint8_t)descriptor->irqN,
        .NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(priority),
        .NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(priority),
        .NVIC_IRQChannelCmd = ENABLE,
    };
    NVIC_Init(&nvicInit);
}

void dmaX32ClearFlag(const dmaChannelDescriptor_t *descriptor, uint32_t flags)
{
    const uint32_t events = dmaX32EventFromFlags(flags);

    if (events) {
        DMA_ClearChannelEventStatus(descriptor->dma, (DMA_ChNumType)descriptor->stream, events);
    }
}

uint32_t dmaX32GetFlagStatus(const dmaChannelDescriptor_t *descriptor, uint32_t flags)
{
    uint32_t result = 0;
    const DMA_ChNumType channel = (DMA_ChNumType)descriptor->stream;
    if (DMA_GetCombinedStatus(descriptor->dma))
    {
        if ((flags & DMA_IT_TCIF) && DMA_GetChannelIntTfrStatus(descriptor->dma, channel) == SET)
        {
             result |= DMA_IT_TCIF;
        }

        if (DMA_GetChannelIntErrStatus(descriptor->dma, channel) == SET)
        {
            result |= (flags & (DMA_IT_TEIF | DMA_IT_DMEIF | DMA_IT_FEIF));
        }
    }

    return result;
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
    const uint32_t index = dmaX32MuxChannelFromResource((DMA_ARCH_TYPE *)ref);
    const uint32_t blockSize = (index < DMA_LAST_HANDLER) ? dmaBlockSizeCache[index] : 0;

#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    // Invalidate the RX buffer cache before serial_uart.c reads it so the CPU
    // always sees DMA-written data, not a stale cached copy.
    // TX channels have addr == 0 and are skipped.
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U
            && index < DMA_LAST_HANDLER && dmaRxBufCache[index].addr != 0U) {
        const uint32_t base = dmaRxBufCache[index].addr & ~(uint32_t)CACHE_LINE_MASK;
        const int32_t  len  = (int32_t)(((dmaRxBufCache[index].addr - base)
                                          + dmaRxBufCache[index].size
                                          + CACHE_LINE_SIZE - 1U)
                                         & ~(uint32_t)CACHE_LINE_MASK);
        SCB_InvalidateDCache_by_Addr((uint32_t *)base, len);
    }
#endif

    const uint32_t transferred = xDMA_GetCurrDataCounter(ref);
    return blockSize > transferred ? blockSize - transferred : 0;
}

#endif
