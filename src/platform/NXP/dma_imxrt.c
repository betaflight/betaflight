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
 * eDMA and DMAMUX for i.MXRT: one engine with 32 channels, each fed by a DMAMUX
 * request source. All memory the engine touches must be in DTCM (DMA_DATA,
 * DMA_DATA_ZERO_INIT or DMA_TCD_LIST), which is uncached.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_DMA

#include "common/utils.h"

#include "drivers/dma.h"
#include "drivers/dma_impl.h"
#include "drivers/resource.h"

#include "platform/dma.h"
#include "platform/nxp_clock.h"

STATIC_ASSERT(sizeof(dmaTcd_t) == DMA_TCD_ALIGNMENT, dmaTcd_t_must_match_the_hardware_TCD);
STATIC_ASSERT(__alignof__(dmaTcd_t) == DMA_TCD_ALIGNMENT, dmaTcd_t_must_be_32_byte_aligned);
STATIC_ASSERT(NXP_EDMA_CHANNEL_COUNT == DMA_TCD_COUNT, channel_count_must_match_the_vendor_header);

// Allocation bookkeeping lives in drivers/dma.c, which indexes this array.
dmaChannelDescriptor_t dmaDescriptors[NXP_EDMA_CHANNEL_COUNT] = {
    DEFINE_DMA_CHANNEL(0),  DEFINE_DMA_CHANNEL(1),  DEFINE_DMA_CHANNEL(2),  DEFINE_DMA_CHANNEL(3),
    DEFINE_DMA_CHANNEL(4),  DEFINE_DMA_CHANNEL(5),  DEFINE_DMA_CHANNEL(6),  DEFINE_DMA_CHANNEL(7),
    DEFINE_DMA_CHANNEL(8),  DEFINE_DMA_CHANNEL(9),  DEFINE_DMA_CHANNEL(10), DEFINE_DMA_CHANNEL(11),
    DEFINE_DMA_CHANNEL(12), DEFINE_DMA_CHANNEL(13), DEFINE_DMA_CHANNEL(14), DEFINE_DMA_CHANNEL(15),
    DEFINE_DMA_CHANNEL(16), DEFINE_DMA_CHANNEL(17), DEFINE_DMA_CHANNEL(18), DEFINE_DMA_CHANNEL(19),
    DEFINE_DMA_CHANNEL(20), DEFINE_DMA_CHANNEL(21), DEFINE_DMA_CHANNEL(22), DEFINE_DMA_CHANNEL(23),
    DEFINE_DMA_CHANNEL(24), DEFINE_DMA_CHANNEL(25), DEFINE_DMA_CHANNEL(26), DEFINE_DMA_CHANNEL(27),
    DEFINE_DMA_CHANNEL(28), DEFINE_DMA_CHANNEL(29), DEFINE_DMA_CHANNEL(30), DEFINE_DMA_CHANNEL(31),
};

uint32_t dmaLastErrorStatus;

// The faulting channel's bit while its callback runs, read by DMA_GET_FLAG_STATUS().
uint32_t dmaErrorChannel;

static bool edmaEngineStarted;

// Most urgent priority any registered channel asked for; 0xff is least urgent.
static uint8_t edmaErrorPriority = 0xff;

// Per vector state, indexed by the low four bits of the channel number.
#define EDMA_CHANNEL_VECTORS    16

// The one channel that can raise a vector, so its handler can skip reading INT.
#define EDMA_VECTOR_FREE        0xffu
#define EDMA_VECTOR_SHARED      0xfeu

static uint8_t edmaVectorOwner[EDMA_CHANNEL_VECTORS] = {
    EDMA_VECTOR_FREE, EDMA_VECTOR_FREE, EDMA_VECTOR_FREE, EDMA_VECTOR_FREE,
    EDMA_VECTOR_FREE, EDMA_VECTOR_FREE, EDMA_VECTOR_FREE, EDMA_VECTOR_FREE,
    EDMA_VECTOR_FREE, EDMA_VECTOR_FREE, EDMA_VECTOR_FREE, EDMA_VECTOR_FREE,
    EDMA_VECTOR_FREE, EDMA_VECTOR_FREE, EDMA_VECTOR_FREE, EDMA_VECTOR_FREE,
};

// A second claimant makes the vector shared and its handler reads INT.
static void edmaClaimVector(unsigned channel)
{
    const unsigned vector = channel & (EDMA_CHANNEL_VECTORS - 1);

    if (edmaVectorOwner[vector] == EDMA_VECTOR_FREE) {
        edmaVectorOwner[vector] = (uint8_t)channel;
    } else if (edmaVectorOwner[vector] != (uint8_t)channel) {
        edmaVectorOwner[vector] = EDMA_VECTOR_SHARED;
    }
}
static uint8_t edmaChannelPriority[EDMA_CHANNEL_VECTORS] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
};

static volatile dmaTcd_t *edmaChannelTcd(unsigned channel)
{
    return (volatile dmaTcd_t *)&DMA0->TCD[channel];
}

static bool edmaValidChannel(dmaIdentifier_e identifier)
{
    return identifier >= DMA_FIRST_HANDLER && identifier <= DMA_LAST_HANDLER;
}

// CR mode bits this driver owns; EMLM must stay clear or NBYTES changes meaning.
#define EDMA_CR_MODE_MASK (DMA_CR_EDBG_MASK | DMA_CR_ERCA_MASK | DMA_CR_HOE_MASK \
                           | DMA_CR_HALT_MASK | DMA_CR_CLM_MASK | DMA_CR_EMLM_MASK)

// Enable the clock and set the engine once: stop under debug, round robin
// arbitration, no halt on error (which would stop the whole engine).
static void edmaStart(void)
{
    nxpClockGateEnable(NXP_CLK_EDMA);

    if (!edmaEngineStarted) {
        DMA0->CR = (DMA0->CR & ~EDMA_CR_MODE_MASK) | DMA_CR_EDBG_MASK | DMA_CR_ERCA_MASK;
        edmaEngineStarted = true;
    }
}

void dmaEnable(dmaIdentifier_e identifier)
{
    if (!edmaValidChannel(identifier)) {
        return;
    }

    const unsigned channel = DMA_IDENTIFIER_TO_CHANNEL(identifier);

    edmaStart();

    // These command registers take a channel number, not a mask.
    DMA0->CERQ = channel;
    DMA0->CINT = channel;
    DMA0->CERR = channel;
    DMAMUX->CHCFG[channel] = 0;
}

// The mux source may only change while the mux channel is off.
void dmaMuxEnable(dmaIdentifier_e identifier, uint32_t dmaMuxId)
{
    if (!edmaValidChannel(identifier)) {
        return;
    }

    const unsigned channel = DMA_IDENTIFIER_TO_CHANNEL(identifier);

    edmaStart();

    DMAMUX->CHCFG[channel] = 0;
    DMAMUX->CHCFG[channel] = DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(dmaMuxId);
}

void dmaSetHandler(dmaIdentifier_e identifier, dmaCallbackHandlerFuncPtr callback, uint32_t priority,
                   uint32_t userParam)
{
    if (!edmaValidChannel(identifier)) {
        return;
    }

    const int index = DMA_IDENTIFIER_TO_INDEX(identifier);
    const unsigned channel = DMA_IDENTIFIER_TO_CHANNEL(identifier);

    edmaStart();

    dmaDescriptors[index].irqHandlerCallback = callback;
    dmaDescriptors[index].userParam = userParam;

    // Enable the channel's error interrupt.
    DMA0->SEEI = channel;

    // The shared error vector runs at the most urgent priority any owner asked for.
    if (edmaErrorPriority > priority) {
        edmaErrorPriority = priority;
        NVIC_SetPriority(DMA_ERROR_IRQn, NVIC_PRIORITY_LEVEL(priority));
    }
    NVIC_EnableIRQ(DMA_ERROR_IRQn);

    // The vector is shared with channel n+16, so keep the most urgent priority.
    const unsigned vector = channel & (EDMA_CHANNEL_VECTORS - 1);

    if (edmaChannelPriority[vector] > priority) {
        edmaChannelPriority[vector] = priority;
        NVIC_SetPriority(dmaDescriptors[index].irqN, NVIC_PRIORITY_LEVEL(priority));
    }
    NVIC_EnableIRQ(dmaDescriptors[index].irqN);
}

// Channels are handed out first come first served and never given back.
dmaIdentifier_e dmaGetFreeIdentifier(void)
{
    for (int index = 0; index < NXP_EDMA_CHANNEL_COUNT; index++) {
        if (dmaDescriptors[index].resourceOwner.owner == OWNER_FREE) {
            return DMA_CHANNEL_TO_IDENTIFIER(index);
        }
    }

    return DMA_NONE;
}

FAST_CODE bool dmaBuildTcd(dmaTcd_t *tcd, const dmaTransfer_t *transfer)
{
    if (!tcd || !transfer) {
        return false;
    }

    if (transfer->majorCount == 0 || transfer->majorCount > DMA_MAJOR_COUNT_MAX) {
        return false;
    }

    if (transfer->minorBytes == 0) {
        return false;
    }

    tcd->SADDR = (uint32_t)(uintptr_t)transfer->src;
    tcd->SOFF = transfer->srcOffset;
    tcd->ATTR = DMA_ATTR_SSIZE(transfer->srcSize) | DMA_ATTR_DSIZE(transfer->dstSize);
    tcd->NBYTES = transfer->minorBytes;
    tcd->SLAST = transfer->srcLastAdjust;
    tcd->DADDR = (uint32_t)(uintptr_t)transfer->dst;
    tcd->DOFF = transfer->dstOffset;
    tcd->CITER = transfer->majorCount;
    tcd->BITER = transfer->majorCount;
    tcd->DLAST_SGA = transfer->dstLastAdjust;
    tcd->CSR = 0;

    return true;
}

FAST_CODE bool dmaChainTcds(dmaTcd_t *list, unsigned count)
{
    if (!list || count == 0) {
        return false;
    }

    for (unsigned i = 0; i < count; i++) {
        if ((uintptr_t)&list[i] & (DMA_TCD_ALIGNMENT - 1)) {
            return false;
        }
    }

    for (unsigned i = 0; i + 1 < count; i++) {
        list[i].DLAST_SGA = (int32_t)(uintptr_t)&list[i + 1];
        list[i].CSR = (list[i].CSR & ~(DMA_CSR_DREQ_MASK | DMA_CSR_INTMAJOR_MASK)) | DMA_CSR_ESG_MASK;
    }

    list[count - 1].DLAST_SGA = 0;
    list[count - 1].CSR = (list[count - 1].CSR & ~DMA_CSR_ESG_MASK) | DMA_CSR_INTMAJOR_MASK | DMA_CSR_DREQ_MASK;

    return true;
}

// CSR is cleared first (clearing DONE) and written last, since ESG is ignored
// while DONE is set.
FAST_CODE void dmaLoadTcd(dmaIdentifier_e identifier, const dmaTcd_t *tcd)
{
    if (!edmaValidChannel(identifier) || !tcd) {
        return;
    }

    volatile dmaTcd_t *hw = edmaChannelTcd(DMA_IDENTIFIER_TO_CHANNEL(identifier));

    hw->CSR = 0;
    hw->SADDR = tcd->SADDR;
    hw->SOFF = tcd->SOFF;
    hw->ATTR = tcd->ATTR;
    hw->NBYTES = tcd->NBYTES;
    hw->SLAST = tcd->SLAST;
    hw->DADDR = tcd->DADDR;
    hw->DOFF = tcd->DOFF;
    hw->CITER = tcd->CITER;
    hw->DLAST_SGA = tcd->DLAST_SGA;
    hw->BITER = tcd->BITER;
    hw->CSR = tcd->CSR;
}

FAST_CODE bool dmaSetTransfer(dmaIdentifier_e identifier, const dmaTransfer_t *transfer, bool completeInterrupt)
{
    dmaTcd_t tcd;

    if (!dmaBuildTcd(&tcd, transfer)) {
        return false;
    }

    tcd.CSR = DMA_CSR_DREQ_MASK | (completeInterrupt ? DMA_CSR_INTMAJOR_MASK : 0);

    if (completeInterrupt) {
        edmaClaimVector(DMA_IDENTIFIER_TO_CHANNEL(identifier));
    }

    dmaLoadTcd(identifier, &tcd);

    return true;
}

FAST_CODE bool dmaSetTransferChain(dmaIdentifier_e identifier, dmaTcd_t *list, unsigned count)
{
    if (!dmaChainTcds(list, count)) {
        return false;
    }

    edmaClaimVector(DMA_IDENTIFIER_TO_CHANNEL(identifier));

    dmaLoadTcd(identifier, &list[0]);

    return true;
}

// The last descriptor keeps DREQ and gains ESG back to the head, so the channel
// stops holding the head and dmaEnableRequest() alone rearms it.
FAST_CODE bool dmaSetTransferRing(dmaIdentifier_e identifier, dmaTcd_t *list, unsigned count,
                                  bool completeInterrupt)
{
    if (!dmaChainTcds(list, count)) {
        return false;
    }

    list[count - 1].DLAST_SGA = (int32_t)(uintptr_t)&list[0];
    list[count - 1].CSR |= DMA_CSR_ESG_MASK;

    // Without a handler nothing would clear INT.
    if (!completeInterrupt) {
        list[count - 1].CSR &= ~DMA_CSR_INTMAJOR_MASK;
    } else {
        edmaClaimVector(DMA_IDENTIFIER_TO_CHANNEL(identifier));
    }

    dmaLoadTcd(identifier, &list[0]);

    return true;
}

FAST_CODE void dmaEnableRequest(dmaIdentifier_e identifier)
{
    if (edmaValidChannel(identifier)) {
        // Order the caller's descriptor and buffer stores before the channel start.
        __DMB();
        DMA0->SERQ = DMA_IDENTIFIER_TO_CHANNEL(identifier);
    }
}

FAST_CODE void dmaDisableRequest(dmaIdentifier_e identifier)
{
    if (edmaValidChannel(identifier)) {
        DMA0->CERQ = DMA_IDENTIFIER_TO_CHANNEL(identifier);
    }
}

// The channel must already be stopped.
FAST_CODE void dmaClearError(dmaIdentifier_e identifier)
{
    if (edmaValidChannel(identifier)) {
        DMA0->CERR = DMA_IDENTIFIER_TO_CHANNEL(identifier);
    }
}

FAST_CODE bool dmaChannelBusy(dmaIdentifier_e identifier)
{
    if (!edmaValidChannel(identifier)) {
        return false;
    }

    const unsigned channel = DMA_IDENTIFIER_TO_CHANNEL(identifier);

    // ACTIVE: moving data now. ERQ: still armed for the next request.
    return (edmaChannelTcd(channel)->CSR & DMA_CSR_ACTIVE_MASK) || (DMA0->ERQ & (1UL << channel));
}

// Channels n and n+16 share a vector; a vector with one owner skips the INT read.

static void edmaServeChannel(unsigned channel)
{
    // Cleared first: the callback may start the next transfer on this channel.
    DMA0->CINT = channel;

    dmaChannelDescriptor_t *descriptor = &dmaDescriptors[channel];
    if (descriptor->irqHandlerCallback) {
        descriptor->irqHandlerCallback(descriptor);
    }
}

#define DEFINE_DMA_IRQ_HANDLER(low, high)                                 \
    FAST_IRQ_HANDLER void DMA ## low ## _DMA ## high ## _IRQHandler(void) \
    {                                                                     \
        const unsigned owner = edmaVectorOwner[(low)];                    \
        if (owner < NXP_EDMA_CHANNEL_COUNT) {                           \
            edmaServeChannel(owner);                                      \
            return;                                                       \
        }                                                                 \
                                                                          \
        const uint32_t pending = DMA0->INT;                               \
                                                                          \
        if (pending & (1UL << (low))) {                                   \
            edmaServeChannel(low);                                        \
        }                                                                 \
        if (pending & (1UL << (high))) {                                  \
            edmaServeChannel(high);                                       \
        }                                                                 \
    }

DEFINE_DMA_IRQ_HANDLER(0, 16)
DEFINE_DMA_IRQ_HANDLER(1, 17)
DEFINE_DMA_IRQ_HANDLER(2, 18)
DEFINE_DMA_IRQ_HANDLER(3, 19)
DEFINE_DMA_IRQ_HANDLER(4, 20)
DEFINE_DMA_IRQ_HANDLER(5, 21)
DEFINE_DMA_IRQ_HANDLER(6, 22)
DEFINE_DMA_IRQ_HANDLER(7, 23)
DEFINE_DMA_IRQ_HANDLER(8, 24)
DEFINE_DMA_IRQ_HANDLER(9, 25)
DEFINE_DMA_IRQ_HANDLER(10, 26)
DEFINE_DMA_IRQ_HANDLER(11, 27)
DEFINE_DMA_IRQ_HANDLER(12, 28)
DEFINE_DMA_IRQ_HANDLER(13, 29)
DEFINE_DMA_IRQ_HANDLER(14, 30)
DEFINE_DMA_IRQ_HANDLER(15, 31)

// One vector for every channel error: stop the channel and report through the
// owner's callback with DMA_GET_FLAG_STATUS(d, DMA_IT_TEIF) set.
FAST_IRQ_HANDLER void DMA_ERROR_IRQHandler(void)
{
    dmaLastErrorStatus = DMA0->ES;

    uint32_t errors = DMA0->ERR;
    for (unsigned channel = 0; errors; channel++, errors >>= 1) {
        // Skip a fault an earlier callback has already cleared.
        if (!(errors & 1) || !(DMA0->ERR & (1UL << channel))) {
            continue;
        }

        volatile dmaTcd_t *hw = edmaChannelTcd(channel);

        DMA0->CERQ = channel;
        // Clearing CSR drops ESG and clears DONE.
        hw->CSR = 0;
        hw->DLAST_SGA = 0;
        DMA0->CINT = channel;

        dmaErrorChannel = 1UL << channel;

        dmaChannelDescriptor_t *descriptor = &dmaDescriptors[channel];
        if (descriptor->irqHandlerCallback) {
            descriptor->irqHandlerCallback(descriptor);
        }

        dmaErrorChannel = 0;
        DMA0->CERR = channel;
    }
}

int dmaGetHandlerCount(void)
{
    return NXP_EDMA_CHANNEL_COUNT;
}

int dmaGetDeviceNumber(dmaIdentifier_e identifier)
{
    UNUSED(identifier);
    return DMA_DEVICE_NO(identifier);
}

int dmaGetDeviceIndex(dmaIdentifier_e identifier)
{
    return DMA_IDENTIFIER_TO_CHANNEL(identifier);
}

const char *dmaGetDisplayString(void)
{
    return DMA_OUTPUT_STRING;
}

// Requests left in the major loop.
FAST_CODE uint32_t dmaGetDataLength(dmaResource_t *ref)
{
    if (!ref) {
        return 0;
    }

    return ((volatile dmaTcd_t *)ref)->CITER & DMA_CITER_ELINKNO_CITER_MASK;
}

#endif // USE_DMA
