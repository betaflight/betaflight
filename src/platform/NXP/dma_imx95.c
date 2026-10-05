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
 * eDMA2 (eDMA5 IP): request source set in CH_MUX, the M7 owns the channels from NXP_EDMA_CHANNEL_FIRST on.
 * eDMA3 (eDMA v4 IP): channel n serves request n, the M7 owns the channels in NXP_EDMA3_CHANNEL_MASK.
 * Engine wide settings are not the M7's.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "platform/dma.h"

// The first registers of the channel page, the same on both engines.
typedef struct {
    volatile uint32_t CH_CSR;
    volatile uint32_t CH_ES;
    volatile uint32_t CH_INT;
} edmaChannelPage_t;

// Bounds the wait for a channel to finish its current minor loop.
#define EDMA_STOP_POLLS 10000

static void edmaStopChannel(edmaChannelPage_t *page, volatile uint16_t *tcdCsr)
{
    page->CH_CSR = 0;
    for (unsigned polls = 0; (page->CH_CSR & DMA5_CH_CSR_ACTIVE_MASK) && polls < EDMA_STOP_POLLS; polls++) {
    }

    // No scatter gather reload, channel link or start once the request is gone.
    *tcdCsr = 0;

    page->CH_CSR = DMA5_CH_CSR_DONE_MASK;
    page->CH_INT = DMA5_CH_INT_INT_MASK;
    page->CH_ES = DMA5_CH_ES_ERR_MASK;
}

// Called from the reset handler before the TCMs are seeded: stays in flash, word sized stack accesses only.
void imx95DmaStopOwnedChannels(void)
{
    const unsigned end = NXP_EDMA_CHANNEL_FIRST + NXP_EDMA_CHANNEL_COUNT;

    for (unsigned channel = NXP_EDMA_CHANNEL_FIRST; channel < end; channel++) {
        edmaStopChannel((edmaChannelPage_t *)&EDMA5_2->TCD[channel].CH_CSR, &EDMA5_2->TCD[channel].CSR);
        EDMA5_2->CH_MUX[channel] = 0;
    }

    for (unsigned channel = 0; channel < NXP_EDMA3_CHANNEL_COUNT; channel++) {
        if ((NXP_EDMA3_CHANNEL_MASK >> channel) & 1) {
            edmaStopChannel((edmaChannelPage_t *)&DMA3->CH[channel].CH_CSR, &DMA3->CH[channel].TCD_CSR);
        }
    }
}

#ifdef USE_DMA

#include "common/utils.h"

#include "drivers/dma.h"
#include "drivers/dma_impl.h"
#include "drivers/resource.h"

STATIC_ASSERT(sizeof(dmaTcd_t) == 2 * DMA_TCD_ALIGNMENT, dmaTcd_t_must_hold_both_layouts);
STATIC_ASSERT(__alignof__(dmaTcd_t) == DMA_TCD_ALIGNMENT, dmaTcd_t_must_be_64_byte_aligned);
STATIC_ASSERT(offsetof(dmaTcd_t, BITER) == 0x2e, dmaTcd_t_must_match_the_channel_page);
STATIC_ASSERT(sizeof(edma3Tcd_t) == 32, edma3Tcd_t_must_match_the_hardware_TCD);
STATIC_ASSERT(offsetof(edma3Tcd_t, BITER) == 0x1e, edma3Tcd_t_must_match_the_channel_page);
STATIC_ASSERT(offsetof(dmaTcd_t, edma3) == DMA_TCD_ALIGNMENT, edma3_copy_must_be_32_byte_aligned);
STATIC_ASSERT(NXP_EDMA_CHANNEL_FIRST + NXP_EDMA_CHANNEL_COUNT <= DMA5_TCD_COUNT, channels_must_exist);
STATIC_ASSERT(NXP_EDMA3_CHANNEL_COUNT <= DMA_CH_COUNT, edma3_channels_must_exist);
STATIC_ASSERT((NXP_EDMA3_CHANNEL_MASK >> NXP_EDMA3_CHANNEL_COUNT) == 0, edma3_mask_must_name_real_channels);

// eDMA3 is driven with the eDMA5 bit masks.
STATIC_ASSERT(DMA_CH_CSR_ERQ_MASK == DMA5_CH_CSR_ERQ_MASK && DMA_CH_CSR_EEI_MASK == DMA5_CH_CSR_EEI_MASK
              && DMA_CH_CSR_DONE_MASK == DMA5_CH_CSR_DONE_MASK && DMA_CH_CSR_ACTIVE_MASK == DMA5_CH_CSR_ACTIVE_MASK
              && DMA_CH_ES_ERR_MASK == DMA5_CH_ES_ERR_MASK && DMA_CH_INT_INT_MASK == DMA5_CH_INT_INT_MASK
              && DMA_TCD_CSR_ESG_MASK == DMA5_CSR_ESG_MASK && DMA_TCD_CSR_DREQ_MASK == DMA5_CSR_DREQ_MASK
              && DMA_TCD_CSR_INTMAJOR_MASK == DMA5_CSR_INTMAJOR_MASK
              && DMA_TCD_CITER_ELINKNO_CITER_MASK == DMA5_CITER_ELINKNO_CITER_MASK,
              edma3_bits_must_match_edma5);

// n descriptors for the owned eDMA2 channels from index c on.
#define EDMA2_CHANNELS_1(c)     DEFINE_DMA_CHANNEL(c),
#define EDMA2_CHANNELS_2(c)     EDMA2_CHANNELS_1(c) EDMA2_CHANNELS_1((c) + 1)
#define EDMA2_CHANNELS_4(c)     EDMA2_CHANNELS_2(c) EDMA2_CHANNELS_2((c) + 2)
#define EDMA2_CHANNELS_8(c)     EDMA2_CHANNELS_4(c) EDMA2_CHANNELS_4((c) + 4)
#define EDMA2_CHANNELS_16(c)    EDMA2_CHANNELS_8(c) EDMA2_CHANNELS_8((c) + 8)
#define EDMA2_CHANNELS_32(c)    EDMA2_CHANNELS_16(c) EDMA2_CHANNELS_16((c) + 16)
#define EDMA2_CHANNELS_64(c)    EDMA2_CHANNELS_32(c) EDMA2_CHANNELS_32((c) + 32)

// Indexed by drivers/dma.c; the eDMA2 part is built from the binary digits of NXP_EDMA_CHANNEL_COUNT.
dmaChannelDescriptor_t dmaDescriptors[] = {
#if NXP_EDMA_CHANNEL_COUNT & 64
    EDMA2_CHANNELS_64(0)
#endif
#if NXP_EDMA_CHANNEL_COUNT & 32
    EDMA2_CHANNELS_32(NXP_EDMA_CHANNEL_COUNT & 64)
#endif
#if NXP_EDMA_CHANNEL_COUNT & 16
    EDMA2_CHANNELS_16(NXP_EDMA_CHANNEL_COUNT & 96)
#endif
#if NXP_EDMA_CHANNEL_COUNT & 8
    EDMA2_CHANNELS_8(NXP_EDMA_CHANNEL_COUNT & 112)
#endif
#if NXP_EDMA_CHANNEL_COUNT & 4
    EDMA2_CHANNELS_4(NXP_EDMA_CHANNEL_COUNT & 120)
#endif
#if NXP_EDMA_CHANNEL_COUNT & 2
    EDMA2_CHANNELS_2(NXP_EDMA_CHANNEL_COUNT & 124)
#endif
#if NXP_EDMA_CHANNEL_COUNT & 1
    EDMA2_CHANNELS_1(NXP_EDMA_CHANNEL_COUNT & 126)
#endif
    DEFINE_EDMA3_CHANNEL(0), DEFINE_EDMA3_CHANNEL(1), DEFINE_EDMA3_CHANNEL(2), DEFINE_EDMA3_CHANNEL(3),
    DEFINE_EDMA3_CHANNEL(4), DEFINE_EDMA3_CHANNEL(5), DEFINE_EDMA3_CHANNEL(6), DEFINE_EDMA3_CHANNEL(7),
    DEFINE_EDMA3_CHANNEL(8), DEFINE_EDMA3_CHANNEL(9), DEFINE_EDMA3_CHANNEL(10), DEFINE_EDMA3_CHANNEL(11),
    DEFINE_EDMA3_CHANNEL(12), DEFINE_EDMA3_CHANNEL(13), DEFINE_EDMA3_CHANNEL(14), DEFINE_EDMA3_CHANNEL(15),
    DEFINE_EDMA3_CHANNEL(16), DEFINE_EDMA3_CHANNEL(17), DEFINE_EDMA3_CHANNEL(18), DEFINE_EDMA3_CHANNEL(19),
    DEFINE_EDMA3_CHANNEL(20), DEFINE_EDMA3_CHANNEL(21), DEFINE_EDMA3_CHANNEL(22), DEFINE_EDMA3_CHANNEL(23),
    DEFINE_EDMA3_CHANNEL(24), DEFINE_EDMA3_CHANNEL(25), DEFINE_EDMA3_CHANNEL(26), DEFINE_EDMA3_CHANNEL(27),
    DEFINE_EDMA3_CHANNEL(28), DEFINE_EDMA3_CHANNEL(29), DEFINE_EDMA3_CHANNEL(30),
};

STATIC_ASSERT(ARRAYLEN(dmaDescriptors) == DMA_LAST_HANDLER, one_descriptor_per_channel);

uint32_t dmaLastErrorStatus;

// The faulting channel's identifier while its callback runs, read by DMA_GET_FLAG_STATUS().
uint32_t dmaErrorChannel;

// Most urgent priority asked for per engine (eDMA2, eDMA3); 0xff is least urgent.
static uint8_t edmaErrorPriority[2] = { 0xff, 0xff };

STATIC_ASSERT(DMA5_TCD_COUNT == 64, edmaPairPriority_initialiser_must_cover_every_pair);

// Per vector, the most urgent priority either channel of the pair asked for.
static uint8_t edmaPairPriority[DMA5_TCD_COUNT / 2] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
};

// The eDMA reaches DTCM through an alias 4 MB up and cannot reach ITCM at all.
#define EDMA_ITCM_END           0x00040000u
#define EDMA_DTCM_START         0x20000000u
#define EDMA_DTCM_END           0x20080000u
#define EDMA_DTCM_ALIAS_OFFSET  0x00400000u

#define EDMA_NO_ADDRESS         0xffffffffu

FAST_CODE static uint32_t edmaAddress(const void *pointer)
{
    const uint32_t address = (uint32_t)(uintptr_t)pointer;

    if (address < EDMA_ITCM_END) {
#ifdef DEBUG
        __BKPT(0);
#endif
        return EDMA_NO_ADDRESS;
    }

    if (address >= EDMA_DTCM_START && address < EDMA_DTCM_END) {
        return address + EDMA_DTCM_ALIAS_OFFSET;
    }

    return address;
}

static bool edmaIsEdma3(dmaIdentifier_e identifier)
{
    return DMA_IDENTIFIER_TO_CHANNEL(identifier) >= NXP_EDMA_CHANNEL_COUNT;
}

static unsigned edmaHwChannel(dmaIdentifier_e identifier)
{
    const int index = DMA_IDENTIFIER_TO_CHANNEL(identifier);

    return index < NXP_EDMA_CHANNEL_COUNT ? NXP_EDMA_CHANNEL_FIRST + index : index - NXP_EDMA_CHANNEL_COUNT;
}

static bool edmaValidChannel(dmaIdentifier_e identifier)
{
    if (identifier < DMA_FIRST_HANDLER || identifier > DMA_LAST_HANDLER) {
        return false;
    }

    return !edmaIsEdma3(identifier) || (NXP_EDMA3_CHANNEL_MASK >> edmaHwChannel(identifier)) & 1;
}

static edmaChannelPage_t *edmaPage(dmaIdentifier_e identifier)
{
    const unsigned channel = edmaHwChannel(identifier);

    if (edmaIsEdma3(identifier)) {
        return (edmaChannelPage_t *)&DMA3->CH[channel].CH_CSR;
    }

    return (edmaChannelPage_t *)&EDMA5_2->TCD[channel].CH_CSR;
}

static volatile dmaTcd_t *edmaChannelTcd(unsigned hwChannel)
{
    return (volatile dmaTcd_t *)&EDMA5_2->TCD[hwChannel].SADDR;
}

static volatile edma3Tcd_t *edma3ChannelTcd(unsigned hwChannel)
{
    return (volatile edma3Tcd_t *)&DMA3->CH[hwChannel].TCD_SADDR;
}

// DONE is write one to clear, so writes that must keep it mask it out.
#define EDMA_CH_CSR_KEEP(csr)   ((csr) & ~DMA5_CH_CSR_DONE_MASK)

void dmaEnable(dmaIdentifier_e identifier)
{
    if (!edmaValidChannel(identifier)) {
        return;
    }

    edmaChannelPage_t *page = edmaPage(identifier);

    page->CH_CSR = (page->CH_CSR & ~DMA5_CH_CSR_ERQ_MASK) | DMA5_CH_CSR_DONE_MASK;
    page->CH_INT = DMA5_CH_INT_INT_MASK;
    page->CH_ES = DMA5_CH_ES_ERR_MASK;

    if (!edmaIsEdma3(identifier)) {
        EDMA5_2->CH_MUX[edmaHwChannel(identifier)] = 0;
    }
}

// dmaMuxId is an NXP_DMA_REQ_* id. eDMA3 channels have no mux. A source may
// only be given to one eDMA2 channel, so the old one is cleared first.
void dmaMuxEnable(dmaIdentifier_e identifier, uint32_t dmaMuxId)
{
    if (!edmaValidChannel(identifier) || edmaIsEdma3(identifier) || NXP_DMA_REQ_EDMA(dmaMuxId) != 2) {
        return;
    }

    const unsigned channel = edmaHwChannel(identifier);

    EDMA5_2->CH_MUX[channel] = 0;
    EDMA5_2->CH_MUX[channel] = DMA5_CH_MUX_SRC(NXP_DMA_REQ_SOURCE(dmaMuxId));
}

void dmaSetHandler(dmaIdentifier_e identifier, dmaCallbackHandlerFuncPtr callback, uint32_t priority,
                   uint32_t userParam)
{
    if (!edmaValidChannel(identifier)) {
        return;
    }

    const int index = DMA_IDENTIFIER_TO_INDEX(identifier);

    dmaDescriptors[index].irqHandlerCallback = callback;
    dmaDescriptors[index].userParam = userParam;

    edmaChannelPage_t *page = edmaPage(identifier);

    page->CH_CSR = EDMA_CH_CSR_KEEP(page->CH_CSR) | DMA5_CH_CSR_EEI_MASK;

    const bool edma3 = edmaIsEdma3(identifier);
    const IRQn_Type errorIrq = edma3 ? DMA3_ERROR_IRQn : DMA5_2_ERROR_IRQn;

    if (edmaErrorPriority[edma3] > priority) {
        edmaErrorPriority[edma3] = priority;
        NVIC_SetPriority(errorIrq, NVIC_PRIORITY_LEVEL(priority));
    }
    NVIC_EnableIRQ(errorIrq);

    if (edma3) {
        NVIC_SetPriority(dmaDescriptors[index].irqN, NVIC_PRIORITY_LEVEL(priority));
    } else {
        const unsigned pair = edmaHwChannel(identifier) >> 1;

        if (edmaPairPriority[pair] > priority) {
            edmaPairPriority[pair] = priority;
            NVIC_SetPriority(dmaDescriptors[index].irqN, NVIC_PRIORITY_LEVEL(priority));
        }
    }
    NVIC_EnableIRQ(dmaDescriptors[index].irqN);
}

// The System Manager may give eDMA2 a domain id the DTCM alias refuses, so one copy is tried first.
static DMA_DATA_ZERO_INIT uint32_t edma2TestBuffer[2][8];
static bool edma2Tested;
static bool edma2Works;

static bool edma2CopyWorks(void)
{
    const dmaIdentifier_e identifier = DMA_CHANNEL_TO_IDENTIFIER(0);
    edmaChannelPage_t *page = edmaPage(identifier);
    const dmaTransfer_t transfer = {
        .src = edma2TestBuffer[0], .dst = edma2TestBuffer[1], .srcOffset = 4, .dstOffset = 4,
        .srcSize = DMA_XFER_SIZE_4B, .dstSize = DMA_XFER_SIZE_4B,
        .minorBytes = sizeof(edma2TestBuffer[0]), .majorCount = 1,
    };
    dmaTcd_t tcd;

    for (unsigned i = 0; i < ARRAYLEN(edma2TestBuffer[0]); i++) {
        edma2TestBuffer[0][i] = 0xa5c30000u + i;
        edma2TestBuffer[1][i] = 0;
    }

    if (!dmaBuildTcd(&tcd, &transfer)) {
        return false;
    }
    dmaLoadTcd(identifier, &tcd);
    __DMB();
    EDMA5_2->TCD[NXP_EDMA_CHANNEL_FIRST].CSR = DMA5_CSR_START_MASK;

    for (unsigned polls = 0; !(page->CH_CSR & DMA5_CH_CSR_DONE_MASK) && !(page->CH_ES & DMA5_CH_ES_ERR_MASK)
         && polls < EDMA_STOP_POLLS; polls++) {
    }
    __DMB();

    const bool works = (page->CH_CSR & DMA5_CH_CSR_DONE_MASK) && !(page->CH_ES & DMA5_CH_ES_ERR_MASK)
                       && memcmp(edma2TestBuffer[0], edma2TestBuffer[1], sizeof(edma2TestBuffer[0])) == 0;

    edmaStopChannel(page, &EDMA5_2->TCD[NXP_EDMA_CHANNEL_FIRST].CSR);

    return works;
}

// eDMA2 channels are handed out first come first served and never given back.
dmaIdentifier_e dmaGetFreeIdentifier(void)
{
    if (NXP_EDMA_CHANNEL_COUNT > 0 && !edma2Tested) {
        edma2Tested = true;
        edma2Works = edma2CopyWorks();
    }

    if (!edma2Works) {
        return DMA_NONE;
    }

    for (int index = 0; index < NXP_EDMA_CHANNEL_COUNT; index++) {
        if (dmaDescriptors[index].resourceOwner.owner == OWNER_FREE) {
            return DMA_CHANNEL_TO_IDENTIFIER(index);
        }
    }

    return DMA_NONE;
}

dmaIdentifier_e dmaGetFreeIdentifierForRequest(uint32_t request)
{
    if (NXP_DMA_REQ_EDMA(request) == 2) {
        return dmaGetFreeIdentifier();
    }

    if (NXP_DMA_REQ_EDMA(request) != 3) {
        return DMA_NONE;
    }

    const dmaIdentifier_e identifier = DMA_CHANNEL_TO_IDENTIFIER(NXP_EDMA_CHANNEL_COUNT + NXP_DMA_REQ_SOURCE(request));

    if (!edmaValidChannel(identifier)
        || dmaDescriptors[DMA_IDENTIFIER_TO_INDEX(identifier)].resourceOwner.owner != OWNER_FREE) {
        return DMA_NONE;
    }

    return identifier;
}

// The high words sign extend the 32 bit adjustments to 64 bits.
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

    const uint32_t src = edmaAddress(transfer->src);
    const uint32_t dst = edmaAddress(transfer->dst);

    if (src == EDMA_NO_ADDRESS || dst == EDMA_NO_ADDRESS) {
        return false;
    }

    tcd->SADDR = src;
    tcd->SADDR_HIGH = 0;
    tcd->SOFF = transfer->srcOffset;
    tcd->ATTR = DMA5_ATTR_SSIZE(transfer->srcSize) | DMA5_ATTR_DSIZE(transfer->dstSize);
    tcd->NBYTES = transfer->minorBytes;
    tcd->SLAST = transfer->srcLastAdjust;
    tcd->SLAST_HIGH = transfer->srcLastAdjust < 0 ? 0xffffffffu : 0;
    tcd->DADDR = dst;
    tcd->DADDR_HIGH = 0;
    tcd->DOFF = transfer->dstOffset;
    tcd->CITER = transfer->majorCount;
    tcd->BITER = transfer->majorCount;
    tcd->DLAST_SGA = transfer->dstLastAdjust;
    tcd->DLAST_SGA_HIGH = transfer->dstLastAdjust < 0 ? 0xffffffffu : 0;
    tcd->CSR = 0;

    return true;
}

FAST_CODE bool dmaChainTcds(dmaTcd_t *list, unsigned count)
{
    if (!list || count == 0) {
        return false;
    }

    for (unsigned i = 0; i < count; i++) {
        if (((uintptr_t)&list[i] & (DMA_TCD_ALIGNMENT - 1)) || edmaAddress(&list[i]) == EDMA_NO_ADDRESS) {
            return false;
        }
    }

    for (unsigned i = 0; i + 1 < count; i++) {
        list[i].DLAST_SGA = (int32_t)edmaAddress(&list[i + 1]);
        list[i].DLAST_SGA_HIGH = 0;
        list[i].CSR = (list[i].CSR & ~(DMA5_CSR_DREQ_MASK | DMA5_CSR_INTMAJOR_MASK)) | DMA5_CSR_ESG_MASK;
    }

    list[count - 1].DLAST_SGA = 0;
    list[count - 1].DLAST_SGA_HIGH = 0;
    list[count - 1].CSR = (list[count - 1].CSR & ~DMA5_CSR_ESG_MASK) | DMA5_CSR_INTMAJOR_MASK | DMA5_CSR_DREQ_MASK;

    return true;
}

// eDMA3 reads 32 byte descriptors, so a link points at the copy inside the next entry.
FAST_CODE static void edma3PackTcd(volatile edma3Tcd_t *dst, const dmaTcd_t *src)
{
    dst->CSR = 0;
    dst->SADDR = src->SADDR;
    dst->SOFF = src->SOFF;
    dst->ATTR = src->ATTR;
    dst->NBYTES = src->NBYTES;
    dst->SLAST = src->SLAST;
    dst->DADDR = src->DADDR;
    dst->DOFF = src->DOFF;
    dst->CITER = src->CITER;
    dst->DLAST_SGA = (src->CSR & DMA5_CSR_ESG_MASK) ? src->DLAST_SGA + (int32_t)offsetof(dmaTcd_t, edma3)
                                                    : src->DLAST_SGA;
    dst->BITER = src->BITER;
    dst->CSR = src->CSR;
}

FAST_CODE static void edmaPackList(dmaIdentifier_e identifier, dmaTcd_t *list, unsigned count)
{
    if (edmaIsEdma3(identifier)) {
        for (unsigned i = 0; i < count; i++) {
            edma3PackTcd(&list[i].edma3, &list[i]);
        }
    }
}

// DONE is cleared first and CSR written last, since ESG is ignored while DONE is set.
FAST_CODE void dmaLoadTcd(dmaIdentifier_e identifier, const dmaTcd_t *tcd)
{
    if (!edmaValidChannel(identifier) || !tcd) {
        return;
    }

    const unsigned channel = edmaHwChannel(identifier);

    edmaPage(identifier)->CH_CSR |= DMA5_CH_CSR_DONE_MASK;

    if (edmaIsEdma3(identifier)) {
        edma3PackTcd(edma3ChannelTcd(channel), tcd);
        return;
    }

    volatile dmaTcd_t *hw = edmaChannelTcd(channel);

    hw->CSR = 0;
    hw->SADDR = tcd->SADDR;
    hw->SADDR_HIGH = tcd->SADDR_HIGH;
    hw->SOFF = tcd->SOFF;
    hw->ATTR = tcd->ATTR;
    hw->NBYTES = tcd->NBYTES;
    hw->SLAST = tcd->SLAST;
    hw->SLAST_HIGH = tcd->SLAST_HIGH;
    hw->DADDR = tcd->DADDR;
    hw->DADDR_HIGH = tcd->DADDR_HIGH;
    hw->DOFF = tcd->DOFF;
    hw->CITER = tcd->CITER;
    hw->DLAST_SGA = tcd->DLAST_SGA;
    hw->DLAST_SGA_HIGH = tcd->DLAST_SGA_HIGH;
    hw->BITER = tcd->BITER;
    hw->CSR = tcd->CSR;
}

FAST_CODE bool dmaSetTransfer(dmaIdentifier_e identifier, const dmaTransfer_t *transfer, bool completeInterrupt)
{
    dmaTcd_t tcd;

    if (!dmaBuildTcd(&tcd, transfer)) {
        return false;
    }

    tcd.CSR = DMA5_CSR_DREQ_MASK | (completeInterrupt ? DMA5_CSR_INTMAJOR_MASK : 0);

    dmaLoadTcd(identifier, &tcd);

    return true;
}

FAST_CODE bool dmaSetTransferChain(dmaIdentifier_e identifier, dmaTcd_t *list, unsigned count)
{
    if (!dmaChainTcds(list, count)) {
        return false;
    }

    edmaPackList(identifier, list, count);
    dmaLoadTcd(identifier, &list[0]);

    return true;
}

// The last descriptor keeps DREQ and links back to the head, so dmaEnableRequest() alone rearms the ring.
FAST_CODE bool dmaSetTransferRing(dmaIdentifier_e identifier, dmaTcd_t *list, unsigned count,
                                  bool completeInterrupt)
{
    if (!dmaChainTcds(list, count)) {
        return false;
    }

    list[count - 1].DLAST_SGA = (int32_t)edmaAddress(&list[0]);
    list[count - 1].CSR |= DMA5_CSR_ESG_MASK;

    // Without a handler nothing would clear INT.
    if (!completeInterrupt) {
        list[count - 1].CSR &= ~DMA5_CSR_INTMAJOR_MASK;
    }

    edmaPackList(identifier, list, count);
    dmaLoadTcd(identifier, &list[0]);

    return true;
}

FAST_CODE void dmaEnableRequest(dmaIdentifier_e identifier)
{
    if (edmaValidChannel(identifier)) {
        // Order the caller's descriptor and buffer stores before the channel start.
        __DMB();
        edmaChannelPage_t *page = edmaPage(identifier);
        page->CH_CSR = EDMA_CH_CSR_KEEP(page->CH_CSR) | DMA5_CH_CSR_ERQ_MASK;
    }
}

FAST_CODE void dmaDisableRequest(dmaIdentifier_e identifier)
{
    if (edmaValidChannel(identifier)) {
        edmaChannelPage_t *page = edmaPage(identifier);
        page->CH_CSR = EDMA_CH_CSR_KEEP(page->CH_CSR) & ~DMA5_CH_CSR_ERQ_MASK;
    }
}

// The channel must already be stopped.
FAST_CODE void dmaClearError(dmaIdentifier_e identifier)
{
    if (edmaValidChannel(identifier)) {
        edmaPage(identifier)->CH_ES = DMA5_CH_ES_ERR_MASK;
    }
}

FAST_CODE bool dmaChannelBusy(dmaIdentifier_e identifier)
{
    if (!edmaValidChannel(identifier)) {
        return false;
    }

    // ACTIVE: moving data now. ERQ: still armed for the next request.
    return edmaPage(identifier)->CH_CSR & (DMA5_CH_CSR_ACTIVE_MASK | DMA5_CH_CSR_ERQ_MASK);
}

FAST_CODE static void edmaServeChannel(dmaIdentifier_e identifier)
{
    edmaChannelPage_t *page = edmaPage(identifier);

    if (!(page->CH_INT & DMA5_CH_INT_INT_MASK)) {
        return;
    }

    // Cleared first: the callback may start the next transfer on this channel.
    page->CH_INT = DMA5_CH_INT_INT_MASK;

    dmaChannelDescriptor_t *descriptor = &dmaDescriptors[DMA_IDENTIFIER_TO_INDEX(identifier)];
    if (descriptor->irqHandlerCallback) {
        descriptor->irqHandlerCallback(descriptor);
    }
}

// Only owned channels are looked at; the other channel of a pair may be the A55's.
FAST_CODE static void edmaServePair(unsigned low)
{
    for (unsigned channel = low; channel <= low + 1; channel++) {
        const int index = (int)channel - NXP_EDMA_CHANNEL_FIRST;

        if (index >= 0 && index < NXP_EDMA_CHANNEL_COUNT) {
            edmaServeChannel(DMA_CHANNEL_TO_IDENTIFIER(index));
        }
    }
}

FAST_CODE static void edma3Serve(unsigned channel)
{
    if ((NXP_EDMA3_CHANNEL_MASK >> channel) & 1) {
        edmaServeChannel(DMA_CHANNEL_TO_IDENTIFIER(NXP_EDMA_CHANNEL_COUNT + channel));
    }
}

#define DEFINE_DMA_IRQ_HANDLER(low, high) \
    FAST_IRQ_HANDLER void DMA5_2_ ## low ## _ ## high ## _IRQHandler(void) { edmaServePair(low); }

DEFINE_DMA_IRQ_HANDLER(0, 1)
DEFINE_DMA_IRQ_HANDLER(2, 3)
DEFINE_DMA_IRQ_HANDLER(4, 5)
DEFINE_DMA_IRQ_HANDLER(6, 7)
DEFINE_DMA_IRQ_HANDLER(8, 9)
DEFINE_DMA_IRQ_HANDLER(10, 11)
DEFINE_DMA_IRQ_HANDLER(12, 13)
DEFINE_DMA_IRQ_HANDLER(14, 15)
DEFINE_DMA_IRQ_HANDLER(16, 17)
DEFINE_DMA_IRQ_HANDLER(18, 19)
DEFINE_DMA_IRQ_HANDLER(20, 21)
DEFINE_DMA_IRQ_HANDLER(22, 23)
DEFINE_DMA_IRQ_HANDLER(24, 25)
DEFINE_DMA_IRQ_HANDLER(26, 27)
DEFINE_DMA_IRQ_HANDLER(28, 29)
DEFINE_DMA_IRQ_HANDLER(30, 31)
DEFINE_DMA_IRQ_HANDLER(32, 33)
DEFINE_DMA_IRQ_HANDLER(34, 35)
DEFINE_DMA_IRQ_HANDLER(36, 37)
DEFINE_DMA_IRQ_HANDLER(38, 39)
DEFINE_DMA_IRQ_HANDLER(40, 41)
DEFINE_DMA_IRQ_HANDLER(42, 43)
DEFINE_DMA_IRQ_HANDLER(44, 45)
DEFINE_DMA_IRQ_HANDLER(46, 47)
DEFINE_DMA_IRQ_HANDLER(48, 49)
DEFINE_DMA_IRQ_HANDLER(50, 51)
DEFINE_DMA_IRQ_HANDLER(52, 53)
DEFINE_DMA_IRQ_HANDLER(54, 55)
DEFINE_DMA_IRQ_HANDLER(56, 57)
DEFINE_DMA_IRQ_HANDLER(58, 59)
DEFINE_DMA_IRQ_HANDLER(60, 61)
DEFINE_DMA_IRQ_HANDLER(62, 63)

#define DEFINE_EDMA3_IRQ_HANDLER(n) \
    FAST_IRQ_HANDLER void DMA3_ ## n ## _IRQHandler(void) { edma3Serve(n); }

DEFINE_EDMA3_IRQ_HANDLER(0)
DEFINE_EDMA3_IRQ_HANDLER(1)
DEFINE_EDMA3_IRQ_HANDLER(2)
DEFINE_EDMA3_IRQ_HANDLER(3)
DEFINE_EDMA3_IRQ_HANDLER(4)
DEFINE_EDMA3_IRQ_HANDLER(5)
DEFINE_EDMA3_IRQ_HANDLER(6)
DEFINE_EDMA3_IRQ_HANDLER(7)
DEFINE_EDMA3_IRQ_HANDLER(8)
DEFINE_EDMA3_IRQ_HANDLER(9)
DEFINE_EDMA3_IRQ_HANDLER(10)
DEFINE_EDMA3_IRQ_HANDLER(11)
DEFINE_EDMA3_IRQ_HANDLER(12)
DEFINE_EDMA3_IRQ_HANDLER(13)
DEFINE_EDMA3_IRQ_HANDLER(14)
DEFINE_EDMA3_IRQ_HANDLER(15)
DEFINE_EDMA3_IRQ_HANDLER(16)
DEFINE_EDMA3_IRQ_HANDLER(17)
DEFINE_EDMA3_IRQ_HANDLER(18)
DEFINE_EDMA3_IRQ_HANDLER(19)
DEFINE_EDMA3_IRQ_HANDLER(20)
DEFINE_EDMA3_IRQ_HANDLER(21)
DEFINE_EDMA3_IRQ_HANDLER(22)
DEFINE_EDMA3_IRQ_HANDLER(23)
DEFINE_EDMA3_IRQ_HANDLER(24)
DEFINE_EDMA3_IRQ_HANDLER(25)
DEFINE_EDMA3_IRQ_HANDLER(26)
DEFINE_EDMA3_IRQ_HANDLER(27)
DEFINE_EDMA3_IRQ_HANDLER(28)
DEFINE_EDMA3_IRQ_HANDLER(29)
DEFINE_EDMA3_IRQ_HANDLER(30)

// Stops each owned faulting channel and calls its callback with DMA_GET_FLAG_STATUS(d, DMA_IT_TEIF) set.
FAST_CODE static void edmaServeErrors(unsigned firstIndex, unsigned endIndex)
{
    for (unsigned index = firstIndex; index < endIndex; index++) {
        const dmaIdentifier_e identifier = DMA_CHANNEL_TO_IDENTIFIER(index);

        if (!edmaValidChannel(identifier)) {
            continue;
        }

        edmaChannelPage_t *page = edmaPage(identifier);
        const uint32_t status = page->CH_ES;

        // Also skips a fault a callback has already cleared.
        if (!(status & DMA5_CH_ES_ERR_MASK)) {
            continue;
        }

        dmaLastErrorStatus = status;

        page->CH_CSR = (page->CH_CSR & ~DMA5_CH_CSR_ERQ_MASK) | DMA5_CH_CSR_DONE_MASK;

        const unsigned channel = edmaHwChannel(identifier);

        if (edmaIsEdma3(identifier)) {
            edma3ChannelTcd(channel)->CSR = 0;
            edma3ChannelTcd(channel)->DLAST_SGA = 0;
        } else {
            volatile dmaTcd_t *hw = edmaChannelTcd(channel);

            hw->CSR = 0;
            hw->DLAST_SGA = 0;
            hw->DLAST_SGA_HIGH = 0;
        }
        page->CH_INT = DMA5_CH_INT_INT_MASK;

        dmaChannelDescriptor_t *descriptor = &dmaDescriptors[index];

        dmaErrorChannel = descriptor->completeFlag;

        if (descriptor->irqHandlerCallback) {
            descriptor->irqHandlerCallback(descriptor);
        }

        dmaErrorChannel = 0;
        page->CH_ES = DMA5_CH_ES_ERR_MASK;
    }
}

FAST_IRQ_HANDLER void DMA5_2_ERROR_IRQHandler(void)
{
    edmaServeErrors(0, NXP_EDMA_CHANNEL_COUNT);
}

FAST_IRQ_HANDLER void DMA3_ERROR_IRQHandler(void)
{
    edmaServeErrors(NXP_EDMA_CHANNEL_COUNT, ARRAYLEN(dmaDescriptors));
}

int dmaGetHandlerCount(void)
{
    return ARRAYLEN(dmaDescriptors);
}

int dmaGetDeviceNumber(dmaIdentifier_e identifier)
{
    return edmaIsEdma3(identifier) ? 3 : 2;
}

int dmaGetDeviceIndex(dmaIdentifier_e identifier)
{
    return edmaHwChannel(identifier);
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

    if ((uintptr_t)ref >= (uintptr_t)DMA3 && (uintptr_t)ref < (uintptr_t)(DMA3 + 1)) {
        return ((volatile edma3Tcd_t *)ref)->CITER & DMA5_CITER_ELINKNO_CITER_MASK;
    }

    return ((volatile dmaTcd_t *)ref)->CITER & DMA5_CITER_ELINKNO_CITER_MASK;
}

#endif // USE_DMA
