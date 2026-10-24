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

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "drivers/dma.h"

#include "platform/nxp_family.h"

#if NXP_HAS_EDMA5

#include "platform/dma_request_imx95.h"

// The eDMA2 channels the M7 owns, from nxp_family.h or the board config.
#define NXP_EDMA_CHANNEL_FIRST      NXP_EDMA2_CHANNEL_FIRST
#define NXP_EDMA_CHANNEL_COUNT      NXP_EDMA2_CHANNEL_COUNT

#define NXP_EDMA3_CHANNEL_COUNT     31

// Identifier = DMA_FIRST_HANDLER + index. Indexes below NXP_EDMA_CHANNEL_COUNT
// are eDMA2 channel NXP_EDMA_CHANNEL_FIRST + index, where channels 2k and 2k+1
// share one vector; the rest are eDMA3 channel index - NXP_EDMA_CHANNEL_COUNT,
// one vector each.
#define DMA_LAST_HANDLER     (DMA_FIRST_HANDLER + NXP_EDMA_CHANNEL_COUNT + NXP_EDMA3_CHANNEL_COUNT - 1)

#else

// Identifier = DMA_FIRST_HANDLER + eDMA channel. Channels n and n+16 share one
// vector, so handlers must test INT to find the channel.
#define DMA_LAST_HANDLER     (DMA_FIRST_HANDLER + NXP_EDMA_CHANNEL_COUNT - 1)

#endif

#define DMA_CHANNEL_TO_IDENTIFIER(channel) ((dmaIdentifier_e)((channel) + DMA_FIRST_HANDLER))
#define DMA_IDENTIFIER_TO_CHANNEL(identifier) ((identifier) - DMA_FIRST_HANDLER)

#define DMA_DEVICE_NO(x)     (1)
#define DMA_OUTPUT_STRING    "DMA%d Channel %d:"

#if NXP_HAS_EDMA5

// ref is the TCD inside the channel page; completeFlag is the identifier.
#define DEFINE_DMA_CHANNEL(c) { \
    .dma = EDMA5_2, \
    .ref = (dmaResource_t *)&EDMA5_2->TCD[NXP_EDMA_CHANNEL_FIRST + (c)].SADDR, \
    .stream = 0, \
    .channel = (c), \
    .irqHandlerCallback = NULL, \
    .flagsShift = 0, \
    .irqN = DMA5_2_0_1_IRQn + ((NXP_EDMA_CHANNEL_FIRST + (c)) >> 1), \
    .userParam = 0, \
    .resourceOwner.owner = 0, \
    .resourceOwner.index = 0, \
    .completeFlag = DMA_CHANNEL_TO_IDENTIFIER(c), \
    .dmamux = NULL \
    }

// c is the eDMA3 channel.
#define DEFINE_EDMA3_CHANNEL(c) { \
    .dma = DMA3, \
    .ref = (dmaResource_t *)&DMA3->CH[c].TCD_SADDR, \
    .stream = 0, \
    .channel = NXP_EDMA_CHANNEL_COUNT + (c), \
    .irqHandlerCallback = NULL, \
    .flagsShift = 0, \
    .irqN = DMA3_0_IRQn + (c), \
    .userParam = 0, \
    .resourceOwner.owner = 0, \
    .resourceOwner.index = 0, \
    .completeFlag = DMA_CHANNEL_TO_IDENTIFIER(NXP_EDMA_CHANNEL_COUNT + (c)), \
    .dmamux = NULL \
    }

#else

// completeFlag is the channel's own bit in the eDMA INT and ERR registers.
#define DEFINE_DMA_CHANNEL(c) { \
    .dma = DMA0, \
    .ref = (dmaResource_t *)&DMA0->TCD[c], \
    .stream = 0, \
    .channel = (c), \
    .irqHandlerCallback = NULL, \
    .flagsShift = 0, \
    .irqN = DMA0_DMA16_IRQn + ((c) & 0x0f), \
    .userParam = 0, \
    .resourceOwner.owner = 0, \
    .resourceOwner.index = 0, \
    .completeFlag = (1UL << (c)), \
    .dmamux = DMAMUX \
    }

#endif

// The only flag: set while the error vector runs a callback for a channel error.
#define DMA_IT_TEIF          (1U << 0)

extern uint32_t dmaErrorChannel;

#if NXP_HAS_EDMA5

#define DMA_GET_FLAG_STATUS(d, flag) \
    (((flag) & DMA_IT_TEIF) ? (dmaErrorChannel == (d)->completeFlag) : 0U)

#else

#define DMA_GET_FLAG_STATUS(d, flag) \
    (((flag) & DMA_IT_TEIF) ? (dmaErrorChannel & (d)->completeFlag) : 0U)

#endif

dmaIdentifier_e dmaGetFreeIdentifier(void);

#if NXP_HAS_EDMA5

// An eDMA2 request gets any free owned channel, an eDMA3 request only the
// channel of its own number, and only if NXP_EDMA3_CHANNEL_MASK allows it.
dmaIdentifier_e dmaGetFreeIdentifierForRequest(uint32_t request);

// Stops every owned channel on both engines and drops their requests and flags.
void imx95DmaStopOwnedChannels(void);

#else

static inline dmaIdentifier_e dmaGetFreeIdentifierForRequest(uint32_t request)
{
    (void)request;
    return dmaGetFreeIdentifier();
}

#endif

#if NXP_HAS_EDMA5

// eDMA3 (eDMA v4) Transfer Control Descriptor, the layout of both its channel
// registers and a 32 byte descriptor in memory.
typedef struct edma3Tcd_s {
    uint32_t SADDR;
    int16_t  SOFF;
    uint16_t ATTR;
    uint32_t NBYTES;
    int32_t  SLAST;
    uint32_t DADDR;
    int16_t  DOFF;
    uint16_t CITER;
    int32_t  DLAST_SGA;
    uint16_t CSR;
    uint16_t BITER;
} __attribute__((aligned(32))) edma3Tcd_t;

// eDMA5 Transfer Control Descriptor with 64 bit addresses, the layout of both the
// channel registers and a 64 byte descriptor in memory, followed by the same
// descriptor as eDMA3 reads it. CITER and BITER are 15 bit.
typedef struct dmaTcd_s {
    uint32_t SADDR;
    uint32_t SADDR_HIGH;
    int16_t  SOFF;
    uint16_t ATTR;
    uint32_t NBYTES;
    int32_t  SLAST;
    uint32_t SLAST_HIGH;
    uint32_t DADDR;
    uint32_t DADDR_HIGH;
    int32_t  DLAST_SGA;
    uint32_t DLAST_SGA_HIGH;
    int16_t  DOFF;
    uint16_t CITER;
    uint16_t CSR;
    uint16_t BITER;
    uint8_t  RESERVED[16];
    edma3Tcd_t edma3;           // filled from the fields above for eDMA3 lists
} __attribute__((aligned(64))) dmaTcd_t;

#define DMA_TCD_ALIGNMENT       64

#else

// eDMA Transfer Control Descriptor, the layout of both the channel registers
// and a descriptor in memory. CITER and BITER are 15 bit (the top bit is ELINK).
typedef struct dmaTcd_s {
    uint32_t SADDR;
    int16_t  SOFF;
    uint16_t ATTR;
    uint32_t NBYTES;
    int32_t  SLAST;
    uint32_t DADDR;
    int16_t  DOFF;
    uint16_t CITER;
    int32_t  DLAST_SGA;
    uint16_t CSR;
    uint16_t BITER;
} __attribute__((aligned(32))) dmaTcd_t;

#define DMA_TCD_ALIGNMENT       32

#endif

#define DMA_MAJOR_COUNT_MAX     0x7fff

// Descriptor lists must be DMA_TCD_ALIGNMENT aligned or the engine refuses the fetch.
#define DMA_TCD_LIST(name, count) DMA_DATA_ZERO_INIT dmaTcd_t name[count]

// TCD ATTR SSIZE and DSIZE values; the address must be aligned to the size.
typedef enum {
    DMA_XFER_SIZE_1B  = 0,
    DMA_XFER_SIZE_2B  = 1,
    DMA_XFER_SIZE_4B  = 2,
    DMA_XFER_SIZE_8B  = 3,
    DMA_XFER_SIZE_32B = 5,      // 32 byte burst, four 64 bit beats
} dmaTransferSize_e;

typedef struct dmaTransfer_s {
    const void *src;
    void *dst;
    int16_t srcOffset;          // bytes added to the source after each read
    int16_t dstOffset;          // bytes added to the destination after each write
    uint8_t srcSize;            // dmaTransferSize_e
    uint8_t dstSize;            // dmaTransferSize_e
    // Bytes per peripheral request; the engine switches channels only between these.
    uint32_t minorBytes;
    uint16_t majorCount;        // number of requests, at most DMA_MAJOR_COUNT_MAX
    int32_t srcLastAdjust;      // added to the source when the major loop ends
    int32_t dstLastAdjust;      // added to the destination when the major loop
                                // ends; holds the next descriptor when chained
} dmaTransfer_t;

// Fill in one descriptor. Sets no CSR flags.
bool dmaBuildTcd(dmaTcd_t *tcd, const dmaTransfer_t *transfer);

// Link descriptors into a scatter gather chain; only the last sets INTMAJOR and
// DREQ. False if the list is empty or an entry is not 32 byte aligned.
bool dmaChainTcds(dmaTcd_t *list, unsigned count);

void dmaLoadTcd(dmaIdentifier_e identifier, const dmaTcd_t *tcd);

bool dmaSetTransfer(dmaIdentifier_e identifier, const dmaTransfer_t *transfer, bool completeInterrupt);

// Chain the list and load its head; the list must stay in place while in use.
bool dmaSetTransferChain(dmaIdentifier_e identifier, dmaTcd_t *list, unsigned count);

// The same as a ring, so dmaEnableRequest() alone rearms it. Pass
// completeInterrupt false when the channel has no handler.
bool dmaSetTransferRing(dmaIdentifier_e identifier, dmaTcd_t *list, unsigned count,
                        bool completeInterrupt);

// Nothing moves until dmaEnableRequest() is called.
void dmaEnableRequest(dmaIdentifier_e identifier);
void dmaDisableRequest(dmaIdentifier_e identifier);

// Clear a stopped channel's pending error so the error vector ignores it.
void dmaClearError(dmaIdentifier_e identifier);

bool dmaChannelBusy(dmaIdentifier_e identifier);

// ES register value of the last channel error, zero until one happens.
extern uint32_t dmaLastErrorStatus;
