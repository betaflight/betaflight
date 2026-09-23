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
// HPMicro HDMA/XDMA resource definitions and handshake-mode DMA setup helpers.
#ifndef DMA_HPMICRO_H
#define DMA_HPMICRO_H

#include "drivers/dma.h"
#include "drivers/resource.h"
#ifdef HPMSOC_HAS_HPMSDK_DMAV2
#include "hpm_dmav2_drv.h"
#else
#include "hpm_dma_drv.h"
#endif

extern dmaChannelDescriptor_t dmaChannelDescriptors[];
extern dmaChannelDescriptor_t dmaChannelDescriptors2[];

/* @brief Channel config */
typedef struct dma_handshake_config_fixed {
    uint32_t dst;
    uint32_t src;
    uint32_t size_in_byte;
    uint8_t data_width;         /* data width, value defined by DMA_TRANSFER_WIDTH_xxx */
    uint8_t ch_index;
    bool dst_fixed;
    bool src_fixed;
    uint16_t interrupt_mask;        /**< Interrupt mask */
} dma_handshake_config_fixed_t;

hpm_stat_t dma_setup_handshake_fixed(DMA_Type *ptr, dma_handshake_config_fixed_t *pconfig, bool start_transfer);

/*
 * The HPM DMA controller reports terminal-count, abort and error events through
 * the same shared IRQ.  The dispatcher stores the event(s) that caused the
 * callback here so peripheral handlers can distinguish success from failure.
 */
uint32_t hpmDmaGetChannelStatus(const dmaChannelDescriptor_t *descriptor);
uint32_t hpmDmaGetErrorCount(void);
void hpmDmaRelease(dmaIdentifier_e identifier);

// HPMicro DMA handler identifiers — defined here since HEAD removed the old-style
// per-platform DMA enums from dma.h
enum {
    DMA1_CH1_HANDLER = DMA_FIRST_HANDLER,
    DMA1_CH2_HANDLER,
    DMA1_CH3_HANDLER,
    DMA1_CH4_HANDLER,
    DMA1_CH5_HANDLER,
    DMA1_CH6_HANDLER,
    DMA1_CH7_HANDLER,
    DMA1_CH8_HANDLER,
    DMA2_CH1_HANDLER,
    DMA2_CH2_HANDLER,
    DMA2_CH3_HANDLER,
    DMA2_CH4_HANDLER,
    DMA2_CH5_HANDLER,
    DMA2_CH6_HANDLER,
    DMA2_CH7_HANDLER,
    DMA2_CH8_HANDLER,
    DMA_LAST_HANDLER = DMA2_CH8_HANDLER
};

// HPMicro DMA channel initializer.
// Maps: .dma = DMA_Type *, .ref = dmaResource_t * (auto-computed from base array),
// .stream = ch, .irqN = irq, .dmamux = rcc clock
#define DEFINE_DMA_CHANNEL(d, c, i, r, base) { \
    .dma = (void *)(d), \
    .ref = (dmaResource_t *)(base + (c)), \
    .stream = (c), \
    .channel = 0, \
    .irqHandlerCallback = NULL, \
    .flagsShift = 0, \
    .irqN = (i), \
    .userParam = 0, \
    .resourceOwner = { .owner = OWNER_FREE, .index = 0 }, \
    .dmamux = (void *)(uintptr_t)(r) \
}

// HPM-specific DMA resource helpers.
// Resolve a concrete dmaResource_t pointer from (controller, channel) at compile time
// for static initialisers.  controller: 1 = HDMA, 2 = XDMA.  channel: 0..7.
#define HPM_DMA_RESOURCE(d, c) \
    ((dmaResource_t *)((d) == 1 ? &hdma_channels[(c)] : &xdma_channels[(c)]))

// Accessors for HPM-specific data stored in generic DMA descriptor fields
#define DMA_DESC_RCC(desc)     ((uint32_t)(uintptr_t)((desc)->dmamux))
#define DMA_DESC_CHANNEL(desc) ((desc)->stream)

#endif
