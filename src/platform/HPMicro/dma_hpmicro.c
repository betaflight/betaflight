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

// DMA descriptor table and interrupt dispatch for the HPM HDMA/XDMA
// controllers, plus DMAMUX routing and channel lifecycle helpers.

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"
#include "dma.h"

#ifdef USE_DMA
#include "dma_hpmicro.h"
#include "drivers/dma.h"
#include "nvic_hpmicro.h"
#include "hpm_clock_drv.h"
#include "hpm_dmamux_drv.h"

/*
 * DMA resource objects - must be defined before dmaDescriptors
 * so they can be referenced in the descriptor initializers.
 */
struct dmaResource_s hdma_channels[8] = {
    {.base = HPM_HDMA, .channel = 0, },
    {.base = HPM_HDMA, .channel = 1, },
    {.base = HPM_HDMA, .channel = 2, },
    {.base = HPM_HDMA, .channel = 3, },
    {.base = HPM_HDMA, .channel = 4, },
    {.base = HPM_HDMA, .channel = 5, },
    {.base = HPM_HDMA, .channel = 6, },
    {.base = HPM_HDMA, .channel = 7, },
};

struct dmaResource_s xdma_channels[8] = {
    {.base = HPM_XDMA, .channel = 0, },
    {.base = HPM_XDMA, .channel = 1, },
    {.base = HPM_XDMA, .channel = 2, },
    {.base = HPM_XDMA, .channel = 3, },
    {.base = HPM_XDMA, .channel = 4, },
    {.base = HPM_XDMA, .channel = 5, },
    {.base = HPM_XDMA, .channel = 6, },
    {.base = HPM_XDMA, .channel = 7, },
};

/*
 * DMA descriptors.
 */
dmaChannelDescriptor_t dmaDescriptors[DMA_LAST_HANDLER] = {
    DEFINE_DMA_CHANNEL(HPM_HDMA, 0, IRQn_HDMA, clock_hdma, hdma_channels),
    DEFINE_DMA_CHANNEL(HPM_HDMA, 1, IRQn_HDMA, clock_hdma, hdma_channels),
    DEFINE_DMA_CHANNEL(HPM_HDMA, 2, IRQn_HDMA, clock_hdma, hdma_channels),
    DEFINE_DMA_CHANNEL(HPM_HDMA, 3, IRQn_HDMA, clock_hdma, hdma_channels),
    DEFINE_DMA_CHANNEL(HPM_HDMA, 4, IRQn_HDMA, clock_hdma, hdma_channels),
    DEFINE_DMA_CHANNEL(HPM_HDMA, 5, IRQn_HDMA, clock_hdma, hdma_channels),
    DEFINE_DMA_CHANNEL(HPM_HDMA, 6, IRQn_HDMA, clock_hdma, hdma_channels),
    DEFINE_DMA_CHANNEL(HPM_HDMA, 7, IRQn_HDMA, clock_hdma, hdma_channels),

    DEFINE_DMA_CHANNEL(HPM_XDMA, 0, IRQn_XDMA, clock_xdma, xdma_channels),
    DEFINE_DMA_CHANNEL(HPM_XDMA, 1, IRQn_XDMA, clock_xdma, xdma_channels),
    DEFINE_DMA_CHANNEL(HPM_XDMA, 2, IRQn_XDMA, clock_xdma, xdma_channels),
    DEFINE_DMA_CHANNEL(HPM_XDMA, 3, IRQn_XDMA, clock_xdma, xdma_channels),
    DEFINE_DMA_CHANNEL(HPM_XDMA, 4, IRQn_XDMA, clock_xdma, xdma_channels),
    DEFINE_DMA_CHANNEL(HPM_XDMA, 5, IRQn_XDMA, clock_xdma, xdma_channels),
    DEFINE_DMA_CHANNEL(HPM_XDMA, 6, IRQn_XDMA, clock_xdma, xdma_channels),
    DEFINE_DMA_CHANNEL(HPM_XDMA, 7, IRQn_XDMA, clock_xdma, xdma_channels),

};

static volatile uint32_t dmaErrorCount;

int dmaGetHandlerCount(void)
{
    return DMA_LAST_HANDLER;
}

void hpmDmaRelease(dmaIdentifier_e identifier)
{
    const int index = DMA_IDENTIFIER_TO_INDEX(identifier);

    if (index < 0 || index >= dmaGetHandlerCount()) {
        return;
    }

    dmaDescriptors[index].resourceOwner.owner = OWNER_FREE;
    dmaDescriptors[index].resourceOwner.index = 0;
}

hpm_stat_t dma_setup_handshake_fixed(DMA_Type *ptr, dma_handshake_config_fixed_t *pconfig, bool start_transfer)
{
    if (pconfig->ch_index >= DMA_SOC_CHANNEL_NUM) {
        return status_invalid_argument;
    }

    dma_channel_config_t config = { 0 };
    dma_default_channel_config(ptr, &config);

    if (true == pconfig->dst_fixed) {
        config.dst_addr_ctrl = DMA_ADDRESS_CONTROL_FIXED;
        config.dst_mode = DMA_HANDSHAKE_MODE_HANDSHAKE;
    }
    if (true == pconfig->src_fixed) {
        config.src_addr_ctrl = DMA_ADDRESS_CONTROL_FIXED;
        config.src_mode = DMA_HANDSHAKE_MODE_HANDSHAKE;
    }

    config.src_width = pconfig->data_width;
    config.dst_width = pconfig->data_width;
    config.src_addr = pconfig->src;
    config.dst_addr = pconfig->dst;
    config.size_in_byte = pconfig->size_in_byte;
    config.interrupt_mask = pconfig->interrupt_mask;
    /*  In DMA handshake case, source burst size must be 1 transfer, that is 0. */
    config.src_burst_size = 0;

    return dma_setup_channel(ptr, pconfig->ch_index, &config, start_transfer);
}

static FAST_CODE void dmaDispatchIrq(DMA_Type *dma, dmaIdentifier_e first, dmaIdentifier_e last)
{
    uint32_t intStat;

    while ((intStat = dma->INTSTATUS) != 0) {
        // Handle failures before terminal-count callbacks.  A paired transfer
        // (for example SPI TX/RX) can report RX completion and TX failure in
        // the same snapshot; failure cleanup must win in that case.
        for (dmaIdentifier_e i = first; i <= last; i++) {
            const uint8_t index = DMA_IDENTIFIER_TO_INDEX(i);
            dmaChannelDescriptor_t *desc = &dmaDescriptors[index];
            const uint8_t ch = DMA_DESC_CHANNEL(desc);
            const uint32_t chError = DMA_CHANNEL_IRQ_STATUS_ERROR(ch);
            const uint32_t chAbort = DMA_CHANNEL_IRQ_STATUS_ABORT(ch);
            const uint32_t chTc = DMA_CHANNEL_IRQ_STATUS_TC(ch);
            const uint32_t failureIrqStatus = intStat & (chError | chAbort);

            if (!failureIrqStatus) {
                continue;
            }

            uint32_t transferStatus = 0;
            uint32_t channelIrqStatus = failureIrqStatus;

            if (failureIrqStatus & chError) {
                transferStatus |= DMA_CHANNEL_STATUS_ERROR;
            }
            if (failureIrqStatus & chAbort) {
                transferStatus |= DMA_CHANNEL_STATUS_ABORT;
            }
            if (intStat & chTc) {
                transferStatus |= DMA_CHANNEL_STATUS_TC;
                channelIrqStatus |= chTc;
            }

            desc->completeFlag = transferStatus;
            dma->INTSTATUS = channelIrqStatus;
            intStat &= ~channelIrqStatus;
            dma_disable_channel(dma, ch);
            dmaErrorCount++;

            if (desc->irqHandlerCallback) {
                desc->irqHandlerCallback(desc);
            }
        }

        for (dmaIdentifier_e i = first; i <= last; i++) {
            const uint8_t index = DMA_IDENTIFIER_TO_INDEX(i);
            dmaChannelDescriptor_t *desc = &dmaDescriptors[index];
            const uint8_t ch = DMA_DESC_CHANNEL(desc);
            const uint32_t chTc = DMA_CHANNEL_IRQ_STATUS_TC(ch);

            if (!(intStat & chTc)) {
                continue;
            }

            desc->completeFlag = DMA_CHANNEL_STATUS_TC;
            dma->INTSTATUS = chTc;
            if (desc->irqHandlerCallback) {
                desc->irqHandlerCallback(desc);
            }
        }
    }
}

void hdma_isr(void)
{
    dmaDispatchIrq(HPM_HDMA, DMA1_CH1_HANDLER, DMA1_CH8_HANDLER);
}

SDK_DECLARE_EXT_ISR_M(IRQn_HDMA, hdma_isr)
void xdma_isr(void)
{
    dmaDispatchIrq(HPM_XDMA, DMA2_CH1_HANDLER, DMA2_CH8_HANDLER);
}

SDK_DECLARE_EXT_ISR_M(IRQn_XDMA, xdma_isr)
uint32_t hpmDmaGetChannelStatus(const dmaChannelDescriptor_t *descriptor)
{
    return descriptor->completeFlag;
}

uint32_t hpmDmaGetErrorCount(void)
{
    return dmaErrorCount;
}

uint32_t dmaGetDataLength(dmaResource_t *ref)
{
    const dmaIdentifier_e id = dmaGetIdentifier(ref);

    if (id == DMA_NONE) {
        return 0;
    }
    const dmaChannelDescriptor_t *desc = dmaGetDescriptorByIdentifier(id);
    return dma_get_remaining_transfer_size(desc->dma, DMA_DESC_CHANNEL(desc));
}

void dmaEnable(dmaIdentifier_e identifier)
{
    const int index = DMA_IDENTIFIER_TO_INDEX(identifier);
    clock_add_to_group(DMA_DESC_RCC(&dmaDescriptors[index]), 0);
}

void dmaMuxEnable(dmaIdentifier_e identifier, uint32_t dmaMuxId)
{
    const dmaChannelDescriptor_t *desc = dmaGetDescriptorByIdentifier(identifier);
    DMA_Type *dma = (DMA_Type *) desc->dma;
    dmamux_config(HPM_DMAMUX, DMA_SOC_CHN_TO_DMAMUX_CHN(dma, DMA_DESC_CHANNEL(desc)), dmaMuxId, true);
}

void dmaSetHandler(dmaIdentifier_e identifier, dmaCallbackHandlerFuncPtr callback, uint32_t priority,
                   uint32_t userParam)
{
    const int index = DMA_IDENTIFIER_TO_INDEX(identifier);
    dmaChannelDescriptor_t *desc = &dmaDescriptors[index];

    clock_add_to_group(DMA_DESC_RCC(desc), 0);
    desc->irqHandlerCallback = callback;
    desc->userParam = userParam;
    desc->completeFlag = DMA_CHANNEL_STATUS_ONGOING;
    dma_clear_transfer_status(desc->dma, DMA_DESC_CHANNEL(desc));
    dma_enable_channel_interrupt(desc->dma, DMA_DESC_CHANNEL(desc), DMA_INTERRUPT_MASK_ALL);
    // Callers pass Betaflight NVIC-encoded priorities (NVIC_PRIO_*); decode
    // them for the PLIC instead of clamping the raw encoded value.
    intc_m_enable_irq_with_priority(desc->irqN, hpmPlicPriorityFromNvic(priority));
}

#define DMA_OUTPUT_STRING   "DMA%d Channel %d:"
#define DMA_INPUT_STRING    "DMA%d_CH%d"

const char *dmaGetDisplayString(void)
{
    return DMA_OUTPUT_STRING;
}

#define DMA_DEVICE_NO(x)    ((((x)-1) / 8) + 1)
#define DMA_DEVICE_INDEX(x) ((((x)-1) % 8))
int dmaGetDeviceIndex(dmaIdentifier_e identifier)
{
    return DMA_DEVICE_INDEX(identifier);
}

int dmaGetDeviceNumber(dmaIdentifier_e identifier)
{
    return DMA_DEVICE_NO(identifier);
}
#endif
