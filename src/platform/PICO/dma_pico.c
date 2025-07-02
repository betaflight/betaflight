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

#include "drivers/dma.h"

#include "platform/dma.h"
#include "pico/multicore.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

dmaChannelDescriptor_t dmaDescriptors[DMA_LAST_HANDLER] = {
    DEFINE_DMA_CHANNEL(DMA_CH0_HANDLER),
    DEFINE_DMA_CHANNEL(DMA_CH1_HANDLER),
    DEFINE_DMA_CHANNEL(DMA_CH2_HANDLER),
    DEFINE_DMA_CHANNEL(DMA_CH3_HANDLER),
    DEFINE_DMA_CHANNEL(DMA_CH4_HANDLER),
    DEFINE_DMA_CHANNEL(DMA_CH5_HANDLER),
    DEFINE_DMA_CHANNEL(DMA_CH6_HANDLER),
    DEFINE_DMA_CHANNEL(DMA_CH7_HANDLER),
    DEFINE_DMA_CHANNEL(DMA_CH8_HANDLER),
    DEFINE_DMA_CHANNEL(DMA_CH9_HANDLER),
    DEFINE_DMA_CHANNEL(DMA_CH10_HANDLER),
    DEFINE_DMA_CHANNEL(DMA_CH11_HANDLER),
#ifdef RP2350
    DEFINE_DMA_CHANNEL(DMA_CH12_HANDLER),
    DEFINE_DMA_CHANNEL(DMA_CH13_HANDLER),
    DEFINE_DMA_CHANNEL(DMA_CH14_HANDLER),
    DEFINE_DMA_CHANNEL(DMA_CH15_HANDLER),
#endif
};

void dma_irq_handler(bool isIrq1)
{
    uint32_t status = isIrq1 ? dma_hw->ints1 : dma_hw->ints0; // Read the status register once

    // Iterate through all possible DMA channels that have triggered an interrupt
    // channel equates to index in the dmaDescriptors array
    for (uint8_t channel = 0; channel < DMA_LAST_HANDLER; channel++) {
        if (status & (1u << channel)) {
            uint8_t index = DMA_CHANNEL_TO_INDEX(channel);
            // Call the handler if it is set
            if (dmaDescriptors[index].irqHandlerCallback) {
                dmaDescriptors[index].irqHandlerCallback(&dmaDescriptors[index]);
            }

            // Acknowledge the interrupt for this channel
            if (isIrq1) {
                dma_channel_acknowledge_irq1(channel);
            } else {
                dma_channel_acknowledge_irq0(channel);
            }
        }
    }
}
void dma_irq0_handler(void)
{
    dma_irq_handler(false);
}

void dma_irq1_handler(void)
{
    dma_irq_handler(true);
}

dmaIdentifier_e dmaGetFreeIdentifier(void)
{
    const int channel = dma_claim_unused_channel(false);
    // NOTE: dma_claim_unused_channel returns -1 if no channel is available
    if (channel == -1) {
        return DMA_NONE; // No free channel available
    }
    return DMA_CHANNEL_TO_IDENTIFIER(channel);
}

void dmaSetHandler(dmaIdentifier_e identifier, dmaCallbackHandlerFuncPtr callback, uint32_t priority, uint32_t userParam)
{
    if (identifier < DMA_FIRST_HANDLER || identifier > DMA_LAST_HANDLER) {
        return; // Invalid identifier
    }

    UNUSED(priority);
    /*
        Assign the interrupt handler for the DMA channel based on the core
        this is to minimise contention on the DMA IRQs.

        Each core has its own DMA IRQ:
        - CORE 0 uses DMA_IRQ_0
        - CORE 1 uses DMA_IRQ_1

        If we specify a core to be used for interrupts we will use the corresponding DMA IRQ.
    */
#if defined(USE_MULTICORE) && defined(DMA_IRQ_CORE_NUM)
    const uint8_t core = DMA_IRQ_CORE_NUM;
#elif defined(USE_MULTICORE)
    // Get the current core number
    const uint8_t core = get_core_num();
#else
    const uint8_t core = 0;
#endif

    const int index = DMA_IDENTIFIER_TO_INDEX(identifier);
    const uint32_t channel = dmaDescriptors[index].channel;
    static bool dma_irqN_handler_registered[2];

    if (!dma_irqN_handler_registered[core]) {
        // Register the DMA IRQ handler if needed
        dmaDescriptors[index].irqN = core ? DMA_IRQ_1_IRQn : DMA_IRQ_0_IRQn;
        irq_handler_t irq_handler = core ? dma_irq1_handler : dma_irq0_handler;


        irq_set_exclusive_handler(dmaDescriptors[index].irqN, irq_handler);
        irq_set_enabled(dmaDescriptors[index].irqN, true);

        dma_irqN_handler_registered[core] = true;
    }

    if (core) {
        dma_channel_set_irq1_enabled(channel, true);
    } else {
        dma_channel_set_irq0_enabled(channel, true);
    }

    dmaDescriptors[index].irqHandlerCallback = callback;
    dmaDescriptors[index].userParam = userParam;
}

#endif
