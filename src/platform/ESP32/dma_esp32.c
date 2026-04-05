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
#include "drivers/dma_impl.h"

#include "platform/dma.h"
#include "platform/interrupt.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "hal/gdma_ll.h"
#pragma GCC diagnostic pop

#include "soc/gdma_channel.h"
#include "soc/interrupts.h"

// ESP32-S3 GDMA: 5 channels
dmaChannelDescriptor_t dmaDescriptors[DMA_LAST_HANDLER] = {
    DEFINE_DMA_CHANNEL(DMA_CH0_HANDLER),
    DEFINE_DMA_CHANNEL(DMA_CH1_HANDLER),
    DEFINE_DMA_CHANNEL(DMA_CH2_HANDLER),
    DEFINE_DMA_CHANNEL(DMA_CH3_HANDLER),
    DEFINE_DMA_CHANNEL(DMA_CH4_HANDLER),
};

static bool gdmaInitialised = false;
static int nextFreeChannel = 0;

// GDMA RX interrupt sources for each channel
static const int gdmaRxIntrSource[ESP32_GDMA_CHANNEL_COUNT] = {
    ETS_DMA_IN_CH0_INTR_SOURCE,
    ETS_DMA_IN_CH1_INTR_SOURCE,
    ETS_DMA_IN_CH2_INTR_SOURCE,
    ETS_DMA_IN_CH3_INTR_SOURCE,
    ETS_DMA_IN_CH4_INTR_SOURCE,
};

// Pool of CPU interrupt lines available for GDMA RX callbacks.
// Lines are assigned dynamically to whichever channels register handlers
// via dmaSetHandler, not tied to GDMA channel index.
static const int gdmaCpuIntrPool[] = {
    ESP32_CPU_INTR_DMA_CH0,
    ESP32_CPU_INTR_DMA_CH1,
};

#define GDMA_CPU_INTR_POOL_SIZE (int)(sizeof(gdmaCpuIntrPool) / sizeof(gdmaCpuIntrPool[0]))

static int nextFreeCpuIntr = 0;

// Map from CPU interrupt number back to GDMA channel index for ISR dispatch
static int cpuIntrToChannel[32];

// Common GDMA RX interrupt handler - dispatches to the registered callback
static void gdmaRxIsrDispatch(void *arg)
{
    int cpuIntr = (int)(uintptr_t)arg;
    int channel = cpuIntrToChannel[cpuIntr];

    // Clear RX done interrupt
    gdma_ll_rx_clear_interrupt_status(&GDMA, channel, GDMA_LL_EVENT_RX_SUC_EOF);

    dmaChannelDescriptor_t *descriptor = &dmaDescriptors[channel];
    if (descriptor->irqHandlerCallback) {
        descriptor->irqHandlerCallback(descriptor);
    }
}

void esp32DmaInit(void)
{
    if (gdmaInitialised) {
        return;
    }

    // Enable GDMA bus clock and reset
    {
        int __DECLARE_RCC_ATOMIC_ENV __attribute__((unused));
        gdma_ll_enable_bus_clock(0, true);
        gdma_ll_reset_register(0);
    }

    memset(cpuIntrToChannel, 0, sizeof(cpuIntrToChannel));
    gdmaInitialised = true;
}

dmaIdentifier_e dmaGetFreeIdentifier(void)
{
    if (nextFreeChannel >= DMA_LAST_HANDLER) {
        return DMA_NONE;
    }
    return DMA_CHANNEL_TO_IDENTIFIER(nextFreeChannel++);
}

void dmaSetHandler(dmaIdentifier_e identifier, dmaCallbackHandlerFuncPtr callback, uint32_t priority, uint32_t userParam)
{
    UNUSED(priority);

    if (identifier < DMA_FIRST_HANDLER || identifier > DMA_LAST_HANDLER) {
        return;
    }

    const int index = identifier - DMA_FIRST_HANDLER;
    dmaDescriptors[index].irqHandlerCallback = callback;
    dmaDescriptors[index].userParam = userParam;

    // Dynamically assign a CPU interrupt line from the pool to this channel
    if (nextFreeCpuIntr < GDMA_CPU_INTR_POOL_SIZE) {
        int cpuIntr = gdmaCpuIntrPool[nextFreeCpuIntr++];
        cpuIntrToChannel[cpuIntr] = index;

        // Enable RX SUC_EOF interrupt for this GDMA channel
        gdma_ll_rx_enable_interrupt(&GDMA, index, GDMA_LL_EVENT_RX_SUC_EOF, true);

        // Route GDMA RX interrupt source to CPU interrupt line
        esp32IntrRoute(cpuIntr, gdmaRxIntrSource[index]);
        esp32IntrRegister(cpuIntr, gdmaRxIsrDispatch, (void *)(uintptr_t)cpuIntr);
        esp32IntrEnable(cpuIntr);
    }
}

int dmaGetHandlerCount(void)
{
    return DMA_LAST_HANDLER;
}

int dmaGetDeviceNumber(dmaIdentifier_e identifier)
{
    UNUSED(identifier);
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
    UNUSED(ref);
    return 0;
}

void dmaEnable(dmaIdentifier_e identifier)
{
    UNUSED(identifier);
}

#endif
