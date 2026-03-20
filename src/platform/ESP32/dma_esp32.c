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

// ESP32-S3 GDMA: 5 channels
dmaChannelDescriptor_t dmaDescriptors[DMA_LAST_HANDLER] = {
    DEFINE_DMA_CHANNEL(DMA_CH0_HANDLER),
    DEFINE_DMA_CHANNEL(DMA_CH1_HANDLER),
    DEFINE_DMA_CHANNEL(DMA_CH2_HANDLER),
    DEFINE_DMA_CHANNEL(DMA_CH3_HANDLER),
    DEFINE_DMA_CHANNEL(DMA_CH4_HANDLER),
};

static int nextFreeChannel = 0;

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

    // TODO: configure GDMA interrupt handler via ESP-IDF
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
