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

#pragma once

#include <stdbool.h>

#include "drivers/resource.h"

#include "platform/dma.h"

#define CACHE_LINE_SIZE 32
#define CACHE_LINE_MASK (CACHE_LINE_SIZE - 1)

// dmaResource_t is a opaque data type which represents a single DMA engine,
// called and implemented differently in different families of STM32s.
// The opaque data type provides uniform handling of the engine in source code.
// The engines are referenced by dmaResource_t through out the Betaflight code,
// and then converted back to DMA_ARCH_TYPE which is a native type for
// the particular MCU type when calling library functions.

typedef struct dmaResource_s dmaResource_t;

struct dmaChannelDescriptor_s;
typedef void (*dmaCallbackHandlerFuncPtr)(struct dmaChannelDescriptor_s *channelDescriptor);

typedef struct dmaChannelDescriptor_s {
    DMA_TypeDef*                dma;
    dmaResource_t               *ref;
#if PLATFORM_TRAIT_DMA_STREAM_REQUIRED
    uint8_t                     stream;
#endif
    uint32_t                    channel;
    dmaCallbackHandlerFuncPtr   irqHandlerCallback;
    uint8_t                     flagsShift;
    IRQn_Type                   irqN;
    uint32_t                    userParam;
    resourceOwner_t             resourceOwner;
    uint32_t                    completeFlag;
#if PLATFORM_TRAIT_DMA_MUX_REQUIRED
    dmamux_channel_type         *dmamux;
#endif
} dmaChannelDescriptor_t;

#define DMA_IDENTIFIER_TO_INDEX(x) ((x) - DMA_FIRST_HANDLER)

void dmaMuxEnable(dmaIdentifier_e identifier, uint32_t dmaMuxId);

bool dmaAllocate(dmaIdentifier_e identifier, resourceOwner_e owner, uint8_t resourceIndex);
void dmaEnable(dmaIdentifier_e identifier);
void dmaSetHandler(dmaIdentifier_e identifier, dmaCallbackHandlerFuncPtr callback, uint32_t priority, uint32_t userParam);

dmaIdentifier_e dmaGetIdentifier(const dmaResource_t* channel);
const resourceOwner_t *dmaGetOwner(dmaIdentifier_e identifier);
dmaChannelDescriptor_t* dmaGetDescriptorByIdentifier(const dmaIdentifier_e identifier);
uint32_t dmaGetChannel(const uint8_t channel);

