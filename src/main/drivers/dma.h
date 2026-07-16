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
#include <stdint.h>

#include "drivers/resource.h"

#define CACHE_LINE_SIZE 32
#define CACHE_LINE_MASK (CACHE_LINE_SIZE - 1)

#define DMA_NONE 0
#define DMA_FIRST_HANDLER 1

// dmaIdentifier_e values are platform-specific.
// Specific handler identifiers (e.g. DMA1_ST0_HANDLER) are defined
// in platform headers and only used within src/platform code.
typedef int dmaIdentifier_e;

// dmaResource_t is an opaque data type which represents a single DMA engine,
// called and implemented differently in different families of MCUs.
// The opaque data type provides uniform handling of the engine in source code.
// The engines are referenced by dmaResource_t throughout the Betaflight code,
// and then converted back to the native MCU type when calling library functions
// within platform implementation code.
typedef struct dmaResource_s dmaResource_t;

struct dmaChannelDescriptor_s;
typedef void (*dmaCallbackHandlerFuncPtr)(struct dmaChannelDescriptor_s *channelDescriptor);

typedef struct dmaChannelDescriptor_s {
    void                        *dma;
    dmaResource_t               *ref;
    uint8_t                     stream;
    uint32_t                    channel;
    dmaCallbackHandlerFuncPtr   irqHandlerCallback;
    uint8_t                     flagsShift;
    int32_t                     irqN;
    uint32_t                    userParam;
    resourceOwner_t             resourceOwner;
    uint32_t                    completeFlag;
    void                        *dmamux;
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

// Hardware abstraction - implemented per platform
int dmaGetHandlerCount(void);
int dmaGetDeviceNumber(dmaIdentifier_e identifier);
int dmaGetDeviceIndex(dmaIdentifier_e identifier);
const char *dmaGetDisplayString(void);
uint32_t dmaGetDataLength(dmaResource_t *ref);
