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

#include "drivers/resource.h"


#define CACHE_LINE_SIZE 32
#define CACHE_LINE_MASK (CACHE_LINE_SIZE - 1)

// dmaResource_t is a opaque data type which represents a single DMA engine,
// called and implemented differently in different families of STM32s.
// The opaque data type provides uniform handling of the engine in source code.
// The engines are referenced by dmaResource_t through out the Betaflight code,
// and then converted back to DMA_ARCH_TYPE which is a native type for
// the particular MCU type when calling library functions.

// Forward declaration of the enum
typedef struct dmaResource_s dmaResource_t;

#if defined(DMA_TRAIT_ARCH_STREAM_TYPE)
// currently STM32 {F4, H7, F7} and APM32 {F4}
/// @note H7 has stream based DMA and channel based BDMA, but we ignore BDMA (for now).
#define DMA_ARCH_TYPE DMA_Stream_TypeDef
#elif defined(DMA_TRAIT_ARCH_AT_TYPE)
// only AT32F435 (probably all AT32F4...)
#define DMA_ARCH_TYPE dma_channel_type
#elif defined(DMA_TRAIT_ARCH_CHANNEL_TYPE)
// currently others (STM32G4, STM32H5, pico ? ...)
#define DMA_ARCH_TYPE DMA_Channel_TypeDef
#else
#error "No DMA architecture type defined for this platform"
#endif

struct dmaChannelDescriptor_s;
typedef void (*dmaCallbackHandlerFuncPtr)(struct dmaChannelDescriptor_s *channelDescriptor);

typedef struct dmaChannelDescriptor_s {
    DMA_TypeDef*                dma;
    dmaResource_t               *ref;
#if defined(DMA_TRAIT_SPI_STREAM)
    uint8_t                     stream;
#endif
    uint32_t                    channel;
    dmaCallbackHandlerFuncPtr   irqHandlerCallback;
    uint8_t                     flagsShift;
    IRQn_Type                   irqN;
    uint32_t                    userParam;
    resourceOwner_t             owner;
    uint8_t                     resourceIndex;
    uint32_t                    completeFlag;
#if defined(USE_ATBSP_DRIVER)
    dmamux_channel_type         *dmamux;
#endif
} dmaChannelDescriptor_t;

#define DMA_IDENTIFIER_TO_INDEX(x) ((x) - 1)


// Macros to avoid direct register and register bit access
// default value that could be overwriten by platform specific defines
#ifndef IS_DMA_ENABLED
#define IS_DMA_ENABLED(reg) (((DMA_ARCH_TYPE *)(reg))->CCR & DMA_CCR_EN)
#endif

#ifndef DMAx_SetMemoryAddress
#define DMAx_SetMemoryAddress(reg, address) ((DMA_ARCH_TYPE *)(reg))->CMAR = (uint32_t)&s->port.txBuffer[s->port.txBufferTail]
#endif


dmaIdentifier_e dmaAllocate(dmaIdentifier_e identifier, resourceOwner_e owner, uint8_t resourceIndex);
void dmaEnable(dmaIdentifier_e identifier);
void dmaSetHandler(dmaIdentifier_e identifier, dmaCallbackHandlerFuncPtr callback, uint32_t priority, uint32_t userParam);

dmaIdentifier_e dmaGetIdentifier(const dmaResource_t* channel);
const resourceOwner_t *dmaGetOwner(dmaIdentifier_e identifier);
dmaChannelDescriptor_t* dmaGetDescriptorByIdentifier(const dmaIdentifier_e identifier);
uint32_t dmaGetChannel(const uint8_t channel);

//
// Wrapper macros to cast dmaResource_t back into DMA_ARCH_TYPE
//

#if defined(USE_HAL_DRIVER)

// We actually need these LL case only

#define xLL_EX_DMA_DeInit(dmaResource) LL_EX_DMA_DeInit((DMA_ARCH_TYPE *)(dmaResource))
#define xLL_EX_DMA_Init(dmaResource, initstruct) LL_EX_DMA_Init((DMA_ARCH_TYPE *)(dmaResource), initstruct)
#define xLL_EX_DMA_DisableResource(dmaResource) LL_EX_DMA_DisableResource((DMA_ARCH_TYPE *)(dmaResource))
#define xLL_EX_DMA_EnableResource(dmaResource) LL_EX_DMA_EnableResource((DMA_ARCH_TYPE *)(dmaResource))
#define xLL_EX_DMA_GetDataLength(dmaResource) LL_EX_DMA_GetDataLength((DMA_ARCH_TYPE *)(dmaResource))
#define xLL_EX_DMA_SetDataLength(dmaResource, length) LL_EX_DMA_SetDataLength((DMA_ARCH_TYPE *)(dmaResource), length)
#define xLL_EX_DMA_EnableIT_TC(dmaResource) LL_EX_DMA_EnableIT_TC((DMA_ARCH_TYPE *)(dmaResource))

#elif defined(USE_ATBSP_DRIVER)

#else
/// @todo [DMA-cleanup] what's inside else

#define xDMA_Init(dmaResource, initStruct) DMA_Init((DMA_ARCH_TYPE *)(dmaResource), initStruct)
#define xDMA_DeInit(dmaResource) DMA_DeInit((DMA_ARCH_TYPE *)(dmaResource))
#define xDMA_Cmd(dmaResource, newState) DMA_Cmd((DMA_ARCH_TYPE *)(dmaResource), newState)
#define xDMA_ITConfig(dmaResource, flags, newState) DMA_ITConfig((DMA_ARCH_TYPE *)(dmaResource), flags, newState)
#define xDMA_GetCurrDataCounter(dmaResource) DMA_GetCurrDataCounter((DMA_ARCH_TYPE *)(dmaResource))
#define xDMA_SetCurrDataCounter(dmaResource, count) DMA_SetCurrDataCounter((DMA_ARCH_TYPE *)(dmaResource), count)
#define xDMA_GetFlagStatus(dmaResource, flags) DMA_GetFlagStatus((DMA_ARCH_TYPE *)(dmaResource), flags)
#define xDMA_ClearFlag(dmaResource, flags) DMA_ClearFlag((DMA_ARCH_TYPE *)(dmaResource), flags)
#define xDMA_MemoryTargetConfig(dmaResource, address, target) DMA_MemoryTargetConfig((DMA_ARCH_TYPE *)(dmaResource), address, target)

#endif
