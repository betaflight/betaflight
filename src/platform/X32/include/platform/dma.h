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
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "platform.h"

#if defined(X32M7)
#define PLATFORM_TRAIT_DMA_STREAM_REQUIRED 1
#define DMA_ARCH_TYPE DMA_ChannelType
#endif

#include "drivers/dma.h"

#define DMA1_CH0_HANDLER   (DMA_FIRST_HANDLER + 0)
#define DMA1_CH1_HANDLER   (DMA_FIRST_HANDLER + 1)
#define DMA1_CH2_HANDLER   (DMA_FIRST_HANDLER + 2)
#define DMA1_CH3_HANDLER   (DMA_FIRST_HANDLER + 3)
#define DMA1_CH4_HANDLER   (DMA_FIRST_HANDLER + 4)
#define DMA1_CH5_HANDLER   (DMA_FIRST_HANDLER + 5)
#define DMA1_CH6_HANDLER   (DMA_FIRST_HANDLER + 6)
#define DMA1_CH7_HANDLER   (DMA_FIRST_HANDLER + 7)
#define DMA2_CH0_HANDLER   (DMA_FIRST_HANDLER + 8)
#define DMA2_CH1_HANDLER   (DMA_FIRST_HANDLER + 9)
#define DMA2_CH2_HANDLER   (DMA_FIRST_HANDLER + 10)
#define DMA2_CH3_HANDLER   (DMA_FIRST_HANDLER + 11)
#define DMA2_CH4_HANDLER   (DMA_FIRST_HANDLER + 12)
#define DMA2_CH5_HANDLER   (DMA_FIRST_HANDLER + 13)
#define DMA2_CH6_HANDLER   (DMA_FIRST_HANDLER + 14)
#define DMA2_CH7_HANDLER   (DMA_FIRST_HANDLER + 15)
#define DMA3_CH0_HANDLER   (DMA_FIRST_HANDLER + 16)
#define DMA3_CH1_HANDLER   (DMA_FIRST_HANDLER + 17)
#define DMA3_CH2_HANDLER   (DMA_FIRST_HANDLER + 18)
#define DMA3_CH3_HANDLER   (DMA_FIRST_HANDLER + 19)
#define DMA3_CH4_HANDLER   (DMA_FIRST_HANDLER + 20)
#define DMA3_CH5_HANDLER   (DMA_FIRST_HANDLER + 21)
#define DMA3_CH6_HANDLER   (DMA_FIRST_HANDLER + 22)
#define DMA3_CH7_HANDLER   (DMA_FIRST_HANDLER + 23)
#define DMA_LAST_HANDLER    DMA3_CH7_HANDLER

#define DMA_DEVICE_NO(x)    ((((x) - 1) / DMA_NUM_CHANNELS) + 1)
#define DMA_DEVICE_INDEX(x) ((((x) - 1) % DMA_NUM_CHANNELS))
#define DMA_OUTPUT_INDEX    0
#define DMA_OUTPUT_STRING   "DMA%d Channel %d:"
#define DMA_INPUT_STRING    "DMA%d_CH%d"

#define DMA_IT_GLOB   ((uint32_t)0x00U)
#define DMA_IT_TCIF   DMA_CH_EVENT_TRANSFER_COMPLETE
#define DMA_IT_HTIF   DMA_CH_EVENT_BLOCK_TRANSFER_COMPLETE
#define DMA_IT_TEIF   DMA_CH_EVENT_ERROR
#define DMA_IT_DMEIF  DMA_CH_EVENT_ERROR
#define DMA_IT_FEIF   DMA_CH_EVENT_ERROR

#define DEFINE_DMA_CHANNEL(d, s, f) { \
    .dma = d, \
    .ref = (dmaResource_t *)d ## _CH ## s, \
    .stream = s, \
    .channel = s, \
    .irqHandlerCallback = NULL, \
    .flagsShift = f, \
    .irqN = d ## _Channel ## s ## _IRQn, \
    .userParam = 0, \
    .resourceOwner.owner = 0, \
    .resourceOwner.index = 0, \
    .completeFlag = DMA_IT_TCIF, \
}

#define DEFINE_DMA_IRQ_HANDLER(d, s, i) FAST_IRQ_HANDLER void DMA ## d ## _Channel ## s ## _IRQHandler(void) { \
    const uint8_t index = DMA_IDENTIFIER_TO_INDEX(i); \
    dmaCallbackHandlerFuncPtr handler = dmaDescriptors[index].irqHandlerCallback; \
    if (handler) { \
        handler(&dmaDescriptors[index]); \
    } \
}

#define DMA_X32_CHANNEL_RANGE_END(base) ((base) + (DMA_NUM_CHANNELS * sizeof(DMA_ChannelType)))
#define DMA_X32_CHANNEL_ADDRESS(resource) ((uintptr_t)(resource))

#define DMA_X32_CONTROLLER_FROM_CHANNEL(resource) \
    ((DMA_X32_CHANNEL_ADDRESS(resource) >= DMA1_CH0_BASE && DMA_X32_CHANNEL_ADDRESS(resource) < DMA_X32_CHANNEL_RANGE_END(DMA1_CH0_BASE)) ? DMA1 : \
     (DMA_X32_CHANNEL_ADDRESS(resource) >= DMA2_CH0_BASE && DMA_X32_CHANNEL_ADDRESS(resource) < DMA_X32_CHANNEL_RANGE_END(DMA2_CH0_BASE)) ? DMA2 : \
     (DMA_X32_CHANNEL_ADDRESS(resource) >= DMA3_CH0_BASE && DMA_X32_CHANNEL_ADDRESS(resource) < DMA_X32_CHANNEL_RANGE_END(DMA3_CH0_BASE)) ? DMA3 : NULL)

#define DMA_X32_CHANNEL_FROM_CHANNEL(resource) \
    ((DMA_X32_CHANNEL_ADDRESS(resource) >= DMA1_CH0_BASE && DMA_X32_CHANNEL_ADDRESS(resource) < DMA_X32_CHANNEL_RANGE_END(DMA1_CH0_BASE)) ? \
        (DMA_ChNumType)((DMA_X32_CHANNEL_ADDRESS(resource) - DMA1_CH0_BASE) / sizeof(DMA_ChannelType)) : \
     (DMA_X32_CHANNEL_ADDRESS(resource) >= DMA2_CH0_BASE && DMA_X32_CHANNEL_ADDRESS(resource) < DMA_X32_CHANNEL_RANGE_END(DMA2_CH0_BASE)) ? \
        (DMA_ChNumType)((DMA_X32_CHANNEL_ADDRESS(resource) - DMA2_CH0_BASE) / sizeof(DMA_ChannelType)) : \
     (DMA_X32_CHANNEL_ADDRESS(resource) >= DMA3_CH0_BASE && DMA_X32_CHANNEL_ADDRESS(resource) < DMA_X32_CHANNEL_RANGE_END(DMA3_CH0_BASE)) ? \
        (DMA_ChNumType)((DMA_X32_CHANNEL_ADDRESS(resource) - DMA3_CH0_BASE) / sizeof(DMA_ChannelType)) : DMA_CHANNEL_0)

#define DMA_X32_ALL_EVENTS \
    (DMA_CH_EVENT_TRANSFER_COMPLETE | DMA_CH_EVENT_BLOCK_TRANSFER_COMPLETE | DMA_CH_EVENT_SRC_TRANSACTION_COMPLETE | DMA_CH_EVENT_DST_TRANSACTION_COMPLETE | DMA_CH_EVENT_ERROR)

#define DMA_X32_EVENTS_FROM_FLAGS(flags) \
    ((flags) & (DMA_CH_EVENT_TRANSFER_COMPLETE | DMA_CH_EVENT_BLOCK_TRANSFER_COMPLETE | DMA_CH_EVENT_ERROR))

static inline DMA_Module *dmaX32ControllerFromChannel(DMA_ARCH_TYPE *resource)
{
    return DMA_X32_CONTROLLER_FROM_CHANNEL(resource);
}

static inline DMA_ChNumType dmaX32ChannelFromResource(DMA_ARCH_TYPE *resource)
{
    return DMA_X32_CHANNEL_FROM_CHANNEL(resource);
}

static inline uint32_t dmaX32MuxChannelFromResource(DMA_ARCH_TYPE *resource)
{
    const DMA_Module *controller = dmaX32ControllerFromChannel(resource);
    const uint32_t controllerOffset = controller == DMA2 ? DMA_NUM_CHANNELS : controller == DMA3 ? (2 * DMA_NUM_CHANNELS) : 0;

    return controllerOffset + dmaX32ChannelFromResource(resource);
}

static inline uint32_t dmaX32EventFromFlags(uint32_t flags)
{
    return DMA_X32_EVENTS_FROM_FLAGS(flags);
}

static inline DMA_ChHwHsIfType dmaX32HandshakeInterfaceFromResource(DMA_ARCH_TYPE *resource)
{
    return (DMA_ChHwHsIfType)dmaX32ChannelFromResource(resource);
}

#define DMA_Init(dmaResource, initStruct) DMA_ChannelInit(DMA_X32_CONTROLLER_FROM_CHANNEL(dmaResource), (DMA_ChInitType *)(initStruct), DMA_X32_CHANNEL_FROM_CHANNEL(dmaResource))
#define DMA_Cmd(dmaResource, newState) DMA_ChannelCmd(DMA_X32_CONTROLLER_FROM_CHANNEL(dmaResource), DMA_X32_CHANNEL_FROM_CHANNEL(dmaResource), (newState))
#define DMA_ITConfig(dmaResource, flags, newState) DMA_ChannelEventCmd(DMA_X32_CONTROLLER_FROM_CHANNEL(dmaResource), DMA_X32_CHANNEL_FROM_CHANNEL(dmaResource), DMA_X32_EVENTS_FROM_FLAGS(flags), (newState))
#define DMA_GetCurrDataCounter(dmaResource) DMA_GetTransferredNumber(DMA_X32_CONTROLLER_FROM_CHANNEL(dmaResource), DMA_X32_CHANNEL_FROM_CHANNEL(dmaResource))
#define DMA_SetCurrDataCounter(dmaResource, count) DMA_SetChannelBlockSize(DMA_X32_CONTROLLER_FROM_CHANNEL(dmaResource), DMA_X32_CHANNEL_FROM_CHANNEL(dmaResource), (count))
#define DMA_ClearFlag(dmaResource, flags) DMA_ClearChannelEventStatus(DMA_X32_CONTROLLER_FROM_CHANNEL(dmaResource), DMA_X32_CHANNEL_FROM_CHANNEL(dmaResource), DMA_X32_EVENTS_FROM_FLAGS(flags))
#define DMA_MemoryTargetConfig(dmaResource, address, target) DMA_SetChannelSourceAddress(DMA_X32_CONTROLLER_FROM_CHANNEL(dmaResource), DMA_X32_CHANNEL_FROM_CHANNEL(dmaResource), (uint32_t *)(address))
#define DMA_SourceAddressReloadCmd(dmaResource, newState) DMA_ChannelSourceAddressReloadCmd(DMA_X32_CONTROLLER_FROM_CHANNEL(dmaResource), DMA_X32_CHANNEL_FROM_CHANNEL(dmaResource), (newState))
#define DMA_DestinationAddressReloadCmd(dmaResource, newState) DMA_ChannelDestinationAddressReloadCmd(DMA_X32_CONTROLLER_FROM_CHANNEL(dmaResource), DMA_X32_CHANNEL_FROM_CHANNEL(dmaResource), (newState))

#define DMA_GetFlagStatus(dmaResource, flags) \
    ((((flags) & DMA_IT_TCIF) && (DMA_X32_CONTROLLER_FROM_CHANNEL(dmaResource)->TCINTSTS & ((uint32_t)1U << DMA_X32_CHANNEL_FROM_CHANNEL(dmaResource))) ? DMA_IT_TCIF : 0) | \
     (((flags) & DMA_IT_HTIF) && (DMA_X32_CONTROLLER_FROM_CHANNEL(dmaResource)->BTCINTSTS & ((uint32_t)1U << DMA_X32_CHANNEL_FROM_CHANNEL(dmaResource))) ? DMA_IT_HTIF : 0) | \
     (((flags) & DMA_IT_TEIF) && (DMA_X32_CONTROLLER_FROM_CHANNEL(dmaResource)->ERRINTSTS & ((uint32_t)1U << DMA_X32_CHANNEL_FROM_CHANNEL(dmaResource))) ? DMA_IT_TEIF : 0))

#define DMA_DeInit(dmaResource) do { \
    DMA_Cmd((dmaResource), DISABLE); \
    DMA_ChannelEventCmd(DMA_X32_CONTROLLER_FROM_CHANNEL(dmaResource), DMA_X32_CHANNEL_FROM_CHANNEL(dmaResource), DMA_X32_ALL_EVENTS, DISABLE); \
    DMA_ClearChannelEventStatus(DMA_X32_CONTROLLER_FROM_CHANNEL(dmaResource), DMA_X32_CHANNEL_FROM_CHANNEL(dmaResource), DMA_X32_ALL_EVENTS); \
} while (0)

#define DMA_CLEAR_FLAG(d, flag) DMA_ClearFlag((DMA_ARCH_TYPE *)(d)->ref, (flag))
#define DMA_GET_FLAG_STATUS(d, flag) DMA_GetFlagStatus((DMA_ARCH_TYPE *)(d)->ref, (flag))

#define xDMA_Init(dmaResource, initStruct) DMA_Init((DMA_ARCH_TYPE *)(dmaResource), initStruct)
#define xDMA_DeInit(dmaResource) DMA_DeInit((DMA_ARCH_TYPE *)(dmaResource))
#define xDMA_Cmd(dmaResource, newState) DMA_Cmd((DMA_ARCH_TYPE *)(dmaResource), newState)
#define xDMA_ITConfig(dmaResource, flags, newState) DMA_ITConfig((DMA_ARCH_TYPE *)(dmaResource), flags, newState)
#define xDMA_GetCurrDataCounter(dmaResource) DMA_GetCurrDataCounter((DMA_ARCH_TYPE *)(dmaResource))
#define xDMA_SetCurrDataCounter(dmaResource, count) DMA_SetCurrDataCounter((DMA_ARCH_TYPE *)(dmaResource), count)
#define xDMA_GetFlagStatus(dmaResource, flags) DMA_GetFlagStatus((DMA_ARCH_TYPE *)(dmaResource), flags)
#define xDMA_ClearFlag(dmaResource, flags) DMA_ClearFlag((DMA_ARCH_TYPE *)(dmaResource), flags)
#define xDMA_MemoryTargetConfig(dmaResource, address, target) DMA_MemoryTargetConfig((DMA_ARCH_TYPE *)(dmaResource), address, target)
#define xDMA_SourceAddressReloadCmd(dmaResource, newState) DMA_SourceAddressReloadCmd((DMA_ARCH_TYPE *)(dmaResource), (newState))
#define xDMA_DestinationAddressReloadCmd(dmaResource, newState) DMA_DestinationAddressReloadCmd((DMA_ARCH_TYPE *)(dmaResource), (newState))
#define xDMA_ControlerCmd(dmaResource, newState) DMA_ControllerCmd(DMA_X32_CONTROLLER_FROM_CHANNEL((DMA_ARCH_TYPE *)(dmaResource)), (newState))

#define IS_DMA_ENABLED(reg) DMA_ChannelIsEnabled(DMA_X32_CONTROLLER_FROM_CHANNEL((DMA_ARCH_TYPE *)(reg)), DMA_X32_CHANNEL_FROM_CHANNEL((DMA_ARCH_TYPE *)(reg)))
