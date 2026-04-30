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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"
#include "drivers/dma.h"

#if defined(X32M7)
#define PLATFORM_TRAIT_DMA_STREAM_REQUIRED 1
#endif

#define DMA_ARCH_TYPE DMA_ChannelType

struct dmaChannelDescriptor_s;

#define DMA1_CH0_HANDLER    (DMA_FIRST_HANDLER + 0)
#define DMA1_CH1_HANDLER    (DMA_FIRST_HANDLER + 1)
#define DMA1_CH2_HANDLER    (DMA_FIRST_HANDLER + 2)
#define DMA1_CH3_HANDLER    (DMA_FIRST_HANDLER + 3)
#define DMA1_CH4_HANDLER    (DMA_FIRST_HANDLER + 4)
#define DMA1_CH5_HANDLER    (DMA_FIRST_HANDLER + 5)
#define DMA1_CH6_HANDLER    (DMA_FIRST_HANDLER + 6)
#define DMA1_CH7_HANDLER    (DMA_FIRST_HANDLER + 7)
#define DMA2_CH0_HANDLER    (DMA_FIRST_HANDLER + 8)
#define DMA2_CH1_HANDLER    (DMA_FIRST_HANDLER + 9)
#define DMA2_CH2_HANDLER    (DMA_FIRST_HANDLER + 10)
#define DMA2_CH3_HANDLER    (DMA_FIRST_HANDLER + 11)
#define DMA2_CH4_HANDLER    (DMA_FIRST_HANDLER + 12)
#define DMA2_CH5_HANDLER    (DMA_FIRST_HANDLER + 13)
#define DMA2_CH6_HANDLER    (DMA_FIRST_HANDLER + 14)
#define DMA2_CH7_HANDLER    (DMA_FIRST_HANDLER + 15)
#define DMA3_CH0_HANDLER    (DMA_FIRST_HANDLER + 16)
#define DMA3_CH1_HANDLER    (DMA_FIRST_HANDLER + 17)
#define DMA3_CH2_HANDLER    (DMA_FIRST_HANDLER + 19)
#define DMA3_CH3_HANDLER    (DMA_FIRST_HANDLER + 20)
#define DMA3_CH4_HANDLER    (DMA_FIRST_HANDLER + 21)
#define DMA3_CH5_HANDLER    (DMA_FIRST_HANDLER + 22)
#define DMA3_CH6_HANDLER    (DMA_FIRST_HANDLER + 23)
#define DMA3_CH7_HANDLER    (DMA_FIRST_HANDLER + 24)
#define DMA_LAST_HANDLER    DMA3_CH7_HANDLER

#define DMA_DEVICE_NO(x)    ((((x) - 1) / DMA_NUM_CHANNELS) + 1)
#define DMA_DEVICE_INDEX(x) ((((x) - 1) % DMA_NUM_CHANNELS))
#define DMA_OUTPUT_INDEX    0
#define DMA_OUTPUT_STRING   "DMA%d Channel %d:"
#define DMA_INPUT_STRING    "DMA%d_CH%d"

#define DMA_IT_GLOB   ((uint32_t)0x01U)
#define DMA_IT_TCIF   ((uint32_t)0x02U)
#define DMA_IT_HTIF   ((uint32_t)0x04U)
#define DMA_IT_TEIF   ((uint32_t)0x08U)
#define DMA_IT_DMEIF  ((uint32_t)0x10U)
#define DMA_IT_FEIF   ((uint32_t)0x20U)

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

static inline DMA_Module *dmaX32ControllerFromChannel(const DMA_ARCH_TYPE *resource)
{
    const uintptr_t address = (uintptr_t)resource;

    if (address >= DMA1_BASE && address < (DMA1_BASE + sizeof(DMA_Module))) {
        return DMA1;
    }
    if (address >= DMA2_BASE && address < (DMA2_BASE + sizeof(DMA_Module))) {
        return DMA2;
    }
    if (address >= DMA3_BASE && address < (DMA3_BASE + sizeof(DMA_Module))) {
        return DMA3;
    }

    return NULL;
}

static inline DMA_ChNumType dmaX32ChannelFromChannel(const DMA_ARCH_TYPE *resource)
{
    const uintptr_t address = (uintptr_t)resource;

    if (address >= DMA1_CH0_BASE && address < (DMA1_CH0_BASE + (DMA_NUM_CHANNELS * sizeof(DMA_ChannelType)))) {
        return (DMA_ChNumType)((address - DMA1_CH0_BASE) / sizeof(DMA_ChannelType));
    }
    if (address >= DMA2_CH0_BASE && address < (DMA2_CH0_BASE + (DMA_NUM_CHANNELS * sizeof(DMA_ChannelType)))) {
        return (DMA_ChNumType)((address - DMA2_CH0_BASE) / sizeof(DMA_ChannelType));
    }
    if (address >= DMA3_CH0_BASE && address < (DMA3_CH0_BASE + (DMA_NUM_CHANNELS * sizeof(DMA_ChannelType)))) {
        return (DMA_ChNumType)((address - DMA3_CH0_BASE) / sizeof(DMA_ChannelType));
    }

    return DMA_CHANNEL_0;
}

static inline uint8_t dmaX32ControllerIndex(const DMA_Module *controller)
{
    if (controller == DMA1) {
        return 1;
    }
    if (controller == DMA2) {
        return 2;
    }
    if (controller == DMA3) {
        return 3;
    }

    return 0;
}

static inline DMA_ChHwHsIfType dmaX32HandshakeInterfaceFromResource(const DMA_ARCH_TYPE *resource)
{
    return (DMA_ChHwHsIfType)dmaX32ChannelFromChannel(resource);
}

static inline uint32_t dmaX32MuxChannelFromResource(const DMA_ARCH_TYPE *resource)
{
    DMA_Module *controller = dmaX32ControllerFromChannel(resource);
    const uint8_t controllerIndex = dmaX32ControllerIndex(controller);
    return controllerIndex ? (((uint32_t)(controllerIndex - 1U) * DMA_NUM_CHANNELS) + dmaX32ChannelFromChannel(resource)) : 0U;
}

static inline uint32_t dmaX32EventFromFlags(uint32_t flags)
{
    uint32_t events = 0;

    if (flags & DMA_IT_TCIF) {
        events |= DMA_CH_EVENT_TRANSFER_COMPLETE;
    }

    if (flags & (DMA_IT_TEIF | DMA_IT_DMEIF | DMA_IT_FEIF)) {
        events |= DMA_CH_EVENT_ERROR;
    }

    return events;
}

static inline void dmaX32SetMemoryAddress(DMA_ARCH_TYPE *resource, uint32_t address, bool memoryToPeripheral)
{
    DMA_Module *controller = dmaX32ControllerFromChannel(resource);
    const DMA_ChNumType channel = dmaX32ChannelFromChannel(resource);

    if (!controller) {
        return;
    }

    if (memoryToPeripheral) {
        DMA_SetChannelSourceAddress(controller, channel, (uint32_t *)address);
    } else {
        DMA_SetChannelDestinationAddress(controller, channel, (uint32_t *)address);
    }
}

static inline void dmaX32SetPeripheralAddress(DMA_ARCH_TYPE *resource, uint32_t address, bool peripheralIsDestination)
{
    DMA_Module *controller = dmaX32ControllerFromChannel(resource);
    const DMA_ChNumType channel = dmaX32ChannelFromChannel(resource);

    if (!controller) {
        return;
    }

    if (peripheralIsDestination) {
        DMA_SetChannelDestinationAddress(controller, channel, (uint32_t *)address);
    } else {
        DMA_SetChannelSourceAddress(controller, channel, (uint32_t *)address);
    }
}

static inline void dmaX32DeInit(DMA_ARCH_TYPE *resource)
{
    DMA_Module *controller = dmaX32ControllerFromChannel(resource);
    const DMA_ChNumType channel = dmaX32ChannelFromChannel(resource);

    if (!controller) {
        return;
    }

    DMA_ChannelCmd(controller, channel, DISABLE);
    DMA_ChannelEventCmd(controller, channel,
        DMA_CH_EVENT_TRANSFER_COMPLETE |
        DMA_CH_EVENT_BLOCK_TRANSFER_COMPLETE |
        DMA_CH_EVENT_SRC_TRANSACTION_COMPLETE |
        DMA_CH_EVENT_DST_TRANSACTION_COMPLETE |
        DMA_CH_EVENT_ERROR,
        DISABLE);
    DMA_ClearChannelEventStatus(controller, channel,
        DMA_CH_EVENT_TRANSFER_COMPLETE |
        DMA_CH_EVENT_BLOCK_TRANSFER_COMPLETE |
        DMA_CH_EVENT_SRC_TRANSACTION_COMPLETE |
        DMA_CH_EVENT_DST_TRANSACTION_COMPLETE |
        DMA_CH_EVENT_ERROR);
}

static inline void dmaX32SetSourceReload(DMA_ARCH_TYPE *resource, bool enable)
{
    DMA_Module *controller = dmaX32ControllerFromChannel(resource);
    const DMA_ChNumType channel = dmaX32ChannelFromChannel(resource);

    if (controller) {
        DMA_ChannelSourceAddressReloadCmd(controller, channel, enable ? ENABLE : DISABLE);
    }
}

static inline void dmaX32SetDestinationReload(DMA_ARCH_TYPE *resource, bool enable)
{
    DMA_Module *controller = dmaX32ControllerFromChannel(resource);
    const DMA_ChNumType channel = dmaX32ChannelFromChannel(resource);

    if (controller) {
        DMA_ChannelDestinationAddressReloadCmd(controller, channel, enable ? ENABLE : DISABLE);
    }
}

static inline bool dmaX32IsEnabled(DMA_ARCH_TYPE *resource)
{
    DMA_Module *controller = dmaX32ControllerFromChannel(resource);
    const DMA_ChNumType channel = dmaX32ChannelFromChannel(resource);

    return controller ? DMA_ChannelIsEnabled(controller, channel) : false;
}

void dmaX32ClearFlag(const struct dmaChannelDescriptor_s *descriptor, uint32_t flags);
uint32_t dmaX32GetFlagStatus(const struct dmaChannelDescriptor_s *descriptor, uint32_t flags);

#define DMA_CLEAR_FLAG(d, flag) dmaX32ClearFlag((d), (flag))
#define DMA_GET_FLAG_STATUS(d, flag) dmaX32GetFlagStatus((d), (flag))

#define xDMA_Init(dmaResource, initStruct) DMA_ChannelInit(dmaX32ControllerFromChannel((DMA_ARCH_TYPE *)(dmaResource)), (DMA_InitTypeDef *)(initStruct), dmaX32ChannelFromChannel((DMA_ARCH_TYPE *)(dmaResource)))
#define xDMA_DeInit(dmaResource) dmaX32DeInit((DMA_ARCH_TYPE *)(dmaResource))
#define xDMA_Cmd(dmaResource, newState) DMA_ChannelCmd(dmaX32ControllerFromChannel((DMA_ARCH_TYPE *)(dmaResource)), dmaX32ChannelFromChannel((DMA_ARCH_TYPE *)(dmaResource)), (newState))
#define xDMA_ITConfig(dmaResource, flags, newState) DMA_ChannelEventCmd(dmaX32ControllerFromChannel((DMA_ARCH_TYPE *)(dmaResource)), dmaX32ChannelFromChannel((DMA_ARCH_TYPE *)(dmaResource)), dmaX32EventFromFlags((flags)), (newState))
#define xDMA_GetCurrDataCounter(dmaResource) DMA_GetTransferredNumber(dmaX32ControllerFromChannel((DMA_ARCH_TYPE *)(dmaResource)), dmaX32ChannelFromChannel((DMA_ARCH_TYPE *)(dmaResource)))
#define xDMA_SetCurrDataCounter(dmaResource, count) DMA_SetChannelBlockSize(dmaX32ControllerFromChannel((DMA_ARCH_TYPE *)(dmaResource)), dmaX32ChannelFromChannel((DMA_ARCH_TYPE *)(dmaResource)), (count))

#define xDMA_ControlerCmd(dmaResource, newState) DMA_ControllerCmd(dmaX32ControllerFromChannel((DMA_ARCH_TYPE *)(dmaResource)), (newState))

#define IS_DMA_ENABLED(reg) dmaX32IsEnabled((DMA_ARCH_TYPE *)(reg))
