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

#pragma once

#include "platform.h"


#define PLATFORM_TRAIT_DMA_STREAM_REQUIRED 1

#define DMA_ARCH_TYPE DMA_Stream_TypeDef

#include "drivers/dma.h"

#define DMA1_ST0_HANDLER    (DMA_FIRST_HANDLER + 0)
#define DMA1_ST1_HANDLER    (DMA_FIRST_HANDLER + 1)
#define DMA1_ST2_HANDLER    (DMA_FIRST_HANDLER + 2)
#define DMA1_ST3_HANDLER    (DMA_FIRST_HANDLER + 3)
#define DMA1_ST4_HANDLER    (DMA_FIRST_HANDLER + 4)
#define DMA1_ST5_HANDLER    (DMA_FIRST_HANDLER + 5)
#define DMA1_ST6_HANDLER    (DMA_FIRST_HANDLER + 6)
#define DMA1_ST7_HANDLER    (DMA_FIRST_HANDLER + 7)
#define DMA2_ST0_HANDLER    (DMA_FIRST_HANDLER + 8)
#define DMA2_ST1_HANDLER    (DMA_FIRST_HANDLER + 9)
#define DMA2_ST2_HANDLER    (DMA_FIRST_HANDLER + 10)
#define DMA2_ST3_HANDLER    (DMA_FIRST_HANDLER + 11)
#define DMA2_ST4_HANDLER    (DMA_FIRST_HANDLER + 12)
#define DMA2_ST5_HANDLER    (DMA_FIRST_HANDLER + 13)
#define DMA2_ST6_HANDLER    (DMA_FIRST_HANDLER + 14)
#define DMA2_ST7_HANDLER    (DMA_FIRST_HANDLER + 15)
#define DMA_LAST_HANDLER    DMA2_ST7_HANDLER

#define DMA_DEVICE_NO(x)    ((((x)-1) / 8) + 1)
#define DMA_DEVICE_INDEX(x) ((((x)-1) % 8))
#define DMA_OUTPUT_INDEX    0
#define DMA_OUTPUT_STRING   "DMA%d Stream %d:"

#define DMA_Handshake_0     0
#define DMA_Handshake_1     (DMA_Handshake_0 + 1)
#define DMA_Handshake_2     (DMA_Handshake_0 + 2)
#define DMA_Handshake_3     (DMA_Handshake_0 + 3)
#define DMA_Handshake_4     (DMA_Handshake_0 + 4)
#define DMA_Handshake_5     (DMA_Handshake_0 + 5)
#define DMA_Handshake_6     (DMA_Handshake_0 + 6)
#define DMA_Handshake_7     (DMA_Handshake_0 + 7)
#define DMA_Handshake_8     (DMA_Handshake_0 + 8)
#define DMA_Handshake_9     (DMA_Handshake_0 + 9)
#define DMA_Handshake_10    (DMA_Handshake_0 + 10)
#define DMA_Handshake_11    (DMA_Handshake_0 + 11)
#define DMA_Handshake_12    (DMA_Handshake_0 + 12)
#define DMA_Handshake_13    (DMA_Handshake_0 + 13)
#define DMA_Handshake_Rev   0xF

#define DMA_SRC_HANDSHAKING(hs)   (uint32_t)((hs & 0xf) << 7)
#define DMA_DST_HANDSHAKING(hs)   (uint32_t)((hs & 0xf) << 11)


#define DEFINE_DMA_CHANNEL(d, s, f) { \
    .dma = d, \
    .ref = (dmaResource_t *)d ## _Stream ## s, \
    .stream = s, \
    .irqHandlerCallback = NULL, \
    .flagsShift = f, \
    .irqN = d ## _Stream ## s ## _IRQn, \
    .userParam = 0, \
    .resourceOwner.owner = 0, \
    .resourceOwner.index = 0 \
    }

#define DEFINE_DMA_IRQ_HANDLER(d, s, i) FAST_IRQ_HANDLER void d ## _Stream ## s ## _IRQHandler(void) {\
                                                                const uint8_t index = DMA_IDENTIFIER_TO_INDEX(i); \
                                                                dmaCallbackHandlerFuncPtr handler = dmaDescriptors[index].irqHandlerCallback; \
                                                                if (handler) \
                                                                    handler(&dmaDescriptors[index]); \
                                                            }

//flag:CLEARBLOCK
#define DMA_CLEAR_FLAG(d, flag) ((DMA_TypeDef*)(d)->dma)->flag = (0x1 << d->flagsShift)
//flag:STATUSBLOCK
#define DMA_GET_FLAG_STATUS(d, flag) ((DMA_TypeDef*)(d)->dma)->flag & (0x1 << d->flagsShift)

// Upstream-style channel-busy test for the DW_ahb DMA: the enable bit is not
// in the per-channel registers but in the controller-level CHENREG bitmap
// (bit N = channel N). Derive the controller base and channel index from the
// stream pointer the same way the LL_EX_DMA_* helpers do (controllers are
// 64K-aligned; channel register blocks are 0x58 apart). Note a channel
// paused at a block boundary keeps its CH_EN bit set, so this also reports
// "enabled" for a paused-but-unfinished block — the desired "transfer still
// in progress" semantics.
#define IS_DMA_ENABLED(reg) \
    (((DMA_TypeDef *)((uint32_t)(reg) & ~0xFFFFU))->CHENREG & \
        (DMA_CHENREG_CH_EN_0 << (((uint32_t)(reg) & 0xFFFFU) / 0x58U)))


#if defined(USE_HAL_DRIVER)

// We actually need these LL case only

#define xLL_EX_DMA_DeInit(dmaResource) LL_EX_DMA_DeInit((DMA_ARCH_TYPE *)(dmaResource))
#define xLL_EX_DMA_Init(dmaResource, initstruct) LL_EX_DMA_Init((DMA_ARCH_TYPE *)(dmaResource), initstruct)
#define xLL_EX_DMA_DisableResource(dmaResource) LL_EX_DMA_DisableResource((DMA_ARCH_TYPE *)(dmaResource))
#define xLL_EX_DMA_EnableResource(dmaResource) LL_EX_DMA_EnableResource((DMA_ARCH_TYPE *)(dmaResource))
#define xLL_EX_DMA_GetDataLength(dmaResource) LL_EX_DMA_GetDataLength((DMA_ARCH_TYPE *)(dmaResource))
#define xLL_EX_DMA_SetDataLength(dmaResource, length) LL_EX_DMA_SetDataLength((DMA_ARCH_TYPE *)(dmaResource), length)
#define xLL_EX_DMA_EnableIT_TC(dmaResource) LL_EX_DMA_EnableIT_TC((DMA_ARCH_TYPE *)(dmaResource))
#define xLL_EX_DMA_SetSrcAddress(dmaResource, SrcAddress) LL_EX_DMA_SetSrcAddress((DMA_ARCH_TYPE *)(dmaResource), SrcAddress)
#define xLL_EX_DMA_ConsumeRequest(dmaResource) LL_EX_DMA_ConsumeRequest((DMA_ARCH_TYPE *)(dmaResource))
#else

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
