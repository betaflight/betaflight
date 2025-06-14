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
#include "drivers/resource.h"

#if defined(APM32F4)
#define PLATFORM_TRAIT_DMA_STREAM_REQUIRED 1
#endif

#define DMA_ARCH_TYPE DMA_Stream_TypeDef

typedef enum {
    DMA_NONE = 0,
    DMA1_ST0_HANDLER = 1,
    DMA1_ST1_HANDLER,
    DMA1_ST2_HANDLER,
    DMA1_ST3_HANDLER,
    DMA1_ST4_HANDLER,
    DMA1_ST5_HANDLER,
    DMA1_ST6_HANDLER,
    DMA1_ST7_HANDLER,
    DMA2_ST0_HANDLER,
    DMA2_ST1_HANDLER,
    DMA2_ST2_HANDLER,
    DMA2_ST3_HANDLER,
    DMA2_ST4_HANDLER,
    DMA2_ST5_HANDLER,
    DMA2_ST6_HANDLER,
    DMA2_ST7_HANDLER,
    DMA_LAST_HANDLER = DMA2_ST7_HANDLER
} dmaIdentifier_e;

#define DMA_DEVICE_NO(x)    ((((x)-1) / 8) + 1)
#define DMA_DEVICE_INDEX(x) ((((x)-1) % 8))
#define DMA_OUTPUT_INDEX    0
#define DMA_OUTPUT_STRING   "DMA%d Stream %d:"
#define DMA_INPUT_STRING    "DMA%d_ST%d"

#define DEFINE_DMA_CHANNEL(d, s, f) { \
    .dma = d, \
    .ref = (dmaResource_t *)d ## _Stream ## s, \
    .stream = s, \
    .irqHandlerCallback = NULL, \
    .flagsShift = f, \
    .irqN = d ## _Stream ## s ## _IRQn, \
    .userParam = 0, \
    .owner.owner = 0, \
    .owner.resourceIndex = 0 \
    }

#define DEFINE_DMA_IRQ_HANDLER(d, s, i) FAST_IRQ_HANDLER void DMA ## d ## _STR ## s ## _IRQHandler(void) {\
                                                                const uint8_t index = DMA_IDENTIFIER_TO_INDEX(i); \
                                                                dmaCallbackHandlerFuncPtr handler = dmaDescriptors[index].irqHandlerCallback; \
                                                                if (handler) \
                                                                    handler(&dmaDescriptors[index]); \
                                                            }

#define DMA_CLEAR_FLAG(d, flag) if (d->flagsShift > 31) d->dma->HIFCLR = (flag << (d->flagsShift - 32)); else d->dma->LIFCLR = (flag << d->flagsShift)
#define DMA_GET_FLAG_STATUS(d, flag) (d->flagsShift > 31 ? d->dma->HINTSTS & (flag << (d->flagsShift - 32)): d->dma->LINTSTS & (flag << d->flagsShift))

#define xDDL_EX_DMA_DeInit(dmaResource) DDL_EX_DMA_DeInit((DMA_ARCH_TYPE *)(dmaResource))
#define xDDL_EX_DMA_Init(dmaResource, initstruct) DDL_EX_DMA_Init((DMA_ARCH_TYPE *)(dmaResource), initstruct)
#define xDDL_EX_DMA_DisableResource(dmaResource) DDL_EX_DMA_DisableResource((DMA_ARCH_TYPE *)(dmaResource))
#define xDDL_EX_DMA_EnableResource(dmaResource) DDL_EX_DMA_EnableResource((DMA_ARCH_TYPE *)(dmaResource))
#define xDDL_EX_DMA_GetDataLength(dmaResource) DDL_EX_DMA_GetDataLength((DMA_ARCH_TYPE *)(dmaResource))
#define xDDL_EX_DMA_SetDataLength(dmaResource, length) DDL_EX_DMA_SetDataLength((DMA_ARCH_TYPE *)(dmaResource), length)
#define xDDL_EX_DMA_EnableIT_TC(dmaResource) DDL_EX_DMA_EnableIT_TC((DMA_ARCH_TYPE *)(dmaResource))

#define DMA_IT_TCIF         ((uint32_t)0x00000020)
#define DMA_IT_HTIF         ((uint32_t)0x00000010)
#define DMA_IT_TEIF         ((uint32_t)0x00000008)
#define DMA_IT_DMEIF        ((uint32_t)0x00000004)
#define DMA_IT_FEIF         ((uint32_t)0x00000001)

void dmaMuxEnable(dmaIdentifier_e identifier, uint32_t dmaMuxId);

#define IS_DMA_ENABLED(reg) (((DMA_ARCH_TYPE *)(reg))->SCFG & DMA_SCFGx_EN)
#define REG_NDTR NDATA

#if defined(USE_HAL_DRIVER)
// We actually need these LL case only
#define xLL_EX_DMA_DeInit(dmaResource) LL_EX_DMA_DeInit((DMA_ARCH_TYPE *)(dmaResource))
#define xLL_EX_DMA_Init(dmaResource, initstruct) LL_EX_DMA_Init((DMA_ARCH_TYPE *)(dmaResource), initstruct)
#define xLL_EX_DMA_DisableResource(dmaResource) LL_EX_DMA_DisableResource((DMA_ARCH_TYPE *)(dmaResource))
#define xLL_EX_DMA_EnableResource(dmaResource) LL_EX_DMA_EnableResource((DMA_ARCH_TYPE *)(dmaResource))
#define xLL_EX_DMA_GetDataLength(dmaResource) LL_EX_DMA_GetDataLength((DMA_ARCH_TYPE *)(dmaResource))
#define xLL_EX_DMA_SetDataLength(dmaResource, length) LL_EX_DMA_SetDataLength((DMA_ARCH_TYPE *)(dmaResource), length)
#define xLL_EX_DMA_EnableIT_TC(dmaResource) LL_EX_DMA_EnableIT_TC((DMA_ARCH_TYPE *)(dmaResource))
#endif
