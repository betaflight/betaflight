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
#include "dma_arch_stream_type.h"

/// @todo [DMA-Cleanup] this is really similar to the STM32 version (DMA_TRAIT_ARCH_STREAM_TYPE ones),
// except few naming like _STR instead of _Stream
// and HIFCLR/LIFCLR instead of HIFCR/LIFCR
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

#define IS_DMA_ENABLED(reg) (((DMA_ARCH_TYPE *)(reg))->SCFG & DMA_SCFGx_EN)
#define REG_NDTR NDATA