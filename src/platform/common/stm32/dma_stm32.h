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

// Legacy DMA architecture, used by F4, F7, H7
#if defined(DMA_TRAIT_ARCH_STREAM_TYPE)
#include "dma_arch_stream_type.h"
#define DEFINE_DMA_IRQ_HANDLER(d, s, i) FAST_IRQ_HANDLER void DMA ## d ## _Stream ## s ## _IRQHandler(void) {\
                                                                const uint8_t index = DMA_IDENTIFIER_TO_INDEX(i); \
                                                                dmaCallbackHandlerFuncPtr handler = dmaDescriptors[index].irqHandlerCallback; \
                                                                if (handler) \
                                                                    handler(&dmaDescriptors[index]); \
                                                            }

#define DMA_CLEAR_FLAG(d, flag) if (d->flagsShift > 31) d->dma->HIFCR = (flag << (d->flagsShift - 32)); else d->dma->LIFCR = (flag << d->flagsShift)
#define DMA_GET_FLAG_STATUS(d, flag) (d->flagsShift > 31 ? d->dma->HISR & (flag << (d->flagsShift - 32)): d->dma->LISR & (flag << d->flagsShift))

// new DMA architecture, used by G4
#elif defined(DMA_TRAIT_ARCH_CHANNEL_TYPE)
#include "dma_arch_channel_type.h"
#define DEFINE_DMA_IRQ_HANDLER(d, c, i) DMA_HANDLER_CODE void DMA ## d ## _Channel ## c ## _IRQHandler(void) {\
                                                                        const uint8_t index = DMA_IDENTIFIER_TO_INDEX(i); \
                                                                        dmaCallbackHandlerFuncPtr handler = dmaDescriptors[index].irqHandlerCallback; \
                                                                        if (handler) \
                                                                            handler(&dmaDescriptors[index]); \
                                                                    }

#define DMA_CLEAR_FLAG(d, flag) d->dma->IFCR = (flag << d->flagsShift)
#define DMA_GET_FLAG_STATUS(d, flag) (d->dma->ISR & (flag << d->flagsShift))
#else
#error "No DMA architecture type defined for this platform"
#endif


#if defined(STM32F4) || defined(STM32F7)
    #define IS_DMA_ENABLED(reg) (((DMA_ARCH_TYPE *)(reg))->CR & DMA_SxCR_EN)
    #define REG_NDTR NDTR
#elif defined(STM32H7)
    // For H7, we have to differenciate DMA1/2 and BDMA for access to the control register.
    // HAL library has a macro for this, but it is extremely inefficient in that it compares
    // the address against all possible values.
    // Here, we just compare the address against regions of memory.
    #if defined(STM32H7A3xx) || defined(STM32H7A3xxQ)
    // For H7A3, if it's lower than CD_AHB2PERIPH_BASE, then it's DMA1/2 and it's stream based.
    // If not, it's BDMA and it's channel based.
    #define IS_DMA_ENABLED(reg) \
        ((uint32_t)(reg) < CD_AHB2PERIPH_BASE) ? \
            (((DMA_Stream_TypeDef *)(reg))->CR & DMA_SxCR_EN) : \
            (((BDMA_Channel_TypeDef *)(reg))->CCR & BDMA_CCR_EN)
    #else
    // For H743 and H750, if it's not in D3 peripheral area, then it's DMA1/2 and it's stream based.
    // If not, it's BDMA and it's channel based.
    #define IS_DMA_ENABLED(reg) \
        ((uint32_t)(reg) < D3_AHB1PERIPH_BASE) ? \
            (((DMA_Stream_TypeDef *)(reg))->CR & DMA_SxCR_EN) : \
            (((BDMA_Channel_TypeDef *)(reg))->CCR & BDMA_CCR_EN)
    #endif
#elif defined(STM32G4)
    #define IS_DMA_ENABLED(reg) (((DMA_ARCH_TYPE *)(reg))->CCR & DMA_CCR_EN)
    // Missing __HAL_DMA_SET_COUNTER in FW library V1.0.0
    #define __HAL_DMA_SET_COUNTER(__HANDLE__, __COUNTER__) ((__HANDLE__)->Instance->CNDTR = (uint16_t)(__COUNTER__))
#else
#error "Missing Platform declaration for DMA"
#endif
