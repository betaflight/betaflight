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

#include "stm32n6xx.h"
#include "common/utils.h"

// N6 uses HPDMA1/GPDMA1 with channel-based DMA (not stream-based like H7).
// The LL_EX functions provide a simplified interface for Betaflight's DMA abstraction.

__STATIC_INLINE uint32_t LL_EX_DMA_Channel_to_Channel(DMA_Channel_TypeDef *DMAx_Channely)
{
    // Determine channel number from address offset within the DMA peripheral
    // Channel 0 starts at offset 0x50, each channel is 0x80 apart
    uint32_t addr = (uint32_t)DMAx_Channely;
    uint32_t base;

    if (addr >= HPDMA1_BASE && addr < HPDMA1_BASE + 0x1000) {
        base = HPDMA1_BASE;
    } else {
        base = GPDMA1_BASE;
    }

    return ((addr - base) - 0x50U) / 0x80U;
}

__STATIC_INLINE DMA_TypeDef *LL_EX_DMA_Channel_to_DMA(DMA_Channel_TypeDef *DMAx_Channely)
{
    uint32_t addr = (uint32_t)DMAx_Channely;
    if (addr >= HPDMA1_BASE && addr < HPDMA1_BASE + 0x1000) {
        return HPDMA1;
    }
    return GPDMA1;
}

__STATIC_INLINE uint32_t LL_EX_DMA_Init(DMA_Channel_TypeDef *DMAx_Channely, LL_DMA_InitTypeDef *DMA_InitStruct)
{
    DMA_TypeDef *DMA = LL_EX_DMA_Channel_to_DMA(DMAx_Channely);
    const uint32_t Channel = LL_EX_DMA_Channel_to_Channel(DMAx_Channely);
    return LL_DMA_Init(DMA, Channel, DMA_InitStruct);
}

__STATIC_INLINE uint32_t LL_EX_DMA_DeInit(DMA_Channel_TypeDef *DMAx_Channely)
{
    DMA_TypeDef *DMA = LL_EX_DMA_Channel_to_DMA(DMAx_Channely);
    const uint32_t Channel = LL_EX_DMA_Channel_to_Channel(DMAx_Channely);
    return LL_DMA_DeInit(DMA, Channel);
}

__STATIC_INLINE void LL_EX_DMA_EnableResource(DMA_Channel_TypeDef *DMAx_Channely)
{
    SET_BIT(DMAx_Channely->CCR, DMA_CCR_EN);
}

__STATIC_INLINE void LL_EX_DMA_DisableResource(DMA_Channel_TypeDef *DMAx_Channely)
{
    CLEAR_BIT(DMAx_Channely->CCR, DMA_CCR_EN);
}

__STATIC_INLINE void LL_EX_DMA_EnableIT_TC(DMA_Channel_TypeDef *DMAx_Channely)
{
    SET_BIT(DMAx_Channely->CCR, DMA_CCR_TCIE);
}

__STATIC_INLINE void LL_EX_DMA_SetDataLength(DMA_Channel_TypeDef *DMAx_Channely, uint32_t NbData)
{
    DMA_TypeDef *DMA = LL_EX_DMA_Channel_to_DMA(DMAx_Channely);
    const uint32_t Channel = LL_EX_DMA_Channel_to_Channel(DMAx_Channely);
    LL_DMA_SetBlkDataLength(DMA, Channel, NbData);
}

__STATIC_INLINE uint32_t LL_EX_DMA_GetDataLength(DMA_Channel_TypeDef *DMAx_Channely)
{
    DMA_TypeDef *DMA = LL_EX_DMA_Channel_to_DMA(DMAx_Channely);
    const uint32_t Channel = LL_EX_DMA_Channel_to_Channel(DMAx_Channely);
    return LL_DMA_GetBlkDataLength(DMA, Channel);
}

__STATIC_INLINE void LL_EX_TIM_EnableIT(TIM_TypeDef *TIMx, uint32_t Sources)
{
    SET_BIT(TIMx->DIER, Sources);
}

__STATIC_INLINE void LL_EX_TIM_DisableIT(TIM_TypeDef *TIMx, uint32_t Sources)
{
    CLEAR_BIT(TIMx->DIER, Sources);
}

__STATIC_INLINE void LL_EX_TIM_CC_EnableNChannel(TIM_TypeDef *TIMx, uint32_t Channel)
{
    LL_TIM_CC_EnableChannel(TIMx, 4 * Channel);
}
