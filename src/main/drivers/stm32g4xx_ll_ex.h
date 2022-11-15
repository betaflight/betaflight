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

#include "stm32g4xx.h"
#include "common/utils.h"

#define DMA_CHANNEL_MASK 0xFFU

// As specified in RM0440 Rev 1 "11.6 DMA registers"
#define DMA_FIRST_CHANNEL_OFFSET 0x08U
#define DMA_CHANNEL_SIZE 0x14U

__STATIC_INLINE DMA_TypeDef *LL_EX_DMA_Channel_to_DMA(DMA_Channel_TypeDef *DMAx_Channely)
{
    // clear channel address
    return (DMA_TypeDef *) (((uint32_t) DMAx_Channely) & ((uint32_t) ~DMA_CHANNEL_MASK));
}

__STATIC_INLINE uint32_t LL_EX_DMA_Channel_to_Channel(DMA_Channel_TypeDef *DMAx_Channely)
{

    return (((uint32_t) DMAx_Channely & DMA_CHANNEL_MASK) - DMA_FIRST_CHANNEL_OFFSET) / DMA_CHANNEL_SIZE + 1;
}

#undef DMA_CHANNEL_MASK
#undef DMA_FIRST_CHANNEL_OFFSET
#undef DMA_CHANNEL_SIZE

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

__STATIC_INLINE void LL_EX_DMA_SetDataLength(DMA_Channel_TypeDef* DMAx_Channely, uint32_t NbData)
{
    MODIFY_REG(DMAx_Channely->CNDTR, DMA_CNDTR_NDT, NbData);
}

__STATIC_INLINE uint32_t LL_EX_DMA_GetDataLength(DMA_Channel_TypeDef* DMAx_Channely)
{
    return READ_BIT(DMAx_Channely->CNDTR, DMA_CNDTR_NDT);
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
