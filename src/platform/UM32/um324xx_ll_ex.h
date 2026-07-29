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

#include "um324xx.h"
#include "um324xx_ll_dma.h"
#include "common/utils.h"



#define DMA_CHANNEL_MASK 0xFFFFU
#define DMA_FIRST_CHANNEL_OFFSET 0x58U


__STATIC_INLINE DMA_TypeDef *LL_EX_DMA_Channel_to_DMA(DMA_Stream_TypeDef *DMAx_Channely)
{
    // clear channel address
    return (DMA_TypeDef *) (((uint32_t) DMAx_Channely) & ((uint32_t) ~DMA_CHANNEL_MASK));
}

__STATIC_INLINE uint32_t LL_EX_DMA_Channel_to_Channel(DMA_Stream_TypeDef *DMAx_Channely)
{
    return ((uint32_t) DMAx_Channely & DMA_CHANNEL_MASK) / DMA_FIRST_CHANNEL_OFFSET;
}

__STATIC_INLINE void LL_EX_DMA_ConsumeRequest(DMA_Stream_TypeDef* DMAx_Channely)
{
    static uint32_t noiseRequest[1] = {0};
    DMA_TypeDef *DMAx = (DMA_TypeDef *) (((uint32_t) DMAx_Channely) & ((uint32_t) ~DMA_CHANNEL_MASK));;
    uint32_t Channel = ((uint32_t) DMAx_Channely & DMA_CHANNEL_MASK) / DMA_FIRST_CHANNEL_OFFSET;

    DMAx_Channely->SAR = (uint32_t)&noiseRequest[0];
    DMAx_Channely->CTLH = 1;
    DMAx->CHENREG = ((DMA_CHENREG_CH_EN_WE_0 << Channel) | (DMA_CHENREG_CH_EN_0 << Channel)); __NOP();
}

#undef DMA_CHANNEL_MASK
#undef DMA_FIRST_CHANNEL_OFFSET


__STATIC_INLINE uint32_t LL_EX_DMA_Init(DMA_Stream_TypeDef *DMAx_Channely, LL_DMA_InitTypeDef *DMA_InitStruct)
{
    DMA_TypeDef *DMAx = LL_EX_DMA_Channel_to_DMA(DMAx_Channely);
    const uint32_t Channel = LL_EX_DMA_Channel_to_Channel(DMAx_Channely);

    SET_BIT(DMAx->CFGREG, DMA_CFGREG_DMA_EN);
    
    return LL_DMA_Init(DMAx, Channel, DMA_InitStruct);
}

__STATIC_INLINE uint32_t LL_EX_DMA_DeInit(DMA_Stream_TypeDef *DMAx_Channely)
{
    DMA_TypeDef *DMAx = LL_EX_DMA_Channel_to_DMA(DMAx_Channely);
    const uint32_t Channel = LL_EX_DMA_Channel_to_Channel(DMAx_Channely);

    return LL_DMA_DeInit(DMAx, Channel);
}

__STATIC_INLINE void LL_EX_DMA_EnableResource(DMA_Stream_TypeDef *DMAx_Channely)
{
    DMA_TypeDef *DMAx = LL_EX_DMA_Channel_to_DMA(DMAx_Channely);
    const uint32_t Channel = LL_EX_DMA_Channel_to_Channel(DMAx_Channely);

    WRITE_REG(DMAx->CHENREG, ((DMA_CHENREG_CH_EN_WE_0 << Channel) | (DMA_CHENREG_CH_EN_0 << Channel))); __NOP();
}

__STATIC_INLINE void LL_EX_DMA_DisableResource(DMA_Stream_TypeDef *DMAx_Channely)
{
    DMA_TypeDef *DMAx = LL_EX_DMA_Channel_to_DMA(DMAx_Channely);
    const uint32_t Channel = LL_EX_DMA_Channel_to_Channel(DMAx_Channely);

    WRITE_REG(DMAx->CHENREG, (DMA_CHENREG_CH_EN_WE_0 << Channel)); __NOP();
}

__STATIC_INLINE void LL_EX_DMA_EnableIT_TC(DMA_Stream_TypeDef *DMAx_Channely)
{
    DMA_TypeDef *DMAx = LL_EX_DMA_Channel_to_DMA(DMAx_Channely);
    const uint32_t Channel = LL_EX_DMA_Channel_to_Channel(DMAx_Channely);

    WRITE_REG(DMAx->MASKBLOCK,  (DMA_MASKBLOCK_MASKBLOCK_WE_0 << Channel) | (DMA_MASKBLOCK_MASKBLOCK_0 << Channel));
}

__STATIC_INLINE void LL_EX_DMA_SetDataLength(DMA_Stream_TypeDef* DMAx_Channely, uint32_t NbData)
{
    DMA_TypeDef *DMAx = LL_EX_DMA_Channel_to_DMA(DMAx_Channely);
    const uint32_t Channel = LL_EX_DMA_Channel_to_Channel(DMAx_Channely);

    DMAx->CHENREG = (DMA_CHENREG_CH_EN_WE_0 << Channel); __NOP();
    WRITE_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Channel])))->CTLH, NbData);
}

__STATIC_INLINE uint32_t LL_EX_DMA_GetDataLength(DMA_Stream_TypeDef* DMAx_Channely)
{
    DMA_TypeDef *DMAx = LL_EX_DMA_Channel_to_DMA(DMAx_Channely);
    const uint32_t Channel = LL_EX_DMA_Channel_to_Channel(DMAx_Channely);

    return READ_REG(((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Channel])))->CTLH);
}


__STATIC_INLINE void LL_EX_DMA_SetSrcAddress(DMA_Stream_TypeDef* DMAx_Channely, uint32_t SrcAddress)
{
    DMA_TypeDef *DMAx = LL_EX_DMA_Channel_to_DMA(DMAx_Channely);
    const uint32_t Channel = LL_EX_DMA_Channel_to_Channel(DMAx_Channely);

    DMAx->CHENREG = (DMA_CHENREG_CH_EN_WE_0 << Channel); __NOP();
    DMAx_Channely->SAR = SrcAddress;
}
