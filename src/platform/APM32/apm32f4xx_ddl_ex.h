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

#include "apm32f4xx.h"
#include "common/utils.h"

// XXX Followings are straight copy of stm32f7xx_ll_ex.h.
// XXX Consider consolidation when LL-DShot is stable.

__STATIC_INLINE void DDL_EX_TMR_DisableIT(TMR_TypeDef *TMRx, uint32_t Sources)
{
    CLEAR_BIT(TMRx->DIEN, Sources);
}

__STATIC_INLINE void DDL_EX_TMR_EnableIT(TMR_TypeDef *TMRx, uint32_t Sources)
{
    SET_BIT(TMRx->DIEN, Sources);
}

__STATIC_INLINE uint32_t DDL_EX_DMA_Stream_to_Stream(DMA_Stream_TypeDef *DMAx_Streamy)
{
    STATIC_ASSERT(DMA1_Stream0_BASE - DMA1_BASE == sizeof(DMA_TypeDef), DMA_TypeDef_has_padding);
    STATIC_ASSERT(DMA1_Stream1_BASE - DMA1_Stream0_BASE == sizeof(DMA_Stream_TypeDef), DMA_Stream_TypeDef_has_padding);

    const size_t firstStreamOffset = sizeof(DMA_TypeDef);
    const size_t streamSize = sizeof(DMA_Stream_TypeDef);

    return (((uint32_t) DMAx_Streamy & 0XFF) - firstStreamOffset) / streamSize;
}

__STATIC_INLINE DMA_TypeDef *DDL_EX_DMA_Stream_to_DMA(DMA_Stream_TypeDef *DMAx_Streamy)
{
    // clear stream address
    return (DMA_TypeDef *) (((uint32_t) DMAx_Streamy) & ((uint32_t) ~0XFF));
}

__STATIC_INLINE uint32_t DDL_EX_DMA_DeInit(DMA_Stream_TypeDef *DMAx_Streamy)
{
    DMA_TypeDef *DMA = DDL_EX_DMA_Stream_to_DMA(DMAx_Streamy);
    const uint32_t Stream = DDL_EX_DMA_Stream_to_Stream(DMAx_Streamy);

    return DDL_DMA_DeInit(DMA, Stream);
}

__STATIC_INLINE uint32_t DDL_EX_DMA_Init(DMA_Stream_TypeDef *DMAx_Streamy, DDL_DMA_InitTypeDef *DMA_InitStruct)
{
    DMA_TypeDef *DMA = DDL_EX_DMA_Stream_to_DMA(DMAx_Streamy);
    const uint32_t Stream = DDL_EX_DMA_Stream_to_Stream(DMAx_Streamy);

    return DDL_DMA_Init(DMA, Stream, DMA_InitStruct);
}

__STATIC_INLINE void DDL_EX_DMA_SetDataLength(DMA_Stream_TypeDef* DMAx_Streamy, uint32_t NbData)
{
    MODIFY_REG(DMAx_Streamy->NDATA, DMA_NDATAx, NbData);
}

__STATIC_INLINE uint32_t DDL_EX_DMA_GetDataLength(DMA_Stream_TypeDef* DMAx_Streamy)
{
    DMA_TypeDef *DMA = DDL_EX_DMA_Stream_to_DMA(DMAx_Streamy);
    const uint32_t Stream = DDL_EX_DMA_Stream_to_Stream(DMAx_Streamy);

    return DDL_DMA_GetDataLength(DMA, Stream);
}

__STATIC_INLINE void DDL_EX_DMA_EnableResource(DMA_Stream_TypeDef *DMAx_Streamy)
{
    SET_BIT(DMAx_Streamy->SCFG, DMA_SCFGx_EN);
}

__STATIC_INLINE void DDL_EX_DMA_DisableResource(DMA_Stream_TypeDef *DMAx_Streamy)
{
    CLEAR_BIT(DMAx_Streamy->SCFG, DMA_SCFGx_EN);
}

__STATIC_INLINE void DDL_EX_DMA_EnableIT_TC(DMA_Stream_TypeDef *DMAx_Streamy)
{
    SET_BIT(DMAx_Streamy->SCFG, DMA_SCFGx_TXCIEN);
}

__STATIC_INLINE void DDL_EX_TMR_CC_EnableNChannel(TMR_TypeDef *TMRx, uint32_t Channel)
{
    DDL_TMR_CC_EnableChannel(TMRx, 4 * Channel);
}
