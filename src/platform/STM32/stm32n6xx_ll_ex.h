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

#include <stdbool.h>

#include "stm32n6xx.h"
#include "common/utils.h"

// N6 uses HPDMA1/GPDMA1 with channel-based DMA (not stream-based like H7).
// The LL_EX functions provide a simplified interface for Betaflight's DMA abstraction.

// Compatibility defines: older STM32 families use LL_DMA_MODE_NORMAL / LL_DMA_MODE_CIRCULAR.
// N6 GPDMA has no register-based circular mode; circular is achieved via linked-list self-linking.
#define LL_DMA_MODE_NORMAL   LL_DMA_NORMAL
#define LL_DMA_MODE_CIRCULAR 0x80000000U  // sentinel value handled by LL_EX_DMA_ConfigStream

// Linked-list node for GPDMA circular DMA (linear addressing mode: 6 registers)
typedef struct {
    uint32_t reg[6];  // CTR1, CTR2, CBR1, CSAR, CDAR, CLLR
} dmaCircularNode_t;

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

__STATIC_INLINE void LL_EX_DMA_SetMemoryAddress(DMA_Channel_TypeDef *DMAx_Channely, uint32_t addr)
{
    DMA_TypeDef *DMA = LL_EX_DMA_Channel_to_DMA(DMAx_Channely);
    const uint32_t Channel = LL_EX_DMA_Channel_to_Channel(DMAx_Channely);
    // GPDMA has separate src/dest registers; select based on configured direction.
    if (LL_DMA_GetDataTransferDirection(DMA, Channel) == LL_DMA_DIRECTION_MEMORY_TO_PERIPH) {
        LL_DMA_SetSrcAddress(DMA, Channel, addr);
    } else {
        LL_DMA_SetDestAddress(DMA, Channel, addr);
    }
}

__STATIC_INLINE void LL_EX_DMA_SetPeriphAddress(DMA_Channel_TypeDef *DMAx_Channely, uint32_t addr)
{
    DMA_TypeDef *DMA = LL_EX_DMA_Channel_to_DMA(DMAx_Channely);
    const uint32_t Channel = LL_EX_DMA_Channel_to_Channel(DMAx_Channely);
    if (LL_DMA_GetDataTransferDirection(DMA, Channel) == LL_DMA_DIRECTION_MEMORY_TO_PERIPH) {
        LL_DMA_SetDestAddress(DMA, Channel, addr);
    } else {
        LL_DMA_SetSrcAddress(DMA, Channel, addr);
    }
}

__STATIC_INLINE void LL_EX_DMA_ConfigStream(DMA_Channel_TypeDef *DMAx_Channely,
    uint32_t request, uint32_t direction,
    uint32_t periphAddr, uint32_t memAddr,
    uint32_t dataLength, uint32_t mode)
{
    DMA_TypeDef *DMA = LL_EX_DMA_Channel_to_DMA(DMAx_Channely);
    const uint32_t Channel = LL_EX_DMA_Channel_to_Channel(DMAx_Channely);
    const bool circular = (mode == LL_DMA_MODE_CIRCULAR);
    LL_DMA_InitTypeDef init = { 0 };

    init.Request = request;
    init.BlkHWRequest = LL_DMA_HWREQUEST_SINGLEBURST;
    init.DataAlignment = LL_DMA_DATA_ALIGN_ZEROPADD;
    init.SrcBurstLength = 1;
    init.DestBurstLength = 1;
    init.SrcDataWidth = LL_DMA_SRC_DATAWIDTH_BYTE;
    init.DestDataWidth = LL_DMA_DEST_DATAWIDTH_BYTE;
    init.Priority = LL_DMA_LOW_PRIORITY_LOW_WEIGHT;
    init.BlkDataLength = dataLength;
    init.Mode = LL_DMA_NORMAL;

    if (direction == LL_DMA_DIRECTION_PERIPH_TO_MEMORY) {
        init.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
        init.SrcAddress = periphAddr;
        init.DestAddress = memAddr;
        init.SrcIncMode = LL_DMA_SRC_FIXED;
        init.DestIncMode = LL_DMA_DEST_INCREMENT;
    } else {
        init.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
        init.SrcAddress = memAddr;
        init.DestAddress = periphAddr;
        init.SrcIncMode = LL_DMA_SRC_INCREMENT;
        init.DestIncMode = LL_DMA_DEST_FIXED;
    }

    LL_DMA_DeInit(DMA, Channel);
    LL_DMA_Init(DMA, Channel, &init);

    if (circular) {
        // GPDMA circular mode via linked-list self-linking:
        // Create a node in SRAM that the DMA reloads after each block transfer,
        // with the node's CLLR pointing back to itself.
        static dmaCircularNode_t circularNodes[32] __attribute__((aligned(32)));
        const uint32_t isGPDMA = ((uint32_t)DMAx_Channely >= GPDMA1_BASE) ? 16U : 0U;
        dmaCircularNode_t *node = &circularNodes[isGPDMA + Channel];

        // Copy configured channel registers into the linked-list node
        node->reg[0] = DMAx_Channely->CTR1;
        node->reg[1] = DMAx_Channely->CTR2;
        node->reg[2] = DMAx_Channely->CBR1;
        node->reg[3] = DMAx_Channely->CSAR;
        node->reg[4] = DMAx_Channely->CDAR;

        // Self-link: CLLR points back to this node, reloading all transfer registers
        const uint32_t nodeAddr = (uint32_t)node;
        const uint32_t updateFlags = DMA_CLLR_UT1 | DMA_CLLR_UT2 | DMA_CLLR_UB1
                                   | DMA_CLLR_USA | DMA_CLLR_UDA;
        node->reg[5] = (nodeAddr & DMA_CLLR_LA_Msk) | updateFlags;

        // Program the channel's linked-list base address and link register
        DMAx_Channely->CLBAR = nodeAddr & DMA_CLBAR_LBA;
        DMAx_Channely->CLLR  = (nodeAddr & DMA_CLLR_LA_Msk) | updateFlags;
    }
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
