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

#include "stm32c5xx.h"
#include "common/utils.h"

/*
 * HAL2 LL DMA extension functions for Betaflight.
 *
 * HAL2 LL DMA API takes DMA_Channel_TypeDef* directly for per-channel
 * operations. There is no LL_DMA_Init/LL_DMA_InitTypeDef -- configuration
 * is done through individual register writes.
 */

// Compatibility defines for Betaflight DMA abstraction layer
#define LL_DMA_MODE_NORMAL   0x00000000U
#define LL_DMA_MODE_CIRCULAR 0x80000000U  // sentinel for LL_EX_DMA_ConfigStream

// Linked-list node for LPDMA circular DMA (linear addressing mode: 6 registers)
typedef struct {
    uint32_t reg[6];  // CTR1, CTR2, CBR1, CSAR, CDAR, CLLR
} dmaCircularNode_t;

// Forward declaration — LL_DMA_InitTypeDef is defined in the compat header
// which is included after the device headers. Use void* here to avoid
// header ordering issues; callers always pass LL_DMA_InitTypeDef*.
__STATIC_INLINE uint32_t LL_EX_DMA_Init(DMA_Channel_TypeDef *ch, void *initPtr)
{
    // HAL2 LPDMA has no LL_DMA_Init. Program channel registers directly
    // from the staging struct fields.
    typedef struct {
        uint32_t SrcAddress; uint32_t DestAddress; uint32_t Direction;
        uint32_t SrcIncMode; uint32_t DestIncMode;
        uint32_t SrcDataWidth; uint32_t DestDataWidth;
        uint32_t BlkDataLength; uint32_t Request; uint32_t Priority;
    } dmaInitCompat_t;
    const dmaInitCompat_t *init = (const dmaInitCompat_t *)initPtr;

    // CTR1: data widths + increment modes (constants are already register-field values)
    ch->CTR1 = init->SrcDataWidth | init->DestDataWidth
             | init->SrcIncMode | init->DestIncMode;

    // CTR2: peripheral request selection
    ch->CTR2 = (init->Request & DMA_CTR2_REQSEL);

    // CBR1: block data length in bytes
    ch->CBR1 = init->BlkDataLength & DMA_CBR1_BNDT;

    // Addresses
    ch->CSAR = init->SrcAddress;
    ch->CDAR = init->DestAddress;

    // CCR: priority (preserve other bits like EN, TCIE set earlier)
    STM32_MODIFY_REG(ch->CCR, DMA_CCR_PRIO, init->Priority);

    return 0;
}

__STATIC_INLINE uint32_t LL_EX_DMA_DeInit(DMA_Channel_TypeDef *DMAx_Channely)
{
    // Reset channel: disable and clear configuration
    DMAx_Channely->CCR = 0;
    DMAx_Channely->CTR1 = 0;
    DMAx_Channely->CTR2 = 0;
    DMAx_Channely->CBR1 = 0;
    DMAx_Channely->CSAR = 0;
    DMAx_Channely->CDAR = 0;
    DMAx_Channely->CLLR = 0;
    // Clear all flags
    DMAx_Channely->CFCR = 0x00003F00U;
    return 0;
}

__STATIC_INLINE void LL_EX_DMA_EnableResource(DMA_Channel_TypeDef *DMAx_Channely)
{
    SET_BIT(DMAx_Channely->CCR, DMA_CCR_EN);
}

__STATIC_INLINE void LL_EX_DMA_DisableResource(DMA_Channel_TypeDef *DMAx_Channely)
{
    SET_BIT(DMAx_Channely->CCR, DMA_CCR_SUSP | DMA_CCR_RESET);
}

__STATIC_INLINE void LL_EX_DMA_EnableIT_TC(DMA_Channel_TypeDef *DMAx_Channely)
{
    SET_BIT(DMAx_Channely->CCR, DMA_CCR_TCIE);
}

__STATIC_INLINE void LL_EX_DMA_SetMemoryAddress(DMA_Channel_TypeDef *DMAx_Channely, uint32_t addr)
{
    // HAL2: direction is in CTR2. Check SWREQ/DREQ bits to determine direction.
    uint32_t direction = LL_DMA_GetDataTransferDirection(DMAx_Channely);
    if (direction == LL_DMA_DIRECTION_MEMORY_TO_PERIPH) {
        LL_DMA_SetSrcAddress(DMAx_Channely, addr);
    } else {
        LL_DMA_SetDestAddress(DMAx_Channely, addr);
    }
}

__STATIC_INLINE void LL_EX_DMA_SetPeriphAddress(DMA_Channel_TypeDef *DMAx_Channely, uint32_t addr)
{
    uint32_t direction = LL_DMA_GetDataTransferDirection(DMAx_Channely);
    if (direction == LL_DMA_DIRECTION_MEMORY_TO_PERIPH) {
        LL_DMA_SetDestAddress(DMAx_Channely, addr);
    } else {
        LL_DMA_SetSrcAddress(DMAx_Channely, addr);
    }
}

__STATIC_INLINE void LL_EX_DMA_ConfigStream(DMA_Channel_TypeDef *ch,
    uint32_t request, uint32_t direction,
    uint32_t periphAddr, uint32_t memAddr,
    uint32_t dataLength, uint32_t mode)
{
    const bool circular = (mode == LL_DMA_MODE_CIRCULAR);

    // Reset channel
    LL_EX_DMA_DeInit(ch);

    // CTR1: data widths (byte), increments
    uint32_t ctr1 = 0;
    if (direction == LL_DMA_DIRECTION_PERIPH_TO_MEMORY) {
        ctr1 = DMA_CTR1_DINC;  // dest (memory) increments, src (periph) fixed
    } else {
        ctr1 = DMA_CTR1_SINC;  // src (memory) increments, dest (periph) fixed
    }
    ch->CTR1 = ctr1;

    // CTR2: request selection. HAL2 LPDMA has no direction bit -- src/dst
    // addresses determine direction implicitly.
    ch->CTR2 = (request & DMA_CTR2_REQSEL);

    // CBR1: block data length
    ch->CBR1 = dataLength & DMA_CBR1_BNDT;

    // Addresses
    if (direction == LL_DMA_DIRECTION_PERIPH_TO_MEMORY) {
        ch->CSAR = periphAddr;
        ch->CDAR = memAddr;
    } else {
        ch->CSAR = memAddr;
        ch->CDAR = periphAddr;
    }

    if (circular) {
        // LPDMA circular mode via linked-list self-linking
        static dmaCircularNode_t circularNodes[16] __attribute__((aligned(32)));

        // Determine node index from channel address
        uint32_t addr = (uint32_t)ch;
        uint32_t base = (addr >= LPDMA2_BASE) ? LPDMA2_BASE : LPDMA1_BASE;
        uint32_t chIdx = ((addr - base) - 0x50U) / 0x80U;
        uint32_t nodeIdx = ((addr >= LPDMA2_BASE) ? 8U : 0U) + chIdx;
        dmaCircularNode_t *node = &circularNodes[nodeIdx];

        // Copy configured channel registers into the linked-list node
        node->reg[0] = ch->CTR1;
        node->reg[1] = ch->CTR2;
        node->reg[2] = ch->CBR1;
        node->reg[3] = ch->CSAR;
        node->reg[4] = ch->CDAR;

        // Self-link: CLLR points back to this node
        const uint32_t nodeAddr = (uint32_t)node;
        const uint32_t updateFlags = DMA_CLLR_UT1 | DMA_CLLR_UT2 | DMA_CLLR_UB1
                                   | DMA_CLLR_USA | DMA_CLLR_UDA;
        node->reg[5] = (nodeAddr & DMA_CLLR_LA_Msk) | updateFlags;

        ch->CLBAR = nodeAddr & DMA_CLBAR_LBA;
        ch->CLLR  = (nodeAddr & DMA_CLLR_LA_Msk) | updateFlags;
    }
}

__STATIC_INLINE void LL_EX_DMA_SetDataLength(DMA_Channel_TypeDef *DMAx_Channely, uint32_t NbData)
{
    LL_DMA_SetBlkDataLength(DMAx_Channely, NbData);
}

__STATIC_INLINE uint32_t LL_EX_DMA_GetDataLength(DMA_Channel_TypeDef *DMAx_Channely)
{
    return LL_DMA_GetBlkDataLength(DMAx_Channely);
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
