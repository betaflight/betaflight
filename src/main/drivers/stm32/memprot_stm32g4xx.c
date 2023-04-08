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

#include "platform.h"

#include "drivers/memprot.h"

// Defined in linker script
extern uint8_t dma_ram_r_start;
extern uint8_t dma_ram_r_end;

extern uint8_t dma_ram_w_start;
extern uint8_t dma_ram_w_end;

extern uint8_t dma_ram_rw_start;
extern uint8_t dma_ram_rw_end;

mpuRegion_t mpuRegions[] = {
    {
        // DMA_RAM_R
        // DMA receive buffer in SRAM
        .start      = (uint32_t)&dma_ram_r_start,
        .end        = (uint32_t)&dma_ram_r_end,
        .size       = 0,  // Size determined by ".end"
        .perm       = MPU_REGION_FULL_ACCESS,
        .exec       = MPU_INSTRUCTION_ACCESS_ENABLE,
        .shareable  = MPU_ACCESS_SHAREABLE,
        .cacheable  = MPU_ACCESS_NOT_CACHEABLE,
        .bufferable = MPU_ACCESS_BUFFERABLE,
    },
    {
        // DMA_RAM_W
        // DMA transmit buffer in SRAM
        .start      = (uint32_t)&dma_ram_w_start,
        .end        = (uint32_t)&dma_ram_w_end,
        .size       = 0,  // Size determined by ".end"
        .perm       = MPU_REGION_FULL_ACCESS,
        .exec       = MPU_INSTRUCTION_ACCESS_ENABLE,
        .shareable  = MPU_ACCESS_SHAREABLE,
        .cacheable  = MPU_ACCESS_CACHEABLE,
        .bufferable = MPU_ACCESS_NOT_BUFFERABLE,
    },
    {
        // DMA_RAM_RW
        // DMA transmit and receive buffer in SRAM
        .start      = (uint32_t)&dma_ram_rw_start,
        .end        = (uint32_t)&dma_ram_rw_end,
        .size       = 0,  // Size determined by ".end"
        .perm       = MPU_REGION_FULL_ACCESS,
        .exec       = MPU_INSTRUCTION_ACCESS_ENABLE,
        .shareable  = MPU_ACCESS_SHAREABLE,
        .cacheable  = MPU_ACCESS_NOT_CACHEABLE,
        .bufferable = MPU_ACCESS_NOT_BUFFERABLE,
    },
};

unsigned mpuRegionCount = ARRAYLEN(mpuRegions);
