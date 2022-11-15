/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdlib.h>
#include <limits.h>

extern "C" {
    #include <platform.h>
    #include "build/build_config.h"
    #include "drivers/dma.h"
    #include "drivers/transponder_ir.h"
    #include "drivers/transponder_ir_arcitimer.h"
    #include "drivers/transponder_ir_ilap.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

extern "C" {
    STATIC_UNIT_TESTED extern uint16_t dmaBufferOffset;
    STATIC_UNIT_TESTED void updateTransponderDMABufferIlap(transponder_t *transponder, const uint8_t* transponderData);
    STATIC_UNIT_TESTED void updateTransponderDMABufferArcitimer(transponder_t *transponder, const uint8_t* transponderData);
}

TEST(transponderTest, updateTransponderDMABufferArcitimer)
{
    //input
    uint8_t data[9] = {0x1F, 0xFC, 0x8F, 0x3, 0xF0, 0x1, 0xF8, 0x1F, 0x0};
    //excepted
    uint8_t excepted[TRANSPONDER_DMA_BUFFER_SIZE_ARCITIMER] = {
        78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,78,78,78,78,
        78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,0,0,
        0,0,0,0,0,0,0,0,0,0,78,78,78,78,78,78,78,78,78,78,78,78,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,
        78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0
    };
    uint8_t* transponderData = data;
    transponder_t transponder;
    transponder.dma_buffer_size = TRANSPONDER_DMA_BUFFER_SIZE_ARCITIMER;
    transponder.bitToggleOne = 78;
    memset(&(transponder.transponderIrDMABuffer.arcitimer), 0, TRANSPONDER_DMA_BUFFER_SIZE_ARCITIMER);

    updateTransponderDMABufferArcitimer(&transponder, transponderData);
    uint16_t i;
    for(i = 0; i < transponder.dma_buffer_size; i++) {
        EXPECT_EQ(transponder.transponderIrDMABuffer.arcitimer[i], excepted[i]);
    }
}

TEST(transponderTest, updateTransponderDMABufferIlap)
{
    uint8_t data[9] = {0x1F, 0xFC, 0x8F, 0x3, 0xF0, 0x1, 0x0, 0x0, 0x0};

    uint8_t excepted[TRANSPONDER_DMA_BUFFER_SIZE_ILAP] = {
           78,78,78,78,78,78,78,78,78,78,78,0,78,78,78,78,78,78,78,78,78,78,78,0,78,78,78,78,78,78,78,78,78,78,78,0,78,
           78,78,78,78,78,78,78,78,78,78,0,78,78,78,78,78,78,78,78,78,78,78,0,78,78,78,78,78,78,78,78,78,78,78,0,0,0,0,
           0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,78,78,78,78,78,78,
           78,78,78,78,78,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,78,78,78,78,78,78,78,78,78,78,78,0,78,78,78,
           78,78,78,78,78,78,78,78,0,78,78,78,78,78,78,78,78,78,78,78,0,78,78,78,78,78,78,78,78,78,78,78,0,78,78,78,78,
           78,78,78,78,78,78,78,0,78,78,78,78,78,78,78,78,78,78,78,0,0,0,0,0,0,0,0,0,0,0,0,0,78,78,78,78,78,78,78,78,78,
           78,78,0,78,78,78,78,78,78,78,78,78,78,78,0,78,78,78,78,78,78,78,78,78,78,78,0,78,78,78,78,78,78,78,78,78,78,
           78,0,78,78,78,78,78,78,78,78,78,78,78,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
           0,0,78,78,78,78,78,78,78,78,78,78,78,0,0,0,0,0,0,0,0,0,0,0,0,0,78,78,78,78,78,78,78,78,78,78,78,0,78,78,78,
           78,78,78,78,78,78,78,78,0,78,78,78,78,78,78,78,78,78,78,78,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,78,78,78,78,78,78,78,78,78,78,78,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,78,78,78,78,78,78,78,78,78,78,78,0,78,78,78,78,78,78,78,78,78,78,78,
           0,78,78,78,78,78,78,78,78,78,78,78,0,78,78,78,78,78,78,78,78,78,78,78,0,0,0,0,0,0,0,0,0,0,0,0,0,78,78,78,78,
           78,78,78,78,78,78,78,0,78,78,78,78,78,78,78,78,78,78,78,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
        };

    uint8_t* transponderData = data;
    transponder_t transponder;
    transponder.dma_buffer_size = TRANSPONDER_DMA_BUFFER_SIZE_ILAP;
    transponder.bitToggleOne = 78;

    updateTransponderDMABufferIlap(&transponder, transponderData);

     uint16_t i;
     for(i = 0; i < transponder.dma_buffer_size; i++) {
        EXPECT_EQ(transponder.transponderIrDMABuffer.ilap[i], excepted[i]);
     }
}
