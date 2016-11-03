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
    #include "build_config.h"

    #include "common/color.h"

    #include "drivers/light_ws2811strip.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

extern "C" {
STATIC_UNIT_TESTED extern uint16_t dmaBufferOffset;

STATIC_UNIT_TESTED void fastUpdateLEDDMABuffer(rgbColor24bpp_t *color);
STATIC_UNIT_TESTED void updateLEDDMABuffer(uint8_t componentValue);
}

TEST(WS2812, updateDMABuffer) {
    // given
    rgbColor24bpp_t color1 = { .raw = {0xFF,0xAA,0x55} };

    // and
    dmaBufferOffset = 0;

    // when
#if 0
    updateLEDDMABuffer(color1.rgb.g);
    updateLEDDMABuffer(color1.rgb.r);
    updateLEDDMABuffer(color1.rgb.b);
#else
    fastUpdateLEDDMABuffer(&color1);
#endif

    // then
    EXPECT_EQ(24, dmaBufferOffset);

    // and
    uint8_t byteIndex = 0;

    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 0]);
    EXPECT_EQ(BIT_COMPARE_0, ledStripDMABuffer[(byteIndex * 8) + 1]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 2]);
    EXPECT_EQ(BIT_COMPARE_0, ledStripDMABuffer[(byteIndex * 8) + 3]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 4]);
    EXPECT_EQ(BIT_COMPARE_0, ledStripDMABuffer[(byteIndex * 8) + 5]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 6]);
    EXPECT_EQ(BIT_COMPARE_0, ledStripDMABuffer[(byteIndex * 8) + 7]);
    byteIndex++;

    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 0]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 1]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 2]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 3]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 4]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 5]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 6]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 7]);
    byteIndex++;

    EXPECT_EQ(BIT_COMPARE_0, ledStripDMABuffer[(byteIndex * 8) + 0]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 1]);
    EXPECT_EQ(BIT_COMPARE_0, ledStripDMABuffer[(byteIndex * 8) + 2]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 3]);
    EXPECT_EQ(BIT_COMPARE_0, ledStripDMABuffer[(byteIndex * 8) + 4]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 5]);
    EXPECT_EQ(BIT_COMPARE_0, ledStripDMABuffer[(byteIndex * 8) + 6]);
    EXPECT_EQ(BIT_COMPARE_1, ledStripDMABuffer[(byteIndex * 8) + 7]);
    byteIndex++;
}

extern "C" {
rgbColor24bpp_t* hsvToRgb24(const hsvColor_t *c) {
    UNUSED(c);
    return NULL;
}

void ws2811LedStripHardwareInit(void) {}
void ws2811LedStripDMAEnable(void) {}

void dmaSetHandler(dmaHandlerIdentifier_e, dmaCallbackHandlerFuncPtr ) {}

uint8_t DMA_GetFlagStatus(uint32_t) { return 0; }
void DMA_Cmd(DMA_Channel_TypeDef*, FunctionalState ) {}
void DMA_ClearFlag(uint32_t) {}

}
