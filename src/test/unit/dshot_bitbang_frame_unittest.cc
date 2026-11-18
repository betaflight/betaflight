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

#include <stdint.h>
#include <stdbool.h>

extern "C" {
#include "drivers/dshot_bitbang_frame.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// A frame is 16 bits of 3 states plus 3 hold states, paced at 3 states per bit
static const uint32_t frameSlots = 16 * 3 + 3;

static uint32_t slotRateHz(uint32_t dshotKbit)
{
    return dshotKbit * 1000 * 3;
}

TEST(DshotBitbangFrameUnittest, TimeoutIsTwiceTheFrame)
{
    // 51 slots at 1.8 MHz is 28.3 us
    EXPECT_EQ(56, bbOutputFrameTimeoutUs(frameSlots, slotRateHz(600)));
    EXPECT_EQ(112, bbOutputFrameTimeoutUs(frameSlots, slotRateHz(300)));
    EXPECT_EQ(226, bbOutputFrameTimeoutUs(frameSlots, slotRateHz(150)));
}

TEST(DshotBitbangFrameUnittest, TimeoutLeavesTheNextLoopRoomToFinishACapture)
{
    // A stalled frame must be caught at the next update, so the limit has to be
    // well inside the shortest loop the protocol is run at: 8 kHz for DShot600,
    // 4 kHz for DShot300.
    EXPECT_LT(bbOutputFrameTimeoutUs(frameSlots, slotRateHz(600)), 125);
    EXPECT_LT(bbOutputFrameTimeoutUs(frameSlots, slotRateHz(300)), 250);
}

TEST(DshotBitbangFrameUnittest, FrameThatCompletedIsNotStalled)
{
    const timeDelta_t timeoutUs = bbOutputFrameTimeoutUs(frameSlots, slotRateHz(600));

    // The ISR clears outputPending on completion, however long ago the frame went out
    EXPECT_FALSE(bbOutputFrameStalled(false, 1000000, 1000, timeoutUs));
}

TEST(DshotBitbangFrameUnittest, FrameStillInTimeIsNotStalled)
{
    const timeDelta_t timeoutUs = bbOutputFrameTimeoutUs(frameSlots, slotRateHz(600));
    const timeUs_t sentUs = 5000;

    EXPECT_FALSE(bbOutputFrameStalled(true, sentUs, sentUs, timeoutUs));
    EXPECT_FALSE(bbOutputFrameStalled(true, sentUs + 30, sentUs, timeoutUs));
    EXPECT_FALSE(bbOutputFrameStalled(true, sentUs + timeoutUs, sentUs, timeoutUs));
}

TEST(DshotBitbangFrameUnittest, FramePastTheLimitIsStalled)
{
    const timeDelta_t timeoutUs = bbOutputFrameTimeoutUs(frameSlots, slotRateHz(600));
    const timeUs_t sentUs = 5000;

    EXPECT_TRUE(bbOutputFrameStalled(true, sentUs + timeoutUs + 1, sentUs, timeoutUs));
    // One 8 kHz loop later, where bbUpdateComplete() next looks
    EXPECT_TRUE(bbOutputFrameStalled(true, sentUs + 125, sentUs, timeoutUs));
}

TEST(DshotBitbangFrameUnittest, StallIsDetectedAcrossMicrosWrap)
{
    const timeDelta_t timeoutUs = bbOutputFrameTimeoutUs(frameSlots, slotRateHz(600));
    const timeUs_t sentUs = UINT32_MAX - 20;

    EXPECT_FALSE(bbOutputFrameStalled(true, sentUs + 30, sentUs, timeoutUs));
    EXPECT_TRUE(bbOutputFrameStalled(true, sentUs + 125, sentUs, timeoutUs));
}
