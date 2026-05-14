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

#include <stdint.h>
#include <stdbool.h>

extern "C" {
    #include "platform.h"

    #include "pg/rx.h"
    #include "rx/rx.h"
    #include "rx/msp.h"
}

#include "gtest/gtest.h"

static uint32_t simulationMillis = 0;

extern "C" {
    uint32_t millis(void) { return simulationMillis; }
    uint32_t micros(void) { return simulationMillis * 1000; }
}

// rxMspIsRcChannelFresh returns true only when the most recent MSP_SET_RAW_RC
// frame both (a) included the requested channel and (b) arrived within the
// freshness window. Without these guards an empty or short MSP frame would
// allow rxMspOverrideReadRawRc() to clamp masked channels to rx_min_usec.
TEST(MspOverrideTest, ChannelFreshness)
{
    // Before any frame is received, every channel is stale.
    simulationMillis = 1000;
    EXPECT_FALSE(rxMspIsRcChannelFresh(0));
    EXPECT_FALSE(rxMspIsRcChannelFresh(3));
    EXPECT_FALSE(rxMspIsRcChannelFresh(8));

    // A 4-channel (AETR-only) frame makes channels 0..3 fresh; AUX channels
    // beyond the frame's channelCount remain stale even though the frame
    // itself is current.
    uint16_t frame4[MAX_SUPPORTED_RC_CHANNEL_COUNT] = {1500, 1500, 1500, 1000};
    rxMspFrameReceive(frame4, 4);
    EXPECT_TRUE(rxMspIsRcChannelFresh(0));
    EXPECT_TRUE(rxMspIsRcChannelFresh(3));
    EXPECT_FALSE(rxMspIsRcChannelFresh(4));
    EXPECT_FALSE(rxMspIsRcChannelFresh(8));

    // Inside the freshness window: still fresh.
    simulationMillis = 1000 + 250;
    EXPECT_TRUE(rxMspIsRcChannelFresh(0));
    EXPECT_FALSE(rxMspIsRcChannelFresh(4));

    // A 6-channel frame extends freshness to AUX2 (channel 5).
    simulationMillis = 1500;
    uint16_t frame6[MAX_SUPPORTED_RC_CHANNEL_COUNT] = {1500, 1500, 1500, 1000, 2000, 1750};
    rxMspFrameReceive(frame6, 6);
    EXPECT_TRUE(rxMspIsRcChannelFresh(0));
    EXPECT_TRUE(rxMspIsRcChannelFresh(5));
    EXPECT_FALSE(rxMspIsRcChannelFresh(6));

    // Past the freshness window every channel becomes stale, regardless of
    // whether earlier frames had covered it.
    simulationMillis = 1500 + 301;
    EXPECT_FALSE(rxMspIsRcChannelFresh(0));
    EXPECT_FALSE(rxMspIsRcChannelFresh(5));

    // A subsequent shorter frame shrinks coverage: a previously-fresh AUX
    // channel must drop back to stale even while channels 0..3 stay fresh.
    simulationMillis = 2000;
    rxMspFrameReceive(frame4, 4);
    EXPECT_TRUE(rxMspIsRcChannelFresh(3));
    EXPECT_FALSE(rxMspIsRcChannelFresh(5));
}
