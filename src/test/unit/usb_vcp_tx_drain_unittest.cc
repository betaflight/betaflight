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
    #include "platform.h"

    #include "drivers/serial_usb_vcp.h"

    extern timeMs_t fakeMillis;
}

#include "gtest/gtest.h"

static const timeMs_t TIMEOUT_MS = 50;
static const timeMs_t POLL_MS = 10;

class UsbVcpTxDrainTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        fakeMillis = 1000;
        drain = {};
    }

    // Poll with a fixed pending count until drained, returning the elapsed time.
    timeMs_t pollUntilDrained(uint32_t pending)
    {
        const timeMs_t start = fakeMillis;
        while (!usbVcpTxDrained(&drain, pending, TIMEOUT_MS)) {
            fakeMillis += POLL_MS;
            if (fakeMillis - start > 10 * TIMEOUT_MS) {
                break;
            }
        }
        return fakeMillis - start;
    }

    usbVcpTxDrain_t drain;
};

TEST_F(UsbVcpTxDrainTest, EmptyIsDrainedImmediately)
{
    EXPECT_TRUE(usbVcpTxDrained(&drain, 0, TIMEOUT_MS));
}

TEST_F(UsbVcpTxDrainTest, StalledCountTimesOut)
{
    const timeMs_t elapsed = pollUntilDrained(100);
    EXPECT_GT(elapsed, TIMEOUT_MS);
    EXPECT_LE(elapsed, TIMEOUT_MS + POLL_MS);
}

TEST_F(UsbVcpTxDrainTest, ProgressRestartsTimeout)
{
    for (int i = 0; i < 4; i++) {
        EXPECT_FALSE(usbVcpTxDrained(&drain, 100, TIMEOUT_MS));
        fakeMillis += POLL_MS;
    }
    // Count changes: a fresh full interval is needed from here.
    EXPECT_GT(pollUntilDrained(60), TIMEOUT_MS);
}

TEST_F(UsbVcpTxDrainTest, LongGapBetweenPollsRestartsTimeout)
{
    EXPECT_FALSE(usbVcpTxDrained(&drain, 100, TIMEOUT_MS));
    fakeMillis += 5 * TIMEOUT_MS;
    EXPECT_FALSE(usbVcpTxDrained(&drain, 100, TIMEOUT_MS));
}

// After a timeout, a new wait whose count happens to match the one that timed out
// (the host finished that transfer and an equal-sized write was queued) must not
// report drained immediately.
TEST_F(UsbVcpTxDrainTest, TimeoutDoesNotCarryOverToNextWait)
{
    pollUntilDrained(23);
    fakeMillis += 1;
    EXPECT_FALSE(usbVcpTxDrained(&drain, 23, TIMEOUT_MS));
    EXPECT_GT(pollUntilDrained(23), TIMEOUT_MS - POLL_MS);
}
