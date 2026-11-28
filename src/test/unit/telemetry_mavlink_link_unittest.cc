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

// Compiles the telemetry_mavlink_unittest.cc tests with the mission protocol, and adds the link-up ones.
#include "telemetry_mavlink_unittest.cc"

extern "C" {
    static int missionInits;
    void mavMissionInit(void) { missionInits++; }
    bool mavMissionHandleMessage(const mavlink_message_t *) { return false; }
    void mavMissionUpdate(timeMs_t) {}
    static int missionSaves;
    static bool missionSavedBetweenAckAndReset;
    void mavMissionSaveIfDisarmed(void)
    {
        missionSaves++;
        if (waitForTxCalls == 1 && systemResetCalls == 0) {
            missionSavedBetweenAckAndReset = true;
        }
    }
}

class MavlinkLinkUpTest : public MavlinkTelemetryTest {
protected:
    void TearDown() override
    {
        rxPortShared = false;
        telemetrySharedPort = nullptr;
    }

    void linkUp()
    {
        freeMAVLinkTelemetryPort();
        initMAVLinkTelemetry();
        missionInits = 0;
        checkMAVLinkTelemetryState();
    }
};

TEST_F(MavlinkLinkUpTest, OwnPortStartsTheMissionProtocol)
{
    linkUp();
    EXPECT_EQ(missionInits, 1);
}

TEST_F(MavlinkLinkUpTest, SharedReceiverPortStartsTheMissionProtocol)
{
    rxPortShared = true;
    telemetrySharedPort = &testPort;
    linkUp();
    EXPECT_EQ(missionInits, 1);
}

TEST_F(MavlinkLinkUpTest, PendingMissionIsSavedWithTheLinkDown)
{
    freeMAVLinkTelemetryPort();
    missionSaves = 0;
    handleMAVLinkTelemetry();
    EXPECT_EQ(missionSaves, 1);
}

TEST_F(MavlinkLinkUpTest, PendingMissionIsSavedBeforeReboot)
{
    missionSavedBetweenAckAndReset = false;
    EXPECT_EQ(ackResultFor(MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1), MAV_RESULT_ACCEPTED);
    EXPECT_EQ(systemResetCalls, 1);
    EXPECT_TRUE(missionSavedBetweenAckAndReset);
}
