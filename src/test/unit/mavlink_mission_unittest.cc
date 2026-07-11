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

// Coverage for the MAVLink MISSION microservice extensions: geofence
// upload/download (MAV_MISSION_TYPE_FENCE), MISSION_SET_CURRENT, and a
// regression that the MISSION_TYPE_MISSION waypoint path still round-trips after
// the transfer state machine was made mission-type-aware.

#include <stdint.h>
#include <string.h>

#include <vector>

extern "C" {
    #include "platform.h"

    #include "common/mavlink.h"

    #include "config/config.h"
    #include "fc/runtime_config.h"
    #include "flight/flight_plan_nav.h"
    #include "io/gps.h"
    #include "pg/flight_plan.h"
    #include "telemetry/mavlink.h"
    #include "telemetry/mavlink_mission.h"

    // --- runtime_config backing globals (STATE / FLIGHT_MODE) ---
    uint8_t stateFlags;
    uint16_t flightModeFlags;
    gpsLocation_t GPS_home_llh;

    // --- controllable time ---
    static uint32_t fakeMillis = 1000;
    uint32_t millis(void) { return fakeMillis; }

    // --- config save observed but inert ---
    static int saveCount;
    void saveConfigAndNotify(void) { saveCount++; }

    // --- flight-plan nav stubs ---
    static bool     navActive;
    static uint8_t  navCurrentIndex;
    static bool     navSetResult;          // value returned by SetCurrentIndex
    static int      navSetIndexArg;        // arg captured (-1 if not called)
    bool flightPlanNavIsActive(void) { return navActive; }
    uint8_t flightPlanNavGetCurrentIndex(void) { return navCurrentIndex; }
    flightPlanNavState_e flightPlanNavGetState(void) { return FP_NAV_TARGETING; }
    void flightPlanNavSetReachedListener(flightPlanWaypointReachedFn) {}
    bool flightPlanNavSetCurrentIndex(uint8_t index) {
        navSetIndexArg = index;
        return navSetResult;
    }

    // --- TX capture ---
    static std::vector<mavlink_message_t> *g_sent;
    void mavlinkSendMessage(mavlink_message_t *msg) { g_sent->push_back(*msg); }
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

#define GCS_SYS 255
#define GCS_COMP MAV_COMP_ID_MISSIONPLANNER
#define FC_SYS 1
#define FC_COMP MAV_COMP_ID_AUTOPILOT1

class MavMissionTest : public ::testing::Test {
protected:
    std::vector<mavlink_message_t> sent;

    void SetUp() override {
        g_sent = &sent;
        sent.clear();
        saveCount = 0;
        fakeMillis = 1000;
        stateFlags = 0;
        flightModeFlags = 0;
        memset(&GPS_home_llh, 0, sizeof(GPS_home_llh));

        navActive = false;
        navCurrentIndex = 0;
        navSetResult = true;
        navSetIndexArg = -1;

        memset(flightPlanConfigMutable(), 0, sizeof(flightPlanConfig_t));

        mavMissionInit();
    }

    void feed(const mavlink_message_t *msg) { mavMissionHandleMessage(msg); }

    // Returns the last sent message of a given id, or nullptr.
    const mavlink_message_t *lastOf(uint32_t msgid) {
        for (auto it = sent.rbegin(); it != sent.rend(); ++it) {
            if (it->msgid == msgid) {
                return &(*it);
            }
        }
        return nullptr;
    }

    void sendCount(uint16_t count, uint8_t type) {
        mavlink_message_t m;
        mavlink_msg_mission_count_pack(GCS_SYS, GCS_COMP, &m, FC_SYS, FC_COMP, count, type, 0);
        feed(&m);
    }

    void sendItem(uint16_t seq, uint16_t command, uint8_t type, int32_t x, int32_t y, float param1 = 0.0f) {
        mavlink_message_t m;
        mavlink_msg_mission_item_int_pack(GCS_SYS, GCS_COMP, &m, FC_SYS, FC_COMP,
            seq, MAV_FRAME_GLOBAL_INT, command, 0, 1,
            param1, 0.0f, 0.0f, 0.0f, x, y, 0.0f, type);
        feed(&m);
    }

    void sendRequestList(uint8_t type) {
        mavlink_message_t m;
        mavlink_msg_mission_request_list_pack(GCS_SYS, GCS_COMP, &m, FC_SYS, FC_COMP, type);
        feed(&m);
    }

    void sendRequestInt(uint16_t seq, uint8_t type) {
        mavlink_message_t m;
        mavlink_msg_mission_request_int_pack(GCS_SYS, GCS_COMP, &m, FC_SYS, FC_COMP, seq, type);
        feed(&m);
    }

    uint8_t lastAckResult() {
        const mavlink_message_t *m = lastOf(MAVLINK_MSG_ID_MISSION_ACK);
        EXPECT_NE(m, nullptr);
        mavlink_mission_ack_t a;
        mavlink_msg_mission_ack_decode(m, &a);
        return a.type;
    }
};

// --- Geofence upload + download round-trip ---
TEST_F(MavMissionTest, FenceUploadThenDownloadRoundTrips)
{
    sendCount(2, MAV_MISSION_TYPE_FENCE);
    // FC should request item 0 of the fence.
    const mavlink_message_t *req = lastOf(MAVLINK_MSG_ID_MISSION_REQUEST_INT);
    ASSERT_NE(req, nullptr);
    mavlink_mission_request_int_t r;
    mavlink_msg_mission_request_int_decode(req, &r);
    EXPECT_EQ(r.seq, 0);
    EXPECT_EQ(r.mission_type, MAV_MISSION_TYPE_FENCE);

    sendItem(0, MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, MAV_MISSION_TYPE_FENCE, 100, 200, 4.0f);
    sendItem(1, MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION, MAV_MISSION_TYPE_FENCE, 300, 400, 25.0f);

    EXPECT_EQ(lastAckResult(), MAV_MISSION_ACCEPTED);
    EXPECT_EQ(saveCount, 0); // fence is RAM-only; must not thrash flash

    // Download: request the fence back.
    sent.clear();
    sendRequestList(MAV_MISSION_TYPE_FENCE);
    const mavlink_message_t *cm = lastOf(MAVLINK_MSG_ID_MISSION_COUNT);
    ASSERT_NE(cm, nullptr);
    mavlink_mission_count_t mc;
    mavlink_msg_mission_count_decode(cm, &mc);
    EXPECT_EQ(mc.count, 2);
    EXPECT_EQ(mc.mission_type, MAV_MISSION_TYPE_FENCE);

    // Download requests are sequential: ask for seq 0 first, then seq 1.
    sendRequestInt(0, MAV_MISSION_TYPE_FENCE);
    sendRequestInt(1, MAV_MISSION_TYPE_FENCE);
    const mavlink_message_t *im = lastOf(MAVLINK_MSG_ID_MISSION_ITEM_INT);
    ASSERT_NE(im, nullptr);
    mavlink_mission_item_int_t it;
    mavlink_msg_mission_item_int_decode(im, &it);
    EXPECT_EQ(it.mission_type, MAV_MISSION_TYPE_FENCE);
    EXPECT_EQ(it.command, MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION);
    EXPECT_EQ(it.x, 300);
    EXPECT_EQ(it.y, 400);
    EXPECT_FLOAT_EQ(it.param1, 25.0f);
}

TEST_F(MavMissionTest, FenceUploadRejectsNonFenceCommand)
{
    sendCount(1, MAV_MISSION_TYPE_FENCE);
    sendItem(0, MAV_CMD_NAV_WAYPOINT, MAV_MISSION_TYPE_FENCE, 1, 2);
    EXPECT_EQ(lastAckResult(), MAV_MISSION_UNSUPPORTED);
}

TEST_F(MavMissionTest, FenceUploadRejectsOverCapacity)
{
    sendCount(100, MAV_MISSION_TYPE_FENCE); // exceeds MAX_FENCE_POINTS
    EXPECT_EQ(lastAckResult(), MAV_MISSION_NO_SPACE);
}

// --- MISSION_SET_CURRENT ---
TEST_F(MavMissionTest, SetCurrentValidEmitsMissionCurrent)
{
    navActive = true;
    navCurrentIndex = 3;
    navSetResult = true;

    mavlink_message_t m;
    mavlink_msg_mission_set_current_pack(GCS_SYS, GCS_COMP, &m, FC_SYS, FC_COMP, 3);
    feed(&m);

    EXPECT_EQ(navSetIndexArg, 3);
    EXPECT_NE(lastOf(MAVLINK_MSG_ID_MISSION_CURRENT), nullptr);
    EXPECT_EQ(lastOf(MAVLINK_MSG_ID_MISSION_ACK), nullptr); // success is not an ACK
}

TEST_F(MavMissionTest, SetCurrentInvalidReportsError)
{
    navSetResult = false;

    mavlink_message_t m;
    mavlink_msg_mission_set_current_pack(GCS_SYS, GCS_COMP, &m, FC_SYS, FC_COMP, 9);
    feed(&m);

    EXPECT_EQ(navSetIndexArg, 9);
    EXPECT_EQ(lastAckResult(), MAV_MISSION_ERROR);
}

// --- Regression: waypoint mission path still works ---
TEST_F(MavMissionTest, MissionWaypointUploadStillWorks)
{
    sendCount(1, MAV_MISSION_TYPE_MISSION);
    sendItem(0, MAV_CMD_NAV_WAYPOINT, MAV_MISSION_TYPE_MISSION, 123, 456);

    EXPECT_EQ(lastAckResult(), MAV_MISSION_ACCEPTED);
    EXPECT_EQ(flightPlanConfig()->waypointCount, 1);
    EXPECT_EQ(flightPlanConfig()->waypoints[0].latitude, 123);
    EXPECT_EQ(flightPlanConfig()->waypoints[0].longitude, 456);
    EXPECT_GT(saveCount, 0); // mission IS persisted
}

TEST_F(MavMissionTest, ClearAllTypeAllClearsBoth)
{
    // Seed a waypoint mission + a fence.
    sendCount(1, MAV_MISSION_TYPE_MISSION);
    sendItem(0, MAV_CMD_NAV_WAYPOINT, MAV_MISSION_TYPE_MISSION, 1, 2);
    ASSERT_EQ(flightPlanConfig()->waypointCount, 1);

    sendCount(1, MAV_MISSION_TYPE_FENCE);
    sendItem(0, MAV_CMD_NAV_FENCE_RETURN_POINT, MAV_MISSION_TYPE_FENCE, 3, 4);

    mavlink_message_t m;
    mavlink_msg_mission_clear_all_pack(GCS_SYS, GCS_COMP, &m, FC_SYS, FC_COMP, MAV_MISSION_TYPE_ALL);
    feed(&m);

    EXPECT_EQ(lastAckResult(), MAV_MISSION_ACCEPTED);
    EXPECT_EQ(flightPlanConfig()->waypointCount, 0);

    // Fence should now download as empty.
    sent.clear();
    sendRequestList(MAV_MISSION_TYPE_FENCE);
    const mavlink_message_t *cm = lastOf(MAVLINK_MSG_ID_MISSION_COUNT);
    ASSERT_NE(cm, nullptr);
    mavlink_mission_count_t mc;
    mavlink_msg_mission_count_decode(cm, &mc);
    EXPECT_EQ(mc.count, 0);
}
