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

#include <cstdint>
#include <cstring>
#include <vector>

extern "C" {
    #include "platform.h"

    #include "fc/runtime_config.h"
    #include "io/gps.h"
    #include "pg/autopilot.h"
    #include "pg/flight_plan.h"
    #include "flight/flight_plan_nav.h"
    #include "telemetry/mavlink_mission.h"

    uint16_t flightModeFlags = 0;
    uint8_t stateFlags = 0;
    gpsLocation_t GPS_home_llh;
    gpsSolutionData_t gpsSol;

    // Executor mock state, driven by the tests.
    static bool s_navActive = false;
    static bool s_navInjected = false;
    static flightPlanNavState_e s_navState = FP_NAV_IDLE;
    static uint8_t s_navIndex = 0;
    static int s_setCurrentArg = -1;
    static int s_saveCalls = 0;
    static uint32_t s_millis = 0;
    static flightPlanWaypointReachedFn s_reachedListener = nullptr;

    bool flightPlanNavIsActive(void) { return s_navActive; }
    bool flightPlanNavIsInjectedPlanActive(void) { return s_navInjected; }
    flightPlanNavState_e flightPlanNavGetState(void) { return s_navState; }
    uint8_t flightPlanNavGetCurrentIndex(void) { return s_navIndex; }
    void flightPlanNavSetCurrentIndex(uint8_t index) { s_setCurrentArg = index; }
    void flightPlanNavSetReachedListener(flightPlanWaypointReachedFn fn) { s_reachedListener = fn; }

    // Fixed orbit period so LOITER_TURNS <-> ORBIT conversion is deterministic.
    uint16_t flightPlanNavOrbitPeriodDs(uint16_t) { return 250; }

    void saveConfigAndNotify(void) { s_saveCalls++; }
    uint32_t millis(void) { return s_millis; }

    // Capture every message the module transmits.
    static std::vector<mavlink_message_t> *s_sent = nullptr;
    void mavlinkSendMessage(mavlink_message_t *msg) { if (s_sent) { s_sent->push_back(*msg); } }
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// GCS identity for messages fed to the module.
static const uint8_t GCS_SYS = 255;
static const uint8_t GCS_COMP = 190;

class MavlinkMissionTest : public ::testing::Test {
protected:
    std::vector<mavlink_message_t> sent;

    void SetUp() override {
        flightModeFlags = 0;
        stateFlags = 0;
        memset(&GPS_home_llh, 0, sizeof(GPS_home_llh));
        GPS_home_llh.altCm = 10000;
        ENABLE_STATE(GPS_FIX_HOME);
        memset(&gpsSol, 0, sizeof(gpsSol));

        autopilotConfig_t *cfg = autopilotConfigMutable();
        memset(cfg, 0, sizeof(*cfg));
        cfg->waypointHoldRadius = 800; // 8 m

        flightPlanConfig_t *plan = flightPlanConfigMutable();
        memset(plan, 0, sizeof(*plan));

        s_navActive = false;
        s_navInjected = false;
        s_navState = FP_NAV_IDLE;
        s_navIndex = 0;
        s_setCurrentArg = -1;
        s_saveCalls = 0;
        s_millis = 0;
        s_reachedListener = nullptr;

        sent.clear();
        s_sent = &sent;

        mavMissionInit();
    }

    void feed(const mavlink_message_t &msg) {
        mavlink_message_t copy = msg;
        copy.sysid = GCS_SYS;
        copy.compid = GCS_COMP;
        mavMissionHandleMessage(&copy);
    }

    const mavlink_message_t *lastOfType(uint32_t msgid) const {
        for (auto it = sent.rbegin(); it != sent.rend(); ++it) {
            if (it->msgid == msgid) {
                return &(*it);
            }
        }
        return nullptr;
    }

    int countOfType(uint32_t msgid) const {
        int n = 0;
        for (const auto &m : sent) {
            if (m.msgid == msgid) { n++; }
        }
        return n;
    }

    // Drive one upload item and return the resulting item write.
    void sendItem(uint16_t seq, uint16_t command, uint16_t total,
                  float param1 = 0.0f, int32_t lat = 0, int32_t lon = 0, float alt = 0.0f) {
        mavlink_message_t msg;
        mavlink_msg_mission_item_int_pack(GCS_SYS, GCS_COMP, &msg,
            1, 0, seq, MAV_FRAME_GLOBAL_INT, command, /*current*/ 0, /*autocontinue*/ 1,
            param1, 0.0f, 0.0f, 0.0f, lat, lon, alt, MAV_MISSION_TYPE_MISSION);
        (void)total;
        feed(msg);
    }

    void sendCount(uint16_t count) {
        mavlink_message_t msg;
        mavlink_msg_mission_count_pack(GCS_SYS, GCS_COMP, &msg,
            1, 0, count, MAV_MISSION_TYPE_MISSION, 0);
        feed(msg);
    }
};

TEST_F(MavlinkMissionTest, UploadWritesWaypointsToConfig)
{
    sendCount(3);
    // COUNT is answered with REQUEST_INT(0).
    const mavlink_message_t *req = lastOfType(MAVLINK_MSG_ID_MISSION_REQUEST_INT);
    ASSERT_NE(req, nullptr);
    mavlink_mission_request_int_t r;
    mavlink_msg_mission_request_int_decode(req, &r);
    EXPECT_EQ(r.seq, 0);

    sendItem(0, MAV_CMD_NAV_TAKEOFF, 3, 0.0f, 100, 200, 12.0f);
    sendItem(1, MAV_CMD_NAV_WAYPOINT, 3, 0.0f, 300, 400, 20.0f);
    sendItem(2, MAV_CMD_NAV_LAND, 3, 0.0f, 500, 600, 0.0f);

    // Final item is acked and the config is persisted.
    const mavlink_message_t *ack = lastOfType(MAVLINK_MSG_ID_MISSION_ACK);
    ASSERT_NE(ack, nullptr);
    mavlink_mission_ack_t a;
    mavlink_msg_mission_ack_decode(ack, &a);
    EXPECT_EQ(a.type, MAV_MISSION_ACCEPTED);
    EXPECT_EQ(s_saveCalls, 1);

    const flightPlanConfig_t *plan = flightPlanConfig();
    ASSERT_EQ(plan->waypointCount, 3);
    EXPECT_EQ(plan->waypoints[0].type, WAYPOINT_TYPE_TAKEOFF);
    EXPECT_EQ(plan->waypoints[1].type, WAYPOINT_TYPE_FLYBY);
    EXPECT_EQ(plan->waypoints[1].latitude, 300);
    EXPECT_EQ(plan->waypoints[1].longitude, 400);
    EXPECT_EQ(plan->waypoints[2].type, WAYPOINT_TYPE_LAND);
}

TEST_F(MavlinkMissionTest, UploadMapsLoiterTurnsToOrbitHold)
{
    sendCount(1);
    sendItem(0, MAV_CMD_NAV_LOITER_TURNS, 1, /*turns*/ 3.0f, 100, 200, 15.0f);

    const flightPlanConfig_t *plan = flightPlanConfig();
    ASSERT_EQ(plan->waypointCount, 1);
    EXPECT_EQ(plan->waypoints[0].type, WAYPOINT_TYPE_HOLD);
    EXPECT_EQ(plan->waypoints[0].pattern, WAYPOINT_PATTERN_ORBIT);
    // 3 turns * 250 ds/turn (mocked orbit period) = 750 ds.
    EXPECT_EQ(plan->waypoints[0].duration, 750);
}

TEST_F(MavlinkMissionTest, UploadDefaultsPatternToNone)
{
    sendCount(1);
    sendItem(0, MAV_CMD_NAV_LOITER_TIME, 1, /*seconds*/ 5.0f, 100, 200, 15.0f);

    const flightPlanConfig_t *plan = flightPlanConfig();
    ASSERT_EQ(plan->waypointCount, 1);
    EXPECT_EQ(plan->waypoints[0].type, WAYPOINT_TYPE_HOLD);
    EXPECT_EQ(plan->waypoints[0].pattern, WAYPOINT_PATTERN_NONE);
}

TEST_F(MavlinkMissionTest, UploadRejectedWhileAutopilotActive)
{
    flightModeFlags = AUTOPILOT_MODE;
    sendCount(2);

    const mavlink_message_t *ack = lastOfType(MAVLINK_MSG_ID_MISSION_ACK);
    ASSERT_NE(ack, nullptr);
    mavlink_mission_ack_t a;
    mavlink_msg_mission_ack_decode(ack, &a);
    EXPECT_EQ(a.type, MAV_MISSION_DENIED);
    EXPECT_EQ(countOfType(MAVLINK_MSG_ID_MISSION_REQUEST_INT), 0);
}

TEST_F(MavlinkMissionTest, DownloadEncodesWaypointsRoundTrip)
{
    flightPlanConfig_t *plan = flightPlanConfigMutable();
    plan->waypointCount = 3;
    plan->waypoints[0] = { 100, 200, 1200, 0, 0, WAYPOINT_TYPE_TAKEOFF, WAYPOINT_PATTERN_NONE };
    plan->waypoints[1] = { 300, 400, 2000, 0, 300 /* 30 s */, WAYPOINT_TYPE_HOLD, WAYPOINT_PATTERN_ORBIT };
    plan->waypoints[2] = { 500, 600, 0, 0, 0, WAYPOINT_TYPE_LAND, WAYPOINT_PATTERN_NONE };

    // REQUEST_LIST -> COUNT.
    mavlink_message_t list;
    mavlink_msg_mission_request_list_pack(GCS_SYS, GCS_COMP, &list, 1, 0, MAV_MISSION_TYPE_MISSION);
    feed(list);
    const mavlink_message_t *cnt = lastOfType(MAVLINK_MSG_ID_MISSION_COUNT);
    ASSERT_NE(cnt, nullptr);
    mavlink_mission_count_t mc;
    mavlink_msg_mission_count_decode(cnt, &mc);
    EXPECT_EQ(mc.count, 3);

    auto requestItem = [&](uint16_t seq) -> uint16_t {
        mavlink_message_t req;
        mavlink_msg_mission_request_int_pack(GCS_SYS, GCS_COMP, &req, 1, 0, seq, MAV_MISSION_TYPE_MISSION);
        feed(req);
        const mavlink_message_t *item = lastOfType(MAVLINK_MSG_ID_MISSION_ITEM_INT);
        EXPECT_NE(item, nullptr);
        mavlink_mission_item_int_t it;
        mavlink_msg_mission_item_int_decode(item, &it);
        EXPECT_EQ(it.seq, seq);
        return it.command;
    };

    EXPECT_EQ(requestItem(0), MAV_CMD_NAV_TAKEOFF);
    EXPECT_EQ(requestItem(1), MAV_CMD_NAV_LOITER_TURNS); // ORBIT hold -> LOITER_TURNS
    EXPECT_EQ(requestItem(2), MAV_CMD_NAV_LAND);
}

TEST_F(MavlinkMissionTest, MissionItemReachedEmitted)
{
    ASSERT_NE(s_reachedListener, nullptr);
    s_reachedListener(2);

    mavMissionUpdate(0); // time 0 avoids a coincident MISSION_CURRENT
    const mavlink_message_t *reached = lastOfType(MAVLINK_MSG_ID_MISSION_ITEM_REACHED);
    ASSERT_NE(reached, nullptr);
    mavlink_mission_item_reached_t mr;
    mavlink_msg_mission_item_reached_decode(reached, &mr);
    EXPECT_EQ(mr.seq, 2);

    // Latched index is one-shot: a second update does not re-emit.
    sent.clear();
    mavMissionUpdate(0);
    EXPECT_EQ(countOfType(MAVLINK_MSG_ID_MISSION_ITEM_REACHED), 0);
}

TEST_F(MavlinkMissionTest, MissionCurrentReflectsExecutorState)
{
    flightPlanConfigMutable()->waypointCount = 3;
    s_navActive = true;
    s_navState = FP_NAV_TARGETING;
    s_navIndex = 1;

    mavMissionUpdate(1000);
    const mavlink_message_t *cur = lastOfType(MAVLINK_MSG_ID_MISSION_CURRENT);
    ASSERT_NE(cur, nullptr);
    mavlink_mission_current_t mc;
    mavlink_msg_mission_current_decode(cur, &mc);
    EXPECT_EQ(mc.seq, 1);
    EXPECT_EQ(mc.total, 3);
    EXPECT_EQ(mc.mission_state, MISSION_STATE_ACTIVE);
}

TEST_F(MavlinkMissionTest, MissionCurrentSuppressedForInjectedPlan)
{
    flightPlanConfigMutable()->waypointCount = 3;
    s_navActive = true;
    s_navInjected = true;
    s_navIndex = 2;

    mavMissionUpdate(1000);
    const mavlink_message_t *cur = lastOfType(MAVLINK_MSG_ID_MISSION_CURRENT);
    ASSERT_NE(cur, nullptr);
    mavlink_mission_current_t mc;
    mavlink_msg_mission_current_decode(cur, &mc);
    // An injected runtime plan owns its own sequencing; the uploaded-mission
    // cursor stays at 0 for the tracking GCS.
    EXPECT_EQ(mc.seq, 0);
}

TEST_F(MavlinkMissionTest, SetCurrentDispatchesAndEchoesCursor)
{
    flightPlanConfigMutable()->waypointCount = 3;
    s_navActive = true;
    s_navIndex = 2;

    mavlink_message_t sc;
    mavlink_msg_mission_set_current_pack(GCS_SYS, GCS_COMP, &sc, 1, 0, 2);
    feed(sc);

    EXPECT_EQ(s_setCurrentArg, 2);
    EXPECT_NE(lastOfType(MAVLINK_MSG_ID_MISSION_CURRENT), nullptr);
}
