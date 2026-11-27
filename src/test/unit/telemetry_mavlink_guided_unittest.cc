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

// Compiles the telemetry_mavlink_unittest.cc tests with ENABLE_FLIGHT_PLAN, and adds the guided ones.
#include "telemetry_mavlink_unittest.cc"

extern "C" {
    static bool navActive;
    static flightPlanNavState_e navState;
    static bool navGuided;
    static bool navTakingOff;
    static bool navGuidedLanding;
    static std::vector<waypoint_t> guidedPlan;
    static float guidedRadiusM;
    static bool guidedAccepted;
    static int cancelCalls;
    static int leaveCalls;
    static int resumeCalls;
    static bool resumeResult;
    static int startIndex;
    static bool startResult;
    static int speedOverrideCmS;
    static bool altitudeAvailable;

    bool flightPlanNavIsActive(void) { return navActive; }
    flightPlanNavState_e flightPlanNavGetState(void) { return navState; }
    bool flightPlanNavIsGuided(void) { return navGuided; }
    bool flightPlanNavIsTakingOff(void) { return navTakingOff; }
    bool flightPlanNavIsGuidedLanding(void) { return navGuidedLanding; }
    bool flightPlanNavSetGuidedPlan(const waypoint_t *waypoints, uint8_t count, float orbitRadiusM)
    {
        if (!guidedAccepted) {
            return false;
        }
        guidedPlan.assign(waypoints, waypoints + count);
        guidedRadiusM = orbitRadiusM;
        return true;
    }
    void flightPlanNavCancelGuided(void) { cancelCalls++; }
    void flightPlanNavLeaveGuided(void) { leaveCalls++; }
    bool flightPlanNavResumeMission(void)
    {
        resumeCalls++;
        return resumeResult;
    }
    bool flightPlanNavStartMission(uint8_t index)
    {
        startIndex = index;
        return startResult;
    }
    void flightPlanNavSetSpeedOverride(uint16_t speedCmS) { speedOverrideCmS = speedCmS; }
    uint16_t flightPlanNavGuidedOrbitPeriodDs(uint16_t, float) { return 200; }
    bool isAltitudeAvailable(void) { return altitudeAvailable; }
}

#define HERE_LAT -335000001
#define HERE_LON 1512500001
#define HERE_ALT_CM 4250

static int32_t deg1e7(float deg)
{
    return lround((double)deg * 1e7);
}

class MavlinkGuidedTest : public MavlinkTelemetryTest {
protected:
    void SetUp() override
    {
        MavlinkTelemetryTest::SetUp();
        navActive = false;
        navState = FP_NAV_IDLE;
        navGuided = false;
        navTakingOff = false;
        navGuidedLanding = false;
        guidedPlan.clear();
        guidedRadiusM = NAN;
        guidedAccepted = true;
        cancelCalls = 0;
        leaveCalls = 0;
        resumeCalls = 0;
        resumeResult = false;
        startIndex = -1;
        startResult = true;
        speedOverrideCmS = -1;
        altitudeAvailable = true;

        ENABLE_ARMING_FLAG(ARMED);
        throttleRaised = true;
        sensorMask = SENSOR_ACC;
        ENABLE_STATE(GPS_FIX);
        gpsSol.llh.lat = HERE_LAT;
        gpsSol.llh.lon = HERE_LON;
        gpsSol.llh.altCm = HERE_ALT_CM;
    }

    uint8_t command(uint16_t cmd, float p1 = 0, float p2 = 0, float p3 = 0, float p4 = 0,
        float p5 = NAN, float p6 = NAN, float p7 = NAN)
    {
        sent.clear();
        commandLong(cmd, p1, p2, p3, p4, p5, p6, p7);
        return lastAck().result;
    }

    void expectGuided(uint8_t type, int32_t lat, int32_t lon, int32_t altCm)
    {
        ASSERT_EQ(guidedPlan.size(), 1u);
        EXPECT_EQ(guidedPlan[0].type, type);
        EXPECT_EQ(guidedPlan[0].latitude, lat);
        EXPECT_EQ(guidedPlan[0].longitude, lon);
        EXPECT_EQ(guidedPlan[0].altitude, altCm);
        EXPECT_EQ(overrideMask, 1u << BOXAUTOPILOT);
    }

    void flyGuided()
    {
        ENABLE_FLIGHT_MODE(AUTOPILOT_MODE);
        navActive = true;
        navGuided = true;
    }
};

TEST_F(MavlinkGuidedTest, GuidedModeHoldsWhereTheCraftIs)
{
    setMode(MODE_GUIDED);
    EXPECT_EQ(lastAck().result, MAV_RESULT_ACCEPTED);
    expectGuided(WAYPOINT_TYPE_HOLD, HERE_LAT, HERE_LON, HERE_ALT_CM);
    EXPECT_EQ(guidedPlan[0].duration, UINT16_MAX);
    EXPECT_EQ(guidedPlan[0].pattern, WAYPOINT_PATTERN_NONE);
}

TEST_F(MavlinkGuidedTest, GuidedModeNeedsTheCraftReadyToFly)
{
    setMode(MODE_POS_HOLD);
    const uint32_t before = overrideMask;

    throttleRaised = false;
    setMode(MODE_GUIDED);
    EXPECT_EQ(lastAck().result, MAV_RESULT_TEMPORARILY_REJECTED);

    throttleRaised = true;
    DISABLE_STATE(GPS_FIX);
    setMode(MODE_GUIDED);
    EXPECT_EQ(lastAck().result, MAV_RESULT_TEMPORARILY_REJECTED);

    ENABLE_STATE(GPS_FIX);
    altitudeAvailable = false;
    setMode(MODE_GUIDED);
    EXPECT_EQ(lastAck().result, MAV_RESULT_TEMPORARILY_REJECTED);

    altitudeAvailable = true;
    guidedAccepted = false;
    setMode(MODE_GUIDED);
    EXPECT_EQ(lastAck().result, MAV_RESULT_TEMPORARILY_REJECTED);

    EXPECT_TRUE(guidedPlan.empty());
    EXPECT_EQ(overrideMask, before);
}

TEST_F(MavlinkGuidedTest, LandModeLandsWhereTheCraftIs)
{
    setMode(MODE_LAND);
    EXPECT_EQ(lastAck().result, MAV_RESULT_ACCEPTED);
    expectGuided(WAYPOINT_TYPE_LAND, HERE_LAT, HERE_LON, HERE_ALT_CM);

    guidedPlan.clear();
    EXPECT_EQ(ackResultFor(MAV_CMD_DO_SET_STANDARD_MODE, MAV_STANDARD_MODE_LAND), MAV_RESULT_ACCEPTED);
    expectGuided(WAYPOINT_TYPE_LAND, HERE_LAT, HERE_LON, HERE_ALT_CM);
}

TEST_F(MavlinkGuidedTest, TakeoffModeIsNotSelectable)
{
    setMode(MODE_TAKEOFF);
    EXPECT_EQ(lastAck().result, MAV_RESULT_DENIED);
    EXPECT_EQ(ackResultFor(MAV_CMD_DO_SET_STANDARD_MODE, MAV_STANDARD_MODE_TAKEOFF), MAV_RESULT_FAILED);
    EXPECT_TRUE(guidedPlan.empty());
}

TEST_F(MavlinkGuidedTest, AutopilotModeHandsAGuidedPlanBackToTheMission)
{
    setMode(MODE_AUTOPILOT);
    EXPECT_EQ(leaveCalls, 1);
    EXPECT_EQ(cancelCalls, 0);

    setMode(MODE_POS_HOLD);
    setMode(MODE_ACRO);
    EXPECT_EQ(ackResultFor(MAV_CMD_NAV_RETURN_TO_LAUNCH), MAV_RESULT_ACCEPTED);
    EXPECT_EQ(leaveCalls, 1);
    EXPECT_EQ(cancelCalls, 3);
}

TEST_F(MavlinkGuidedTest, HeartbeatReportsGuidedTakeoffAndLand)
{
    ENABLE_FLIGHT_MODE(AUTOPILOT_MODE);
    mavlink_heartbeat_t hb = heartbeat();
    EXPECT_EQ(hb.custom_mode, (uint32_t)MODE_AUTOPILOT);
    EXPECT_TRUE(hb.base_mode & MAV_MODE_FLAG_AUTO_ENABLED);

    navGuided = true;
    hb = heartbeat();
    EXPECT_EQ(hb.custom_mode, (uint32_t)MODE_GUIDED);
    EXPECT_TRUE(hb.base_mode & MAV_MODE_FLAG_GUIDED_ENABLED);
    EXPECT_FALSE(hb.base_mode & MAV_MODE_FLAG_AUTO_ENABLED);

    navTakingOff = true;
    EXPECT_EQ(heartbeat().custom_mode, (uint32_t)MODE_TAKEOFF);

    navTakingOff = false;
    navGuidedLanding = true;
    hb = heartbeat();
    EXPECT_EQ(hb.custom_mode, (uint32_t)MODE_LAND);
    EXPECT_TRUE(hb.base_mode & MAV_MODE_FLAG_GUIDED_ENABLED);
}

TEST_F(MavlinkGuidedTest, NavLandFliesToThePointAtItsHeightAndLands)
{
    EXPECT_EQ(command(MAV_CMD_NAV_LAND, 0, 0, 0, NAN, -33.6f, 151.3f, 10.0f), MAV_RESULT_ACCEPTED);
    expectGuided(WAYPOINT_TYPE_LAND, deg1e7(-33.6f), deg1e7(151.3f), HERE_ALT_CM);

    guidedPlan.clear();
    EXPECT_EQ(command(MAV_CMD_NAV_LAND), MAV_RESULT_ACCEPTED);
    expectGuided(WAYPOINT_TYPE_LAND, HERE_LAT, HERE_LON, HERE_ALT_CM);

    guidedPlan.clear();
    EXPECT_EQ(command(MAV_CMD_NAV_LAND, 0, 0, 0, NAN, 95.0f, 151.3f), MAV_RESULT_DENIED);
    EXPECT_TRUE(guidedPlan.empty());
}

TEST_F(MavlinkGuidedTest, RepositionNeedsChangeModeUnlessAlreadyGuided)
{
    EXPECT_EQ(command(MAV_CMD_DO_REPOSITION, -1, 0, 0, NAN, -33.6f, 151.3f, 60.0f), MAV_RESULT_DENIED);
    EXPECT_TRUE(guidedPlan.empty());

    EXPECT_EQ(command(MAV_CMD_DO_REPOSITION, -1, MAV_DO_REPOSITION_FLAGS_CHANGE_MODE, 0, NAN, -33.6f, 151.3f, 60.0f), MAV_RESULT_ACCEPTED);
    expectGuided(WAYPOINT_TYPE_HOLD, deg1e7(-33.6f), deg1e7(151.3f), 6000);
    EXPECT_EQ(speedOverrideCmS, -1);

    flyGuided();
    guidedPlan.clear();
    EXPECT_EQ(command(MAV_CMD_DO_REPOSITION, 5.0f, 0, 0, NAN, -33.6f, 151.3f, 60.0f), MAV_RESULT_ACCEPTED);
    EXPECT_EQ(guidedPlan.size(), 1u);
    EXPECT_EQ(speedOverrideCmS, 500);
}

TEST_F(MavlinkGuidedTest, RepositionWithoutAPositionHoldsHere)
{
    sent.clear();
    commandInt(MAV_CMD_DO_REPOSITION, -1, INT32_MAX, INT32_MAX, NAN, MAV_DO_REPOSITION_FLAGS_CHANGE_MODE);
    EXPECT_EQ(lastAck().result, MAV_RESULT_ACCEPTED);
    expectGuided(WAYPOINT_TYPE_HOLD, HERE_LAT, HERE_LON, HERE_ALT_CM);
}

TEST_F(MavlinkGuidedTest, PauseHoldsHereAndContinueResumesTheMission)
{
    EXPECT_EQ(command(MAV_CMD_DO_PAUSE_CONTINUE, 0), MAV_RESULT_ACCEPTED);
    expectGuided(WAYPOINT_TYPE_HOLD, HERE_LAT, HERE_LON, HERE_ALT_CM);

    EXPECT_EQ(command(MAV_CMD_DO_PAUSE_CONTINUE, 1), MAV_RESULT_DENIED);
    resumeResult = true;
    EXPECT_EQ(command(MAV_CMD_DO_PAUSE_CONTINUE, 1), MAV_RESULT_ACCEPTED);
    EXPECT_EQ(resumeCalls, 2);

    EXPECT_EQ(command(MAV_CMD_DO_PAUSE_CONTINUE, 2), MAV_RESULT_DENIED);
}

TEST_F(MavlinkGuidedTest, MissionStartStartsAtTheFirstItem)
{
    throttleRaised = false;
    EXPECT_EQ(command(MAV_CMD_MISSION_START, 2), MAV_RESULT_TEMPORARILY_REJECTED);
    EXPECT_EQ(startIndex, -1);

    throttleRaised = true;
    startResult = false;
    EXPECT_EQ(command(MAV_CMD_MISSION_START, 2), MAV_RESULT_DENIED);
    EXPECT_EQ(overrideMask, 0u);

    startResult = true;
    EXPECT_EQ(command(MAV_CMD_MISSION_START, 2), MAV_RESULT_ACCEPTED);
    EXPECT_EQ(startIndex, 2);
    EXPECT_EQ(overrideMask, 1u << BOXAUTOPILOT);
}

TEST_F(MavlinkGuidedTest, ChangeSpeedSetsTheOverride)
{
    EXPECT_EQ(command(MAV_CMD_DO_CHANGE_SPEED, SPEED_TYPE_GROUNDSPEED, 4.0f), MAV_RESULT_ACCEPTED);
    EXPECT_EQ(speedOverrideCmS, 400);

    speedOverrideCmS = -1;
    EXPECT_EQ(command(MAV_CMD_DO_CHANGE_SPEED, SPEED_TYPE_GROUNDSPEED, -1.0f), MAV_RESULT_ACCEPTED);
    EXPECT_EQ(speedOverrideCmS, -1);

    EXPECT_EQ(command(MAV_CMD_DO_CHANGE_SPEED, SPEED_TYPE_GROUNDSPEED, -2.0f), MAV_RESULT_ACCEPTED);
    EXPECT_EQ(speedOverrideCmS, 0);

    EXPECT_EQ(command(MAV_CMD_DO_CHANGE_SPEED, SPEED_TYPE_GROUNDSPEED, 0.0f), MAV_RESULT_DENIED);
    EXPECT_EQ(command(MAV_CMD_DO_CHANGE_SPEED, SPEED_TYPE_GROUNDSPEED, NAN), MAV_RESULT_DENIED);

    DISABLE_ARMING_FLAG(ARMED);
    EXPECT_EQ(command(MAV_CMD_DO_CHANGE_SPEED, SPEED_TYPE_GROUNDSPEED, 4.0f), MAV_RESULT_TEMPORARILY_REJECTED);
}

TEST_F(MavlinkGuidedTest, OrbitFliesAGuidedOrbit)
{
    EXPECT_EQ(command(MAV_CMD_DO_ORBIT, 25.0f, 3.0f, 0, 4.0f * M_PIf, -33.6f, 151.3f, 60.0f), MAV_RESULT_ACCEPTED);
    expectGuided(WAYPOINT_TYPE_HOLD, deg1e7(-33.6f), deg1e7(151.3f), 6000);
    EXPECT_EQ(guidedPlan[0].pattern, WAYPOINT_PATTERN_ORBIT);
    EXPECT_EQ(guidedPlan[0].speed, 300);
    EXPECT_EQ(guidedPlan[0].duration, 400);
    EXPECT_FLOAT_EQ(guidedRadiusM, 25.0f);

    EXPECT_EQ(command(MAV_CMD_DO_ORBIT, -25.0f, NAN, 0, NAN), MAV_RESULT_ACCEPTED);
    expectGuided(WAYPOINT_TYPE_HOLD, HERE_LAT, HERE_LON, HERE_ALT_CM);
    EXPECT_EQ(guidedPlan[0].speed, 0);
    EXPECT_EQ(guidedPlan[0].duration, UINT16_MAX);
    EXPECT_FLOAT_EQ(guidedRadiusM, -25.0f);

    EXPECT_EQ(command(MAV_CMD_DO_ORBIT, NAN, NAN, 0, 0), MAV_RESULT_ACCEPTED);
    EXPECT_FLOAT_EQ(guidedRadiusM, 0.0f);

    guidedPlan.clear();
    EXPECT_EQ(command(MAV_CMD_DO_ORBIT, 0.5f), MAV_RESULT_DENIED);
    EXPECT_EQ(command(MAV_CMD_DO_ORBIT, 25.0f, NAN, 0, -1.0f), MAV_RESULT_DENIED);
    EXPECT_TRUE(guidedPlan.empty());
}
