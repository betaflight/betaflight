/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

extern "C" {
    #include "platform.h"

    #include "common/time.h"

    #include "fc/runtime_config.h"

    #include "flight/flight_plan_capture.h"

    #include "io/gps.h"

    #include "pg/flight_plan.h"
    #include "pg/pg.h"

    uint8_t stateFlags;
    gpsSolutionData_t gpsSol;
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// --- Stubs ---

namespace {
timeUs_t g_stubMicros;
bool g_stubNavActive;
} // namespace

extern "C" {

timeUs_t micros(void)
{
    return g_stubMicros;
}

bool flightPlanNavIsActive(void)
{
    return g_stubNavActive;
}

} // extern "C"

class FlightPlanCaptureTest : public ::testing::Test {
protected:
    void SetUp() override {
        flightPlanCaptureResetForTest();
        memset(&gpsSol, 0, sizeof(gpsSol));
        flightPlanConfigMutable()->waypointCount = 0;
        stateFlags = GPS_FIX;
        g_stubNavActive = false;
        g_stubMicros = 1000000;
        setPosition(500000000, -1200000000, 3000);
    }

    void setPosition(int32_t lat, int32_t lon, int32_t altCm) {
        gpsSol.llh.lat = lat;
        gpsSol.llh.lon = lon;
        gpsSol.llh.altCm = altCm;
    }

    void advanceMs(uint32_t ms) {
        g_stubMicros += ms * 1000;
    }

    void update(bool switchActive, bool channelsValid = true) {
        flightPlanCaptureUpdate(g_stubMicros, switchActive, channelsValid);
    }

    // press then release with the switch held for holdMs
    void tap(uint32_t holdMs = 100) {
        update(true);
        advanceMs(holdMs);
        update(false);
    }

    void fillPlan(uint8_t count) {
        flightPlanConfig_t *plan = flightPlanConfigMutable();
        plan->waypointCount = count;
        for (uint8_t i = 0; i < count; i++) {
            plan->waypoints[i] = { 100 + i, 200 + i, 1000, 0, 0, WAYPOINT_TYPE_FLYBY, WAYPOINT_PATTERN_NONE };
        }
    }
};

TEST_F(FlightPlanCaptureTest, TapAppendsWaypointSampledAtThePress)
{
    update(true);                                  // press samples here
    setPosition(500001000, -1200001000, 4500);     // craft drifts before release
    advanceMs(200);
    update(false);                                 // commit

    const flightPlanConfig_t *plan = flightPlanConfig();
    ASSERT_EQ(1, plan->waypointCount);
    EXPECT_EQ(500000000, plan->waypoints[0].latitude);
    EXPECT_EQ(-1200000000, plan->waypoints[0].longitude);
    EXPECT_EQ(3000, plan->waypoints[0].altitude);
    EXPECT_EQ(WAYPOINT_TYPE_FLYBY, plan->waypoints[0].type);
    EXPECT_EQ(0, plan->waypoints[0].speed);       // leg default cruise
    ASSERT_TRUE(flightPlanCaptureOsdMessage() != NULL);
    EXPECT_STREQ("WP1 SET", flightPlanCaptureOsdMessage());
}

TEST_F(FlightPlanCaptureTest, HoldDeletesTheLastWaypointAndTheReleaseDropsNothing)
{
    fillPlan(3);

    update(true);
    advanceMs(1600);                               // past the 1.5 s threshold
    update(true);                                  // delete fires while held
    EXPECT_EQ(2, flightPlanConfig()->waypointCount);
    EXPECT_STREQ("WP3 DELETED", flightPlanCaptureOsdMessage());

    advanceMs(400);
    update(false);                                 // release must not also drop
    EXPECT_EQ(2, flightPlanConfig()->waypointCount);
}

TEST_F(FlightPlanCaptureTest, HoldDeleteStillWorksOnAFullPlan)
{
    fillPlan(MAX_WAYPOINTS);

    update(true);
    advanceMs(1600);
    update(true);
    EXPECT_EQ(MAX_WAYPOINTS - 1, flightPlanConfig()->waypointCount);
}

TEST_F(FlightPlanCaptureTest, TapOnAFullPlanAddsNothingAndSaysSo)
{
    fillPlan(MAX_WAYPOINTS);

    tap();
    EXPECT_EQ(MAX_WAYPOINTS, flightPlanConfig()->waypointCount);
    ASSERT_TRUE(flightPlanCaptureOsdMessage() != NULL);
    EXPECT_STREQ("WP FULL", flightPlanCaptureOsdMessage());
}

TEST_F(FlightPlanCaptureTest, NoFixMeansNoWaypoint)
{
    stateFlags = 0;

    tap();
    EXPECT_EQ(0, flightPlanConfig()->waypointCount);
    EXPECT_TRUE(flightPlanCaptureOsdMessage() == NULL);
}

TEST_F(FlightPlanCaptureTest, SwitchIsIgnoredWhileTheMissionFlies)
{
    fillPlan(2);
    g_stubNavActive = true;

    tap();                                         // no append
    EXPECT_EQ(2, flightPlanConfig()->waypointCount);

    update(true);
    advanceMs(1600);
    update(true);                                  // no delete either
    update(false);
    EXPECT_EQ(2, flightPlanConfig()->waypointCount);
    EXPECT_TRUE(flightPlanCaptureOsdMessage() == NULL);
}

TEST_F(FlightPlanCaptureTest, SignalLossMidHoldFreezesTheGestureNotCommitsIt)
{
    fillPlan(2);

    // press starts a hold that's heading for a delete...
    update(true);
    advanceMs(800);
    // ...but the link drops before the 1.5 s threshold. that must not read as a
    // release and commit the pending waypoint - freeze instead
    update(true, false);
    EXPECT_EQ(2, flightPlanConfig()->waypointCount);
    EXPECT_TRUE(flightPlanCaptureOsdMessage() == NULL);

    // link back, still held, cross the threshold: the delete lands as intended
    advanceMs(800);
    update(true);
    EXPECT_EQ(1, flightPlanConfig()->waypointCount);
    EXPECT_STREQ("WP2 DELETED", flightPlanCaptureOsdMessage());
}

TEST_F(FlightPlanCaptureTest, MissionEngagingBetweenPressAndReleaseCancelsTheDrop)
{
    update(true);
    advanceMs(200);
    g_stubNavActive = true;                        // executor takes the plan mid-gesture
    update(false);
    EXPECT_EQ(0, flightPlanConfig()->waypointCount);
}

TEST_F(FlightPlanCaptureTest, ConsecutiveTapsNumberTheWaypoints)
{
    tap();
    advanceMs(500);
    tap();
    EXPECT_EQ(2, flightPlanConfig()->waypointCount);
    ASSERT_TRUE(flightPlanCaptureOsdMessage() != NULL);
    EXPECT_STREQ("WP2 SET", flightPlanCaptureOsdMessage());
}

TEST_F(FlightPlanCaptureTest, OsdMessageExpires)
{
    tap();
    ASSERT_TRUE(flightPlanCaptureOsdMessage() != NULL);
    advanceMs(2400);
    EXPECT_TRUE(flightPlanCaptureOsdMessage() != NULL);
    advanceMs(200);                                // past the 2.5 s cue lifetime
    EXPECT_TRUE(flightPlanCaptureOsdMessage() == NULL);
}
