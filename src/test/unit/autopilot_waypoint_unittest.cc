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
#include <limits.h>
#include <math.h>
#include <string.h>

extern "C" {

    #include "platform.h"
    #include "build/debug.h"
    #include "pg/pg_ids.h"

    #include "common/axis.h"
    #include "common/maths.h"
    #include "common/vector.h"

    #include "fc/rc_controls.h"
    #include "fc/runtime_config.h"

    #include "flight/autopilot_waypoint.h"
    #include "flight/position.h"

    #include "io/gps.h"

    #include "pg/autopilot.h"
    #include "pg/flight_plan.h"
    #include "pg/rx.h"

    PG_REGISTER(autopilotConfig_t, autopilotConfig, PG_AUTOPILOT, 0);
    PG_REGISTER(flightPlanConfig_t, flightPlanConfig, PG_FLIGHT_PLAN_CONFIG, 0);

    timeUs_t currentTimeUs = 0;

    extern gpsSolutionData_t gpsSol;
    extern uint8_t stateFlags;
    extern uint16_t flightModeFlags;
    extern uint8_t armingFlags;
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// ---------------------------------------------------------------------------
// Controllable mock state
// ---------------------------------------------------------------------------
static uint32_t mockDistanceCm = 0;
static int32_t mockBearingCdeg = 0;
static bool mockBelowLandingAlt = false;
static float mockAltitudeCm = 10000.0f;
static bool mockL1Active = false;
static gpsLocation_t mockNavOrigin = {};

uint32_t millisRW;
uint32_t millis() {
    return millisRW;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static void resetTestState(void)
{
    mockDistanceCm = 0;
    mockBearingCdeg = 0;
    mockBelowLandingAlt = false;
    mockAltitudeCm = 10000.0f;
    mockL1Active = false;
    stateFlags = GPS_FIX;
    flightModeFlags = 0;
    armingFlags = 0;
    currentTimeUs = 0;
    memset(&gpsSol, 0, sizeof(gpsSol));
    memset(&mockNavOrigin, 0, sizeof(mockNavOrigin));
    waypointInit();
}

static void setupAutopilotConfig(void)
{
    autopilotConfig_t *cfg = autopilotConfigMutable();
    cfg->waypointArrivalRadius = 500;   // 5 m
    cfg->waypointHoldRadius = 200;      // 2 m
    cfg->l1Enable = 0;
    cfg->maxAngle = 25;
    cfg->maxTurnRate = 3;
    cfg->holdOrbitRadius = 1000;
    cfg->holdFigure8Width = 2000;
    cfg->landingDescentRate = 50;
    cfg->landingDetectionTime = 10;
    cfg->landingVelocityThreshold = 50;
    cfg->landingThrottleThreshold = 100;
    cfg->hoverThrottle = 1500;
    cfg->l1MaxCrossTrackError = 10000;
    cfg->l1Period = 20;
    cfg->l1MinLookahead = 1000;
    cfg->l1MaxLookahead = 10000;
}

static void setupSingleWaypoint(int32_t lat, int32_t lon, int32_t alt, uint8_t type)
{
    flightPlanConfig_t *config = flightPlanConfigMutable();
    config->waypointCount = 1;
    config->waypoints[0].latitude = lat;
    config->waypoints[0].longitude = lon;
    config->waypoints[0].altitude = alt;
    config->waypoints[0].speed = 0;
    config->waypoints[0].duration = 0;
    config->waypoints[0].type = type;
    config->waypoints[0].pattern = 0;
}

static void setupWaypoint(uint8_t idx, int32_t lat, int32_t lon, int32_t alt,
                           uint8_t type, uint8_t pattern, uint16_t duration)
{
    flightPlanConfig_t *config = flightPlanConfigMutable();
    config->waypoints[idx].latitude = lat;
    config->waypoints[idx].longitude = lon;
    config->waypoints[idx].altitude = alt;
    config->waypoints[idx].speed = 0;
    config->waypoints[idx].duration = duration;
    config->waypoints[idx].type = type;
    config->waypoints[idx].pattern = pattern;
}

// =========================================================================
// Existing tests (preserved)
// =========================================================================

TEST(AutopilotWaypointUnittest, InitializationTest)
{
    waypointInit();

    EXPECT_EQ(waypointGetState(), WP_STATE_IDLE);
    EXPECT_EQ(waypointIsSystemValid(), false);
    EXPECT_EQ(waypointGetCurrentIndex(), 0);
}

TEST(AutopilotWaypointUnittest, ResetNoWaypointsTest)
{
    flightPlanConfigMutable()->waypointCount = 0;

    waypointReset();

    EXPECT_EQ(waypointGetState(), WP_STATE_IDLE);
    EXPECT_EQ(waypointIsSystemValid(), false);
}

TEST(AutopilotWaypointUnittest, ResetWithWaypointTest)
{
    setupSingleWaypoint(404635100, -795181700, 50000, WAYPOINT_TYPE_FLYOVER);
    stateFlags |= GPS_FIX;

    waypointReset();

    EXPECT_EQ(waypointGetState(), WP_STATE_APPROACHING);
    EXPECT_EQ(waypointIsSystemValid(), true);
    EXPECT_EQ(waypointGetCurrentIndex(), 0);

    const gpsLocation_t *target = waypointGetTarget();
    EXPECT_EQ(target->lat, 404635100);
    EXPECT_EQ(target->lon, -795181700);
    EXPECT_EQ(target->altCm, 50000);
}

TEST(AutopilotWaypointUnittest, EmergencyLandingSetsLandWaypoint)
{
    setupSingleWaypoint(404635100, -795181700, 50000, WAYPOINT_TYPE_FLYOVER);

    gpsSol.llh.lat = 404700000;
    gpsSol.llh.lon = -795200000;
    gpsSol.llh.altCm = 30000;
    stateFlags |= GPS_FIX;

    waypointSetEmergencyLanding();

    const flightPlanConfig_t *plan = flightPlanConfig();
    EXPECT_EQ(plan->waypointCount, 1);
    EXPECT_EQ(plan->waypoints[0].type, WAYPOINT_TYPE_LAND);
    EXPECT_EQ(plan->waypoints[0].latitude, 404700000);
    EXPECT_EQ(plan->waypoints[0].longitude, -795200000);
}

TEST(AutopilotWaypointUnittest, EmergencyLandingSetsLandingState)
{
    gpsSol.llh.lat = 404700000;
    gpsSol.llh.lon = -795200000;
    gpsSol.llh.altCm = 30000;
    stateFlags |= GPS_FIX;

    waypointSetEmergencyLanding();

    EXPECT_EQ(waypointGetState(), WP_STATE_LANDING);
    EXPECT_EQ(waypointIsSystemValid(), true);
    EXPECT_EQ(waypointGetCurrentIndex(), 0);
}

TEST(AutopilotWaypointUnittest, EmergencyLandingTargetMatchesPosition)
{
    gpsSol.llh.lat = 404700000;
    gpsSol.llh.lon = -795200000;
    gpsSol.llh.altCm = 30000;
    stateFlags |= GPS_FIX;

    waypointSetEmergencyLanding();

    const gpsLocation_t *target = waypointGetTarget();
    EXPECT_EQ(target->lat, gpsSol.llh.lat);
    EXPECT_EQ(target->lon, gpsSol.llh.lon);
}

TEST(AutopilotWaypointUnittest, EmergencyLandingOverwritesMission)
{
    flightPlanConfig_t *config = flightPlanConfigMutable();
    config->waypointCount = 3;
    config->waypoints[0].latitude = 100000000;
    config->waypoints[0].longitude = 200000000;
    config->waypoints[0].type = WAYPOINT_TYPE_FLYOVER;
    config->waypoints[1].latitude = 110000000;
    config->waypoints[1].longitude = 210000000;
    config->waypoints[1].type = WAYPOINT_TYPE_FLYBY;
    config->waypoints[2].latitude = 120000000;
    config->waypoints[2].longitude = 220000000;
    config->waypoints[2].type = WAYPOINT_TYPE_HOLD;

    gpsSol.llh.lat = 115000000;
    gpsSol.llh.lon = 215000000;
    gpsSol.llh.altCm = 20000;
    stateFlags |= GPS_FIX;

    waypointSetEmergencyLanding();

    const flightPlanConfig_t *plan = flightPlanConfig();
    EXPECT_EQ(plan->waypointCount, 1);
    EXPECT_EQ(plan->waypoints[0].type, WAYPOINT_TYPE_LAND);
    EXPECT_EQ(plan->waypoints[0].latitude, 115000000);
    EXPECT_EQ(plan->waypoints[0].longitude, 215000000);
    EXPECT_EQ(waypointGetState(), WP_STATE_LANDING);
}

// =========================================================================
// New state machine transition tests
// =========================================================================

// --- Reset edge cases ---

TEST(AutopilotWaypointUnittest, MultiWaypointResetSetsIndices)
{
    resetTestState();
    setupAutopilotConfig();

    flightPlanConfigMutable()->waypointCount = 3;
    setupWaypoint(0, 400000000, 100000000, 50000, WAYPOINT_TYPE_FLYOVER, 0, 0);
    setupWaypoint(1, 401000000, 100000000, 50000, WAYPOINT_TYPE_FLYOVER, 0, 0);
    setupWaypoint(2, 402000000, 100000000, 50000, WAYPOINT_TYPE_HOLD, 0, 0);

    waypointReset();

    EXPECT_EQ(waypointGetState(), WP_STATE_APPROACHING);
    EXPECT_EQ(waypointGetCurrentIndex(), 0);
    EXPECT_EQ(waypointGetNextIndex(), 1);
    EXPECT_EQ(waypointIsSystemValid(), true);
}

TEST(AutopilotWaypointUnittest, SingleWaypointResetNextEqualsCurrentIndex)
{
    resetTestState();
    setupAutopilotConfig();
    setupSingleWaypoint(400000000, 100000000, 50000, WAYPOINT_TYPE_HOLD);

    waypointReset();

    EXPECT_EQ(waypointGetCurrentIndex(), 0);
    EXPECT_EQ(waypointGetNextIndex(), 0);
}

// --- APPROACHING → ARRIVED transitions ---

TEST(AutopilotWaypointUnittest, ApproachingToArrivedHoldType)
{
    resetTestState();
    setupAutopilotConfig();
    setupSingleWaypoint(400000000, 100000000, 50000, WAYPOINT_TYPE_HOLD);

    waypointReset();
    EXPECT_EQ(waypointGetState(), WP_STATE_APPROACHING);

    // Simulate being within hold radius
    mockDistanceCm = 150;  // < waypointHoldRadius (200)
    currentTimeUs = 1000000;
    waypointUpdate(currentTimeUs);

    EXPECT_EQ(waypointGetState(), WP_STATE_ARRIVED);
}

TEST(AutopilotWaypointUnittest, ApproachingToArrivedLandType)
{
    resetTestState();
    setupAutopilotConfig();
    setupSingleWaypoint(400000000, 100000000, 50000, WAYPOINT_TYPE_LAND);

    waypointReset();

    mockDistanceCm = 100;  // < waypointHoldRadius (200)
    currentTimeUs = 1000000;
    waypointUpdate(currentTimeUs);

    EXPECT_EQ(waypointGetState(), WP_STATE_ARRIVED);
}

TEST(AutopilotWaypointUnittest, SingleWaypointFlyoverUsesArrivalRadius)
{
    // Single FLYOVER waypoint uses waypointArrivalRadius (not plane crossing)
    resetTestState();
    setupAutopilotConfig();
    setupSingleWaypoint(400000000, 100000000, 50000, WAYPOINT_TYPE_FLYOVER);

    waypointReset();

    mockDistanceCm = 400;  // < waypointArrivalRadius (500)
    currentTimeUs = 1000000;
    waypointUpdate(currentTimeUs);

    EXPECT_EQ(waypointGetState(), WP_STATE_ARRIVED);
}

TEST(AutopilotWaypointUnittest, ApproachingStaysWhenOutsideRadius)
{
    resetTestState();
    setupAutopilotConfig();
    setupSingleWaypoint(400000000, 100000000, 50000, WAYPOINT_TYPE_HOLD);

    waypointReset();

    mockDistanceCm = 500;  // > waypointHoldRadius (200), stay in APPROACHING
    currentTimeUs = 1000000;
    waypointUpdate(currentTimeUs);

    EXPECT_EQ(waypointGetState(), WP_STATE_APPROACHING);
}

// --- FLYOVER plane crossing ---

TEST(AutopilotWaypointUnittest, FlyoverPlaneCrossing)
{
    resetTestState();
    setupAutopilotConfig();

    // Two FLYOVER waypoints along latitude (north-south line)
    flightPlanConfigMutable()->waypointCount = 2;
    setupWaypoint(0, 400000000, 100000000, 50000, WAYPOINT_TYPE_FLYOVER, 0, 0);
    setupWaypoint(1, 401000000, 100000000, 50000, WAYPOINT_TYPE_FLYOVER, 0, 0);

    // Set nav origin near the waypoints for NED precision
    mockNavOrigin.lat = 400000000;
    mockNavOrigin.lon = 100000000;
    mockNavOrigin.altCm = 0;

    // Position south of WP0 before reset (sets previousPosition)
    gpsSol.llh.lat = 399900000;
    gpsSol.llh.lon = 100000000;
    gpsSol.llh.altCm = 50000;

    waypointReset();
    EXPECT_EQ(waypointGetState(), WP_STATE_APPROACHING);

    // Move past WP0 (north of it)
    gpsSol.llh.lat = 400100000;
    gpsSol.llh.lon = 100000000;
    gpsSol.llh.altCm = 50000;

    mockDistanceCm = 1000;  // GPS_distance_cm_bearing result (distance/bearing still updated)
    currentTimeUs = 1000000;
    waypointUpdate(currentTimeUs);

    // Should have crossed the perpendicular plane → ARRIVED
    EXPECT_EQ(waypointGetState(), WP_STATE_ARRIVED);
}

TEST(AutopilotWaypointUnittest, FlyoverNoCrossingStaysApproaching)
{
    resetTestState();
    setupAutopilotConfig();

    flightPlanConfigMutable()->waypointCount = 2;
    setupWaypoint(0, 400000000, 100000000, 50000, WAYPOINT_TYPE_FLYOVER, 0, 0);
    setupWaypoint(1, 401000000, 100000000, 50000, WAYPOINT_TYPE_FLYOVER, 0, 0);

    mockNavOrigin.lat = 400000000;
    mockNavOrigin.lon = 100000000;
    mockNavOrigin.altCm = 0;

    // Position south of WP0
    gpsSol.llh.lat = 399900000;
    gpsSol.llh.lon = 100000000;
    gpsSol.llh.altCm = 50000;

    waypointReset();

    // Still south of WP0 (no crossing)
    gpsSol.llh.lat = 399950000;
    gpsSol.llh.lon = 100000000;

    mockDistanceCm = 5000;
    currentTimeUs = 1000000;
    waypointUpdate(currentTimeUs);

    EXPECT_EQ(waypointGetState(), WP_STATE_APPROACHING);
}

// --- FLYBY turn radius transition ---

TEST(AutopilotWaypointUnittest, FlybyApproachWithinTurnRadius)
{
    resetTestState();
    setupAutopilotConfig();

    flightPlanConfigMutable()->waypointCount = 2;
    setupWaypoint(0, 400000000, 100000000, 50000, WAYPOINT_TYPE_FLYBY, 0, 0);
    setupWaypoint(1, 401000000, 100000000, 50000, WAYPOINT_TYPE_FLYBY, 0, 0);

    waypointReset();

    // Ground speed 500 cm/s (5 m/s), maxAngle=25°
    // Turn radius ≈ 250000 / (980 * tan(25°)) ≈ 547 cm
    gpsSol.groundSpeed = 500;
    mockDistanceCm = 400;  // < ~547 → should transition
    currentTimeUs = 1000000;
    waypointUpdate(currentTimeUs);

    EXPECT_EQ(waypointGetState(), WP_STATE_ARRIVED);
}

// --- ARRIVED transitions ---

TEST(AutopilotWaypointUnittest, ArrivedFlyoverAdvancesToNext)
{
    resetTestState();
    setupAutopilotConfig();

    flightPlanConfigMutable()->waypointCount = 2;
    setupWaypoint(0, 400000000, 100000000, 50000, WAYPOINT_TYPE_FLYOVER, 0, 0);
    setupWaypoint(1, 401000000, 100000000, 50000, WAYPOINT_TYPE_FLYOVER, 0, 0);

    mockNavOrigin.lat = 400000000;
    mockNavOrigin.lon = 100000000;

    // Start south of WP0, then move past it to trigger ARRIVED
    gpsSol.llh.lat = 399900000;
    gpsSol.llh.lon = 100000000;
    gpsSol.llh.altCm = 50000;
    waypointReset();

    gpsSol.llh.lat = 400100000;
    mockDistanceCm = 1000;
    currentTimeUs = 1000000;
    waypointUpdate(currentTimeUs);
    EXPECT_EQ(waypointGetState(), WP_STATE_ARRIVED);

    // Process ARRIVED state → should advance to next waypoint
    currentTimeUs = 2000000;
    waypointUpdate(currentTimeUs);

    EXPECT_EQ(waypointGetState(), WP_STATE_APPROACHING);
    EXPECT_EQ(waypointGetCurrentIndex(), 1);
}

TEST(AutopilotWaypointUnittest, ArrivedHoldTransitionsToHolding)
{
    resetTestState();
    setupAutopilotConfig();
    setupSingleWaypoint(400000000, 100000000, 50000, WAYPOINT_TYPE_HOLD);

    waypointReset();

    // Transition to ARRIVED
    mockDistanceCm = 100;
    currentTimeUs = 1000000;
    waypointUpdate(currentTimeUs);
    EXPECT_EQ(waypointGetState(), WP_STATE_ARRIVED);

    // Process ARRIVED → HOLDING
    currentTimeUs = 2000000;
    waypointUpdate(currentTimeUs);

    EXPECT_EQ(waypointGetState(), WP_STATE_HOLDING);
}

TEST(AutopilotWaypointUnittest, ArrivedLandTransitionsToLanding)
{
    resetTestState();
    setupAutopilotConfig();
    setupSingleWaypoint(400000000, 100000000, 50000, WAYPOINT_TYPE_LAND);

    waypointReset();

    // Transition to ARRIVED
    mockDistanceCm = 100;
    currentTimeUs = 1000000;
    waypointUpdate(currentTimeUs);
    EXPECT_EQ(waypointGetState(), WP_STATE_ARRIVED);

    // Process ARRIVED → LANDING
    currentTimeUs = 2000000;
    waypointUpdate(currentTimeUs);

    EXPECT_EQ(waypointGetState(), WP_STATE_LANDING);
}

// --- HOLDING transitions ---

TEST(AutopilotWaypointUnittest, HoldingToOrbiting)
{
    resetTestState();
    setupAutopilotConfig();
    // pattern = WAYPOINT_PATTERN_ORBIT (0)
    setupSingleWaypoint(400000000, 100000000, 50000, WAYPOINT_TYPE_HOLD);
    flightPlanConfigMutable()->waypoints[0].pattern = WAYPOINT_PATTERN_ORBIT;

    waypointReset();

    // APPROACHING → ARRIVED
    mockDistanceCm = 100;
    currentTimeUs = 1000000;
    waypointUpdate(currentTimeUs);
    EXPECT_EQ(waypointGetState(), WP_STATE_ARRIVED);

    // ARRIVED → HOLDING
    currentTimeUs = 2000000;
    waypointUpdate(currentTimeUs);
    EXPECT_EQ(waypointGetState(), WP_STATE_HOLDING);

    // HOLDING → ORBITING (pattern is ORBIT)
    currentTimeUs = 3000000;
    waypointUpdate(currentTimeUs);

    EXPECT_EQ(waypointGetState(), WP_STATE_ORBITING);
}

TEST(AutopilotWaypointUnittest, HoldingToFigure8)
{
    resetTestState();
    setupAutopilotConfig();
    setupSingleWaypoint(400000000, 100000000, 50000, WAYPOINT_TYPE_HOLD);
    flightPlanConfigMutable()->waypoints[0].pattern = WAYPOINT_PATTERN_FIGURE8;

    waypointReset();

    mockDistanceCm = 100;
    currentTimeUs = 1000000;
    waypointUpdate(currentTimeUs);  // → ARRIVED

    currentTimeUs = 2000000;
    waypointUpdate(currentTimeUs);  // → HOLDING

    currentTimeUs = 3000000;
    waypointUpdate(currentTimeUs);  // → FIGURE8

    EXPECT_EQ(waypointGetState(), WP_STATE_FIGURE8);
}

TEST(AutopilotWaypointUnittest, HoldDurationExpiryNoPattern)
{
    resetTestState();
    setupAutopilotConfig();
    // Use invalid pattern (not ORBIT or FIGURE8) to test plain hold with duration
    setupSingleWaypoint(400000000, 100000000, 50000, WAYPOINT_TYPE_HOLD);
    flightPlanConfigMutable()->waypoints[0].pattern = 255;  // invalid → else branch
    flightPlanConfigMutable()->waypoints[0].duration = 10;  // 1 second

    waypointReset();

    mockDistanceCm = 100;
    currentTimeUs = 1000000;
    waypointUpdate(currentTimeUs);  // → ARRIVED

    currentTimeUs = 2000000;
    waypointUpdate(currentTimeUs);  // → HOLDING (holdStartTime = 2000000)
    EXPECT_EQ(waypointGetState(), WP_STATE_HOLDING);

    // Not yet expired
    currentTimeUs = 2500000;  // 0.5s elapsed, need 1s
    waypointUpdate(currentTimeUs);
    EXPECT_EQ(waypointGetState(), WP_STATE_HOLDING);

    // Expired: elapsed >= 1000000µs
    currentTimeUs = 3100000;  // 1.1s elapsed
    waypointUpdate(currentTimeUs);

    // Single waypoint, advance → COMPLETE
    EXPECT_EQ(waypointGetState(), WP_STATE_COMPLETE);
}

TEST(AutopilotWaypointUnittest, OrbitDurationExpiry)
{
    resetTestState();
    setupAutopilotConfig();

    // 2 waypoints: first HOLD with ORBIT pattern and duration, second FLYOVER
    flightPlanConfigMutable()->waypointCount = 2;
    setupWaypoint(0, 400000000, 100000000, 50000, WAYPOINT_TYPE_HOLD, WAYPOINT_PATTERN_ORBIT, 10); // 1s
    setupWaypoint(1, 401000000, 100000000, 50000, WAYPOINT_TYPE_FLYOVER, 0, 0);

    waypointReset();

    mockDistanceCm = 100;
    currentTimeUs = 1000000;
    waypointUpdate(currentTimeUs);  // → ARRIVED

    currentTimeUs = 2000000;
    waypointUpdate(currentTimeUs);  // → HOLDING (holdStartTime = 2000000)

    currentTimeUs = 3000000;
    waypointUpdate(currentTimeUs);  // → ORBITING

    EXPECT_EQ(waypointGetState(), WP_STATE_ORBITING);

    // Duration expires: holdStartTime=2000000, duration=10 → need 1000000µs
    currentTimeUs = 3100000;  // 1.1s after holdStartTime
    waypointUpdate(currentTimeUs);

    // Should advance to next waypoint (WP1)
    EXPECT_EQ(waypointGetState(), WP_STATE_APPROACHING);
    EXPECT_EQ(waypointGetCurrentIndex(), 1);
}

// --- Last waypoint → COMPLETE ---

TEST(AutopilotWaypointUnittest, LastWaypointComplete)
{
    resetTestState();
    setupAutopilotConfig();

    flightPlanConfigMutable()->waypointCount = 2;
    setupWaypoint(0, 400000000, 100000000, 50000, WAYPOINT_TYPE_FLYOVER, 0, 0);
    setupWaypoint(1, 401000000, 100000000, 50000, WAYPOINT_TYPE_FLYOVER, 0, 0);

    mockNavOrigin.lat = 400000000;
    mockNavOrigin.lon = 100000000;

    // Start at WP0, cross it to arrive
    gpsSol.llh.lat = 399900000;
    gpsSol.llh.lon = 100000000;
    gpsSol.llh.altCm = 50000;
    waypointReset();

    gpsSol.llh.lat = 400100000;
    mockDistanceCm = 1000;
    currentTimeUs = 1000000;
    waypointUpdate(currentTimeUs);  // → ARRIVED at WP0

    currentTimeUs = 2000000;
    waypointUpdate(currentTimeUs);  // ARRIVED → advance to WP1, APPROACHING
    EXPECT_EQ(waypointGetState(), WP_STATE_APPROACHING);
    EXPECT_EQ(waypointGetCurrentIndex(), 1);

    // Now at WP1 (last waypoint). nextIndex == currentIndex for last WP.
    // For last FLYOVER waypoint, uses arrival radius.
    mockDistanceCm = 300;  // < waypointArrivalRadius (500)
    currentTimeUs = 3000000;
    waypointUpdate(currentTimeUs);  // → ARRIVED at WP1
    EXPECT_EQ(waypointGetState(), WP_STATE_ARRIVED);

    // ARRIVED at last FLYOVER → advance → no more waypoints → COMPLETE
    currentTimeUs = 4000000;
    waypointUpdate(currentTimeUs);
    EXPECT_EQ(waypointGetState(), WP_STATE_COMPLETE);
}

// --- Safety checks ---

TEST(AutopilotWaypointUnittest, UpdateWithNoGpsFix)
{
    resetTestState();
    setupAutopilotConfig();
    setupSingleWaypoint(400000000, 100000000, 50000, WAYPOINT_TYPE_HOLD);

    waypointReset();
    EXPECT_EQ(waypointGetState(), WP_STATE_APPROACHING);

    // Remove GPS fix
    stateFlags &= ~GPS_FIX;
    currentTimeUs = 1000000;
    waypointUpdate(currentTimeUs);

    EXPECT_EQ(waypointGetState(), WP_STATE_IDLE);
    EXPECT_EQ(waypointIsSystemValid(), false);
}

TEST(AutopilotWaypointUnittest, UpdateWithZeroWaypoints)
{
    resetTestState();
    setupAutopilotConfig();
    setupSingleWaypoint(400000000, 100000000, 50000, WAYPOINT_TYPE_HOLD);

    waypointReset();
    EXPECT_EQ(waypointGetState(), WP_STATE_APPROACHING);

    // Clear waypoints while in flight
    flightPlanConfigMutable()->waypointCount = 0;
    currentTimeUs = 1000000;
    waypointUpdate(currentTimeUs);

    EXPECT_EQ(waypointGetState(), WP_STATE_IDLE);
    EXPECT_EQ(waypointIsSystemValid(), false);
}

// --- waypointAdvanceToNext edge case ---

TEST(AutopilotWaypointUnittest, AdvanceToNextClampsIndex)
{
    resetTestState();
    setupAutopilotConfig();

    // 3 waypoints - verify nextIndex clamping at the last waypoint
    flightPlanConfigMutable()->waypointCount = 3;
    setupWaypoint(0, 400000000, 100000000, 50000, WAYPOINT_TYPE_HOLD, WAYPOINT_PATTERN_ORBIT, 1);
    setupWaypoint(1, 401000000, 100000000, 50000, WAYPOINT_TYPE_HOLD, WAYPOINT_PATTERN_ORBIT, 1);
    setupWaypoint(2, 402000000, 100000000, 50000, WAYPOINT_TYPE_HOLD, WAYPOINT_PATTERN_ORBIT, 1);

    waypointReset();
    EXPECT_EQ(waypointGetCurrentIndex(), 0);
    EXPECT_EQ(waypointGetNextIndex(), 1);

    // Advance: 0→1
    waypointAdvanceToNext();
    EXPECT_EQ(waypointGetCurrentIndex(), 1);
    EXPECT_EQ(waypointGetNextIndex(), 2);

    // Advance: 1→2
    waypointAdvanceToNext();
    EXPECT_EQ(waypointGetCurrentIndex(), 2);
    // nextIndex should be clamped to 2 (same as current, can't go further)
    EXPECT_EQ(waypointGetNextIndex(), 2);

    // Advance again: nextIndex == currentIndex → COMPLETE
    waypointAdvanceToNext();
    EXPECT_EQ(waypointGetState(), WP_STATE_COMPLETE);
}

// --- waypointIsValid ---

TEST(AutopilotWaypointUnittest, WaypointValidation)
{
    resetTestState();
    flightPlanConfigMutable()->waypointCount = 3;

    EXPECT_TRUE(waypointIsValid(0));
    EXPECT_TRUE(waypointIsValid(1));
    EXPECT_TRUE(waypointIsValid(2));
    EXPECT_FALSE(waypointIsValid(3));
    EXPECT_FALSE(waypointIsValid(255));
}

// --- Emergency landing edge cases (RX loss related) ---

TEST(AutopilotWaypointUnittest, EmergencyLandingFromApproaching)
{
    resetTestState();
    setupAutopilotConfig();

    flightPlanConfigMutable()->waypointCount = 3;
    setupWaypoint(0, 400000000, 100000000, 50000, WAYPOINT_TYPE_FLYOVER, 0, 0);
    setupWaypoint(1, 401000000, 100000000, 50000, WAYPOINT_TYPE_FLYOVER, 0, 0);
    setupWaypoint(2, 402000000, 100000000, 50000, WAYPOINT_TYPE_HOLD, 0, 0);

    gpsSol.llh.lat = 400500000;
    gpsSol.llh.lon = 100000000;
    gpsSol.llh.altCm = 50000;

    waypointReset();
    EXPECT_EQ(waypointGetState(), WP_STATE_APPROACHING);

    // Emergency landing mid-mission
    waypointSetEmergencyLanding();

    EXPECT_EQ(waypointGetState(), WP_STATE_LANDING);
    EXPECT_EQ(waypointGetCurrentIndex(), 0);

    // Flight plan reduced to single LAND waypoint
    EXPECT_EQ(flightPlanConfig()->waypointCount, 1);
    EXPECT_EQ(flightPlanConfig()->waypoints[0].type, WAYPOINT_TYPE_LAND);
    EXPECT_EQ(flightPlanConfig()->waypoints[0].latitude, 400500000);
}

TEST(AutopilotWaypointUnittest, EmergencyLandingWhenAlreadyLanding)
{
    resetTestState();
    setupAutopilotConfig();
    setupSingleWaypoint(400000000, 100000000, 50000, WAYPOINT_TYPE_LAND);

    gpsSol.llh.lat = 400000000;
    gpsSol.llh.lon = 100000000;
    gpsSol.llh.altCm = 50000;

    waypointReset();

    // Manually transition to LANDING via the state machine
    mockDistanceCm = 100;
    currentTimeUs = 1000000;
    waypointUpdate(currentTimeUs);  // → ARRIVED

    currentTimeUs = 2000000;
    waypointUpdate(currentTimeUs);  // → LANDING
    EXPECT_EQ(waypointGetState(), WP_STATE_LANDING);

    // Call emergency landing again - should still be LANDING (idempotent)
    gpsSol.llh.lat = 400001000;  // Slightly different position
    waypointSetEmergencyLanding();

    EXPECT_EQ(waypointGetState(), WP_STATE_LANDING);
    // But target position should be updated to current GPS
    EXPECT_EQ(flightPlanConfig()->waypoints[0].latitude, 400001000);
}

// =========================================================================
// Geofence tests
// =========================================================================

TEST(AutopilotWaypointUnittest, GeofenceDisabledAllowsAnyWaypoint)
{
    resetTestState();
    setupAutopilotConfig();
    autopilotConfigMutable()->maxDistanceFromHomeM = 0;  // Disabled

    setupSingleWaypoint(500000000, 100000000, 50000, WAYPOINT_TYPE_FLYOVER);

    waypointReset();

    EXPECT_EQ(waypointGetState(), WP_STATE_APPROACHING);
    EXPECT_EQ(waypointIsSystemValid(), true);
}

TEST(AutopilotWaypointUnittest, GeofenceRejectsMissionBeyondLimit)
{
    resetTestState();
    setupAutopilotConfig();
    autopilotConfigMutable()->maxDistanceFromHomeM = 1000;  // 1km limit

    // Set home position
    GPS_home_llh.lat = 400000000;
    GPS_home_llh.lon = 100000000;
    GPS_home_llh.altCm = 0;
    stateFlags |= GPS_FIX_HOME;

    // mockDistanceCm will be returned by GPS_distance_cm_bearing for ALL calls
    // Set it to exceed geofence (1000m = 100000cm)
    mockDistanceCm = 200000;  // 2km away

    setupSingleWaypoint(410000000, 100000000, 50000, WAYPOINT_TYPE_FLYOVER);

    waypointReset();

    // Mission should be rejected
    EXPECT_EQ(waypointGetState(), WP_STATE_IDLE);
    EXPECT_EQ(waypointIsSystemValid(), false);
}

TEST(AutopilotWaypointUnittest, GeofenceAllowsMissionWithinLimit)
{
    resetTestState();
    setupAutopilotConfig();
    autopilotConfigMutable()->maxDistanceFromHomeM = 1000;  // 1km limit

    GPS_home_llh.lat = 400000000;
    GPS_home_llh.lon = 100000000;
    GPS_home_llh.altCm = 0;
    stateFlags |= GPS_FIX_HOME;

    // Within geofence
    mockDistanceCm = 50000;  // 500m away

    setupSingleWaypoint(400100000, 100000000, 50000, WAYPOINT_TYPE_FLYOVER);

    waypointReset();

    EXPECT_EQ(waypointGetState(), WP_STATE_APPROACHING);
    EXPECT_EQ(waypointIsSystemValid(), true);
}

TEST(AutopilotWaypointUnittest, RuntimeGeofenceTriggersEmergencyLanding)
{
    resetTestState();
    setupAutopilotConfig();
    autopilotConfigMutable()->maxDistanceFromHomeM = 1000;
    autopilotConfigMutable()->geofenceAction = AP_GEOFENCE_LAND;

    GPS_home_llh.lat = 400000000;
    GPS_home_llh.lon = 100000000;
    GPS_home_llh.altCm = 0;
    stateFlags |= GPS_FIX_HOME;

    // Start with waypoint within range so mission begins
    mockDistanceCm = 50000;
    setupSingleWaypoint(400100000, 100000000, 50000, WAYPOINT_TYPE_FLYOVER);

    waypointReset();
    EXPECT_EQ(waypointGetState(), WP_STATE_APPROACHING);

    // Now simulate craft exceeding geofence during flight
    GPS_distanceToHomeCm = 150000;  // 1.5km, exceeds 1km limit
    gpsSol.llh.lat = 400500000;
    gpsSol.llh.lon = 100000000;
    gpsSol.llh.altCm = 50000;

    currentTimeUs = 1000000;
    waypointUpdate(currentTimeUs);

    // Should trigger emergency landing
    EXPECT_EQ(waypointGetState(), WP_STATE_LANDING);
    EXPECT_EQ(flightPlanConfig()->waypoints[0].type, WAYPOINT_TYPE_LAND);
}

TEST(AutopilotWaypointUnittest, RuntimeGeofenceTriggersRTH)
{
    resetTestState();
    setupAutopilotConfig();
    autopilotConfigMutable()->maxDistanceFromHomeM = 1000;
    autopilotConfigMutable()->geofenceAction = AP_GEOFENCE_RTH;

    GPS_home_llh.lat = 400000000;
    GPS_home_llh.lon = 100000000;
    GPS_home_llh.altCm = 0;
    stateFlags |= GPS_FIX_HOME;

    mockDistanceCm = 50000;
    setupSingleWaypoint(400100000, 100000000, 50000, WAYPOINT_TYPE_FLYOVER);

    waypointReset();
    EXPECT_EQ(waypointGetState(), WP_STATE_APPROACHING);

    // Exceed geofence
    GPS_distanceToHomeCm = 150000;
    gpsSol.llh.lat = 400500000;
    gpsSol.llh.lon = 100000000;
    gpsSol.llh.altCm = 50000;

    currentTimeUs = 1000000;
    waypointUpdate(currentTimeUs);

    // Should trigger RTH (APPROACHING toward home)
    EXPECT_EQ(waypointGetState(), WP_STATE_APPROACHING);
    EXPECT_EQ(flightPlanConfig()->waypointCount, 2);
    EXPECT_EQ(flightPlanConfig()->waypoints[0].type, WAYPOINT_TYPE_FLYOVER);
    EXPECT_EQ(flightPlanConfig()->waypoints[0].latitude, GPS_home_llh.lat);
    EXPECT_EQ(flightPlanConfig()->waypoints[1].type, WAYPOINT_TYPE_LAND);
}

TEST(AutopilotWaypointUnittest, RuntimeGeofenceSkipsWhenAlreadyLanding)
{
    resetTestState();
    setupAutopilotConfig();
    autopilotConfigMutable()->maxDistanceFromHomeM = 1000;
    autopilotConfigMutable()->geofenceAction = AP_GEOFENCE_LAND;

    GPS_home_llh.lat = 400000000;
    GPS_home_llh.lon = 100000000;
    GPS_home_llh.altCm = 0;
    stateFlags |= GPS_FIX_HOME;

    gpsSol.llh.lat = 400500000;
    gpsSol.llh.lon = 100000000;
    gpsSol.llh.altCm = 50000;

    // Start emergency landing first
    waypointSetEmergencyLanding();
    EXPECT_EQ(waypointGetState(), WP_STATE_LANDING);

    // Now exceed geofence - should NOT re-trigger since already landing
    GPS_distanceToHomeCm = 150000;
    mockDistanceCm = 50000;
    currentTimeUs = 1000000;
    waypointUpdate(currentTimeUs);

    // Should still be in LANDING, not re-triggered
    EXPECT_EQ(waypointGetState(), WP_STATE_LANDING);
}

// =========================================================================
// Waypoint bounds validation tests
// =========================================================================

TEST(AutopilotWaypointUnittest, IsValidRejectsOutOfBoundsIndex)
{
    resetTestState();
    // Simulate corrupted waypointCount > MAX_WAYPOINTS
    flightPlanConfigMutable()->waypointCount = 50;

    EXPECT_TRUE(waypointIsValid(0));
    EXPECT_TRUE(waypointIsValid(29));    // Last valid array index
    EXPECT_FALSE(waypointIsValid(30));   // At MAX_WAYPOINTS boundary
    EXPECT_FALSE(waypointIsValid(31));   // Beyond MAX_WAYPOINTS
    EXPECT_FALSE(waypointIsValid(49));   // Within corrupted count but beyond array
}

TEST(AutopilotWaypointUnittest, ResetRejectsCorruptedWaypointCount)
{
    resetTestState();
    setupAutopilotConfig();
    flightPlanConfigMutable()->waypointCount = 255;  // Corrupted

    waypointReset();

    EXPECT_EQ(waypointGetState(), WP_STATE_IDLE);
    EXPECT_EQ(waypointIsSystemValid(), false);
}

TEST(AutopilotWaypointUnittest, ReturnToHomeSetsCorrectWaypoints)
{
    resetTestState();
    setupAutopilotConfig();

    GPS_home_llh.lat = 400000000;
    GPS_home_llh.lon = 100000000;
    GPS_home_llh.altCm = 0;

    gpsSol.llh.lat = 401000000;
    gpsSol.llh.lon = 101000000;
    gpsSol.llh.altCm = 50000;

    waypointSetReturnToHome();

    const flightPlanConfig_t *plan = flightPlanConfig();
    EXPECT_EQ(plan->waypointCount, 2);
    EXPECT_EQ(plan->waypoints[0].type, WAYPOINT_TYPE_FLYOVER);
    EXPECT_EQ(plan->waypoints[0].latitude, GPS_home_llh.lat);
    EXPECT_EQ(plan->waypoints[0].longitude, GPS_home_llh.lon);
    EXPECT_EQ(plan->waypoints[1].type, WAYPOINT_TYPE_LAND);
    EXPECT_EQ(plan->waypoints[1].latitude, GPS_home_llh.lat);
    EXPECT_EQ(plan->waypoints[1].longitude, GPS_home_llh.lon);

    EXPECT_EQ(waypointGetState(), WP_STATE_APPROACHING);
    EXPECT_EQ(waypointGetCurrentIndex(), 0);
    EXPECT_EQ(waypointGetNextIndex(), 1);
    EXPECT_EQ(waypointIsSystemValid(), true);
}

// =========================================================================
// STUBS
// =========================================================================

extern "C" {
    uint8_t armingFlags = 0;
    int16_t debug[DEBUG16_VALUE_COUNT];
    uint8_t debugMode;
    uint16_t flightModeFlags = 0;
    uint8_t stateFlags = 0;

    gpsSolutionData_t gpsSol = {};
    gpsLocation_t GPS_home_llh = {};
    uint32_t GPS_distanceToHomeCm = 0;
    uint16_t GPS_distanceToHome = 0;
    float rcCommand[4] = { 0.0f, 0.0f, 0.0f, 0.0f };

    float getAltitudeCm(void) { return mockAltitudeCm; }
    bool isBelowLandingAltitude(void) { return mockBelowLandingAlt; }
    bool isFixedWing(void) { return false; }

    void GPS_distance_cm_bearing(const gpsLocation_t *from, const gpsLocation_t *to,
                                  bool useTinyCalc, uint32_t *dist, int32_t *bearing) {
        UNUSED(from);
        UNUSED(to);
        UNUSED(useTinyCalc);
        if (dist) {
            *dist = mockDistanceCm;
        }
        if (bearing) {
            *bearing = mockBearingCdeg;
        }
    }

    // Simple flat-earth NED conversion relative to mockNavOrigin
    void navOriginLLHtoNED(const gpsLocation_t *llh, vector3_t *ned) {
        if (ned) {
            ned->x = (float)(llh->lat - mockNavOrigin.lat) * EARTH_ANGLE_TO_CM;
            ned->y = (float)(llh->lon - mockNavOrigin.lon) * EARTH_ANGLE_TO_CM;
            ned->z = -(float)(llh->altCm - mockNavOrigin.altCm);
        }
    }

    void l1GuidanceSetPath(const vector2_t *start, const vector2_t *end) {
        UNUSED(start);
        UNUSED(end);
    }

    void l1GuidanceSetArcStart(float courseDeg) {
        UNUSED(courseDeg);
    }

    void l1GuidanceSetActive(bool active) {
        mockL1Active = active;
    }

    bool l1GuidanceIsActive(void) {
        return mockL1Active;
    }

    bool l1GuidanceIsArcActive(void) {
        return false;
    }

    float l1GuidanceGetAlongTrackDistance(void) {
        return 0.0f;
    }

    float l1GuidanceGetPathLength(void) {
        return 0.0f;
    }

    bool navOriginIsValid(void) {
        return true;
    }

    void parseRcChannels(const char *input, rxConfig_t *rxConfig) {
        UNUSED(input);
        UNUSED(rxConfig);
    }

    bool sensors(uint32_t mask) {
        UNUSED(mask);
        return true;
    }

    timeUs_t micros(void) {
        return currentTimeUs;
    }
}
