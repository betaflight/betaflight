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
    #include "build/debug.h"

    #include "common/maths.h"
    #include "common/vector.h"

    #include "fc/core.h"
    #include "fc/runtime_config.h"

    #include "flight/flight_plan_nav.h"
    #include "flight/position_estimator.h"
    #include "flight/position_nav.h"

    #include "io/gps.h"

    #include "pg/autopilot.h"
    #include "pg/flight_plan.h"
    #include "pg/pg.h"

    int16_t debug[DEBUG16_VALUE_COUNT];
    uint8_t debugMode;

    uint8_t stateFlags;
    uint16_t GPS_distanceToHome;
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// --- Stubs and test hooks ---

namespace {

struct CapturedTarget {
    vector3_t targetEfM;
    float cruiseSpeedMps;
    float acceptanceRadiusM;
    float completionSpeedMps;
    bool includeAltitude;
    positionNavReachedCallbackFn callback;
    void *userData;
    bool valid;
};

CapturedTarget g_lastTarget;
int g_setTargetCalls;
int g_clearTargetCalls;

gpsLocation_t g_stubGpsOrigin;
bool g_stubGpsOriginSet;

timeUs_t g_stubMicros;

positionEstimate3d_t g_stubEstimate;
bool g_stubValidXY;
bool g_stubBelowLandingAltitude;

int g_disarmCalls;
flightLogDisarmReason_e g_lastDisarmReason;

} // namespace

extern "C" {

void positionNavSetTargetEf(
    const vector3_t *targetPosEfM,
    float cruiseSpeedMps,
    float acceptanceRadiusM,
    float completionSpeedMps,
    bool includeAltitude,
    positionNavReachedCallbackFn callback,
    void *userData)
{
    g_lastTarget.targetEfM = *targetPosEfM;
    g_lastTarget.cruiseSpeedMps = cruiseSpeedMps;
    g_lastTarget.acceptanceRadiusM = acceptanceRadiusM;
    g_lastTarget.completionSpeedMps = completionSpeedMps;
    g_lastTarget.includeAltitude = includeAltitude;
    g_lastTarget.callback = callback;
    g_lastTarget.userData = userData;
    g_lastTarget.valid = true;
    g_setTargetCalls++;
}

void positionNavClearTarget(void)
{
    g_clearTargetCalls++;
    g_lastTarget.valid = false;
}

void positionNavSetAutoClearOnReach(bool autoClear)
{
    (void)autoClear;
}

bool positionEstimatorGetGpsOrigin(gpsLocation_t *out)
{
    if (!g_stubGpsOriginSet || out == NULL) {
        return false;
    }
    *out = g_stubGpsOrigin;
    return true;
}

float positionEstimatorGetAltitudeCm(void)
{
    return 0.0f;
}

const positionEstimate3d_t *positionEstimatorGetEstimate(void)
{
    return &g_stubEstimate;
}

bool positionEstimatorIsValidXY(void)
{
    return g_stubValidXY;
}

bool positionNavHasActiveTarget(void)
{
    return g_lastTarget.valid;
}

const positionNavCommand_t *positionNavGetActiveCommand(void)
{
    static positionNavCommand_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.active = g_lastTarget.valid;
    cmd.targetPosEfM = g_lastTarget.targetEfM;
    cmd.includeAltitude = g_lastTarget.includeAltitude;
    cmd.cruiseSpeedMps = g_lastTarget.cruiseSpeedMps;
    return &cmd;
}

void disarm(flightLogDisarmReason_e reason)
{
    g_disarmCalls++;
    g_lastDisarmReason = reason;
}

bool isBelowLandingAltitude(void)
{
    return g_stubBelowLandingAltitude;
}

void GPS_distance2d(const gpsLocation_t *from, const gpsLocation_t *to, vector2_t *distance)
{
    // Simplified flat-earth approximation sufficient for unit-test deltas.
    // Matches the axis convention of the real implementation:
    //   x = east (lon delta), y = north (lat delta), both in cm.
    // 1 deg ~= 111319.49 m at the equator; 1e7 lat units per degree.
    const float metresPerLatUnit = 111319.49f / 1.0e7f;
    distance->x = (to->lon - from->lon) * metresPerLatUnit * 100.0f;
    distance->y = (to->lat - from->lat) * metresPerLatUnit * 100.0f;
}

timeUs_t micros(void)
{
    return g_stubMicros;
}

uint32_t millis(void)
{
    return g_stubMicros / 1000;
}

} // extern "C"

class FlightPlanNavTest : public ::testing::Test {
protected:
    void SetUp() override {
        memset(&g_lastTarget, 0, sizeof(g_lastTarget));
        g_setTargetCalls = 0;
        g_clearTargetCalls = 0;
        g_stubMicros = 0;

        memset(&g_stubEstimate, 0, sizeof(g_stubEstimate));
        g_stubValidXY = true;
        g_stubBelowLandingAltitude = true;
        g_disarmCalls = 0;

        stateFlags = 0;
        GPS_distanceToHome = 0;

        // Default GPS origin: equator, prime meridian, 100 m AMSL.
        g_stubGpsOrigin.lat = 0;
        g_stubGpsOrigin.lon = 0;
        g_stubGpsOrigin.altCm = 10000;
        g_stubGpsOriginSet = true;

        flightPlanConfig_t *plan = flightPlanConfigMutable();
        memset(plan, 0, sizeof(*plan));

        autopilotConfig_t *cfg = autopilotConfigMutable();
        memset(cfg, 0, sizeof(*cfg));
        cfg->waypointArrivalRadius = 500;  // 5 m
        cfg->maxVelocity = 1000;           // 10 m/s
        cfg->landingDescentRate = 50;      // 0.5 m/s
        cfg->landingDetectionTime = 10;    // 1 s
        cfg->landingVelocityThreshold = 50; // 0.5 m/s

        flightPlanNavInit();
    }

    void addWaypoint(int32_t lat, int32_t lon, int32_t altCm,
                     uint8_t type, uint16_t speed = 0, uint16_t duration = 0,
                     uint8_t pattern = WAYPOINT_PATTERN_ORBIT)
    {
        flightPlanConfig_t *plan = flightPlanConfigMutable();
        waypoint_t *wp = &plan->waypoints[plan->waypointCount++];
        wp->latitude = lat;
        wp->longitude = lon;
        wp->altitude = altCm;
        wp->type = type;
        wp->speed = speed;
        wp->duration = duration;
        wp->pattern = pattern;
    }

    void triggerReached() {
        ASSERT_NE(g_lastTarget.callback, nullptr);
        g_lastTarget.callback(g_lastTarget.userData);
    }
};

TEST_F(FlightPlanNavTest, EmptyPlanGoesStraightToComplete)
{
    flightPlanNavEngage();
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_COMPLETE);
    EXPECT_EQ(g_setTargetCalls, 0);
}

TEST_F(FlightPlanNavTest, NoGpsOriginLeavesStateIdle)
{
    g_stubGpsOriginSet = false;
    addWaypoint(100, 200, 15000, WAYPOINT_TYPE_FLYOVER);
    flightPlanNavEngage();
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_IDLE);
    EXPECT_EQ(g_setTargetCalls, 0);
}

TEST_F(FlightPlanNavTest, EngageSetsFirstWaypointTarget)
{
    // Origin at (0,0,100m). Waypoint 10m east, 20m north, 50m AMSL.
    // ENU target should be (10, 20, 50 - 100 = -50) m.
    const int32_t lonUnitsFor10m = (int32_t)((10.0f / 111319.49f) * 1.0e7f);
    const int32_t latUnitsFor20m = (int32_t)((20.0f / 111319.49f) * 1.0e7f);
    addWaypoint(latUnitsFor20m, lonUnitsFor10m, 5000, WAYPOINT_TYPE_FLYOVER, 300 /* 3 m/s */);

    flightPlanNavEngage();

    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
    EXPECT_EQ(flightPlanNavGetCurrentIndex(), 0);
    EXPECT_EQ(g_setTargetCalls, 1);
    ASSERT_TRUE(g_lastTarget.valid);
    EXPECT_NEAR(g_lastTarget.targetEfM.x, 10.0f, 0.1f);
    EXPECT_NEAR(g_lastTarget.targetEfM.y, 20.0f, 0.1f);
    EXPECT_NEAR(g_lastTarget.targetEfM.z, -50.0f, 0.01f);
    EXPECT_NEAR(g_lastTarget.cruiseSpeedMps, 3.0f, 0.01f);
    EXPECT_NEAR(g_lastTarget.acceptanceRadiusM, 5.0f, 0.01f);
    EXPECT_TRUE(g_lastTarget.includeAltitude);
}

TEST_F(FlightPlanNavTest, FallsBackToMaxVelocityWhenWaypointSpeedZero)
{
    addWaypoint(10, 20, 15000, WAYPOINT_TYPE_FLYOVER);
    flightPlanNavEngage();
    ASSERT_TRUE(g_lastTarget.valid);
    // maxVelocity is 1000 cm/s = 10 m/s.
    EXPECT_NEAR(g_lastTarget.cruiseSpeedMps, 10.0f, 0.01f);
}

TEST_F(FlightPlanNavTest, FlybyUsesHigherCompletionSpeedThanFlyover)
{
    addWaypoint(10, 20, 15000, WAYPOINT_TYPE_FLYOVER);
    flightPlanNavEngage();
    const float flyoverCompletion = g_lastTarget.completionSpeedMps;

    // Reset and add a FLYBY.
    flightPlanConfigMutable()->waypointCount = 0;
    addWaypoint(10, 20, 15000, WAYPOINT_TYPE_FLYBY);
    flightPlanNavEngage();
    const float flybyCompletion = g_lastTarget.completionSpeedMps;

    EXPECT_GT(flybyCompletion, flyoverCompletion);
}

TEST_F(FlightPlanNavTest, WaypointReachedAdvancesToNext)
{
    addWaypoint(10, 20, 15000, WAYPOINT_TYPE_FLYOVER);
    addWaypoint(30, 40, 20000, WAYPOINT_TYPE_FLYOVER);

    flightPlanNavEngage();
    EXPECT_EQ(flightPlanNavGetCurrentIndex(), 0);
    EXPECT_EQ(g_setTargetCalls, 1);

    triggerReached();

    EXPECT_EQ(flightPlanNavGetCurrentIndex(), 1);
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
    EXPECT_EQ(g_setTargetCalls, 2);
}

TEST_F(FlightPlanNavTest, ReachingLastWaypointCompletes)
{
    addWaypoint(10, 20, 15000, WAYPOINT_TYPE_FLYOVER);

    flightPlanNavEngage();
    triggerReached();

    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_COMPLETE);
    EXPECT_GE(g_clearTargetCalls, 1);
}

TEST_F(FlightPlanNavTest, HoldWithDurationEntersHoldingThenAdvances)
{
    addWaypoint(10, 20, 15000, WAYPOINT_TYPE_HOLD, 0, 20 /* 2.0 s */);
    addWaypoint(30, 40, 15000, WAYPOINT_TYPE_FLYOVER);

    g_stubMicros = 1'000'000;
    flightPlanNavEngage();
    triggerReached();

    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_HOLDING);
    EXPECT_EQ(g_setTargetCalls, 1);

    // Tick before timer expires — still holding.
    g_stubMicros += 1'000'000; // +1 s
    flightPlanNavUpdate(g_stubMicros);
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_HOLDING);

    // Tick past expiry — advances.
    g_stubMicros += 1'500'000; // +2.5 s total
    flightPlanNavUpdate(g_stubMicros);
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
    EXPECT_EQ(flightPlanNavGetCurrentIndex(), 1);
    EXPECT_EQ(g_setTargetCalls, 2);
}

TEST_F(FlightPlanNavTest, HoldWithZeroDurationAdvancesImmediately)
{
    addWaypoint(10, 20, 15000, WAYPOINT_TYPE_HOLD, 0, 0);
    addWaypoint(30, 40, 15000, WAYPOINT_TYPE_FLYOVER);

    flightPlanNavEngage();
    triggerReached();

    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
    EXPECT_EQ(flightPlanNavGetCurrentIndex(), 1);
}

TEST_F(FlightPlanNavTest, DisengageClearsTargetAndResetsState)
{
    addWaypoint(10, 20, 15000, WAYPOINT_TYPE_FLYOVER);
    flightPlanNavEngage();
    ASSERT_TRUE(flightPlanNavIsActive());

    flightPlanNavDisengage();

    EXPECT_FALSE(flightPlanNavIsActive());
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_IDLE);
    EXPECT_GE(g_clearTargetCalls, 1);
}

TEST_F(FlightPlanNavTest, DisengagedUpdateIsNoOp)
{
    addWaypoint(10, 20, 15000, WAYPOINT_TYPE_HOLD, 0, 20);
    flightPlanNavEngage();
    triggerReached();
    ASSERT_EQ(flightPlanNavGetState(), FP_NAV_HOLDING);

    flightPlanNavDisengage();
    g_stubMicros += 10'000'000;
    flightPlanNavUpdate(g_stubMicros);

    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_IDLE);
}

TEST_F(FlightPlanNavTest, ReEngageRestartsAtFirstWaypoint)
{
    addWaypoint(10, 20, 15000, WAYPOINT_TYPE_FLYOVER);
    addWaypoint(30, 40, 15000, WAYPOINT_TYPE_FLYOVER);

    flightPlanNavEngage();
    triggerReached();
    ASSERT_EQ(flightPlanNavGetCurrentIndex(), 1);

    flightPlanNavDisengage();
    flightPlanNavEngage();

    EXPECT_EQ(flightPlanNavGetCurrentIndex(), 0);
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
}

TEST_F(FlightPlanNavTest, EngageClampsBelowMinCruiseSpeed)
{
    // maxVelocity small enough that 1 m/s floor should kick in.
    autopilotConfigMutable()->maxVelocity = 20; // 0.2 m/s
    addWaypoint(10, 20, 15000, WAYPOINT_TYPE_FLYOVER);

    flightPlanNavEngage();
    EXPECT_GE(g_lastTarget.cruiseSpeedMps, 1.0f - 0.01f);
}

// --- Safety behaviour ---

class FlightPlanNavSafetyTest : public FlightPlanNavTest {
protected:
    // A leg to a target ~2.2 km away so sanity margins have room to act.
    void engageDistantLeg() {
        addWaypoint(200000, 0, 15000, WAYPOINT_TYPE_FLYOVER); // ~2.2 km north
        g_stubMicros = 1'000'000;
        flightPlanNavEngage();
        ASSERT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
        // First update captures the initial best distance.
        flightPlanNavUpdate(g_stubMicros);
        ASSERT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
    }
};

TEST_F(FlightPlanNavSafetyTest, EstimatorInvalidAbortsMission)
{
    engageDistantLeg();

    g_stubValidXY = false;
    flightPlanNavUpdate(g_stubMicros + 10'000);

    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_ABORTED);
    EXPECT_EQ(flightPlanNavGetAbortReason(), FP_ABORT_ESTIMATOR);
    EXPECT_GE(g_clearTargetCalls, 1);
}

TEST_F(FlightPlanNavSafetyTest, NoProgressForStallWindowAborts)
{
    engageDistantLeg();

    // Vehicle parked: no movement toward the target.
    g_stubMicros += 29'000'000;
    flightPlanNavUpdate(g_stubMicros);
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);

    g_stubMicros += 2'000'000;
    flightPlanNavUpdate(g_stubMicros);
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_ABORTED);
    EXPECT_EQ(flightPlanNavGetAbortReason(), FP_ABORT_STALLED);
}

TEST_F(FlightPlanNavSafetyTest, ProgressResetsStallWindow)
{
    engageDistantLeg();

    // Move 100 m toward the target just before the stall window expires.
    g_stubMicros += 29'000'000;
    g_stubEstimate.position.v[ENU_N] = 100.0f * 100.0f; // cm
    flightPlanNavUpdate(g_stubMicros);
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);

    // Another near-window with no further progress: still inside the new window.
    g_stubMicros += 29'000'000;
    flightPlanNavUpdate(g_stubMicros);
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
}

TEST_F(FlightPlanNavSafetyTest, MovingAwayPastMarginAbortsAsFlyaway)
{
    engageDistantLeg();

    // Drift 25 m away from the target (south) — beyond the 20 m margin.
    g_stubEstimate.position.v[ENU_N] = -25.0f * 100.0f; // cm
    flightPlanNavUpdate(g_stubMicros + 10'000);

    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_ABORTED);
    EXPECT_EQ(flightPlanNavGetAbortReason(), FP_ABORT_FLYAWAY);
}

TEST_F(FlightPlanNavSafetyTest, AbortReasonClearsOnReEngage)
{
    engageDistantLeg();
    g_stubValidXY = false;
    flightPlanNavUpdate(g_stubMicros + 10'000);
    ASSERT_EQ(flightPlanNavGetAbortReason(), FP_ABORT_ESTIMATOR);

    g_stubValidXY = true;
    flightPlanNavDisengage();
    flightPlanNavEngage();
    EXPECT_EQ(flightPlanNavGetAbortReason(), FP_ABORT_NONE);
}

TEST_F(FlightPlanNavSafetyTest, GeofenceBreachWithLandActionStartsLanding)
{
    autopilotConfigMutable()->maxDistanceFromHomeM = 100;
    autopilotConfigMutable()->geofenceAction = AP_GEOFENCE_LAND;
    stateFlags |= GPS_FIX_HOME;
    engageDistantLeg();

    GPS_distanceToHome = 150;
    g_stubEstimate.position.v[ENU_U] = 30.0f * 100.0f; // 30 m up, in cm
    flightPlanNavUpdate(g_stubMicros + 10'000);

    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_LANDING);
    ASSERT_TRUE(g_lastTarget.valid);
    // Landing target: current position, far below current altitude.
    EXPECT_NEAR(g_lastTarget.targetEfM.z, 30.0f - 200.0f, 0.1f);
    EXPECT_NEAR(g_lastTarget.cruiseSpeedMps, 0.5f, 0.01f);
    EXPECT_FALSE(flightPlanNavRescueRequested());
}

TEST_F(FlightPlanNavSafetyTest, GeofenceInsideLimitDoesNothing)
{
    autopilotConfigMutable()->maxDistanceFromHomeM = 100;
    autopilotConfigMutable()->geofenceAction = AP_GEOFENCE_LAND;
    stateFlags |= GPS_FIX_HOME;
    engageDistantLeg();

    GPS_distanceToHome = 99;
    flightPlanNavUpdate(g_stubMicros + 10'000);

    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
}

TEST_F(FlightPlanNavSafetyTest, GeofenceWithoutHomeFixDoesNothing)
{
    autopilotConfigMutable()->maxDistanceFromHomeM = 100;
    autopilotConfigMutable()->geofenceAction = AP_GEOFENCE_LAND;
    engageDistantLeg();

    GPS_distanceToHome = 150;
    flightPlanNavUpdate(g_stubMicros + 10'000);

    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
}

TEST_F(FlightPlanNavSafetyTest, GeofenceBreachWithRthActionLatchesRescueRequest)
{
    autopilotConfigMutable()->maxDistanceFromHomeM = 100;
    autopilotConfigMutable()->geofenceAction = AP_GEOFENCE_RTH;
    stateFlags |= GPS_FIX_HOME;
    engageDistantLeg();

    GPS_distanceToHome = 150;
    flightPlanNavUpdate(g_stubMicros + 10'000);

    EXPECT_TRUE(flightPlanNavRescueRequested());
    // The request survives disengagement (rescue outranks and disengages the mission).
    flightPlanNavDisengage();
    EXPECT_TRUE(flightPlanNavRescueRequested());

    flightPlanNavClearRescueRequest();
    EXPECT_FALSE(flightPlanNavRescueRequested());
}

TEST_F(FlightPlanNavSafetyTest, LandingTouchdownDisarms)
{
    autopilotConfigMutable()->maxDistanceFromHomeM = 100;
    autopilotConfigMutable()->geofenceAction = AP_GEOFENCE_LAND;
    stateFlags |= GPS_FIX_HOME;
    engageDistantLeg();
    GPS_distanceToHome = 150;
    flightPlanNavUpdate(g_stubMicros + 10'000);
    ASSERT_EQ(flightPlanNavGetState(), FP_NAV_LANDING);

    // Descent establishes (well above the 25% of commanded rate threshold).
    g_stubEstimate.velocity.v[ENU_U] = -40.0f; // cm/s
    g_stubMicros += 1'000'000;
    flightPlanNavUpdate(g_stubMicros);
    EXPECT_EQ(g_disarmCalls, 0);

    // Touchdown: vertical velocity inside the quiet threshold.
    g_stubEstimate.velocity.v[ENU_U] = 0.0f;
    g_stubMicros += 1'000'000;
    flightPlanNavUpdate(g_stubMicros); // starts the quiet timer
    EXPECT_EQ(g_disarmCalls, 0);

    g_stubMicros += 1'100'000; // past landingDetectionTime (1 s)
    flightPlanNavUpdate(g_stubMicros);
    EXPECT_EQ(g_disarmCalls, 1);
    EXPECT_EQ(g_lastDisarmReason, DISARM_REASON_LANDING);
}

TEST_F(FlightPlanNavSafetyTest, LandingAtAltitudeWithoutDescentNeverDisarms)
{
    autopilotConfigMutable()->maxDistanceFromHomeM = 100;
    autopilotConfigMutable()->geofenceAction = AP_GEOFENCE_LAND;
    stateFlags |= GPS_FIX_HOME;
    g_stubBelowLandingAltitude = false; // high up, and unable to descend
    engageDistantLeg();
    GPS_distanceToHome = 150;
    flightPlanNavUpdate(g_stubMicros + 10'000);
    ASSERT_EQ(flightPlanNavGetState(), FP_NAV_LANDING);

    // Descent never establishes; well past establish-timeout + detection time.
    for (int i = 0; i < 15; i++) {
        g_stubMicros += 1'000'000;
        flightPlanNavUpdate(g_stubMicros);
    }
    EXPECT_EQ(g_disarmCalls, 0);
}

TEST_F(FlightPlanNavSafetyTest, GroundLevelLandingFallbackDisarms)
{
    autopilotConfigMutable()->maxDistanceFromHomeM = 100;
    autopilotConfigMutable()->geofenceAction = AP_GEOFENCE_LAND;
    stateFlags |= GPS_FIX_HOME;
    engageDistantLeg();
    GPS_distanceToHome = 150;
    flightPlanNavUpdate(g_stubMicros + 10'000);
    ASSERT_EQ(flightPlanNavGetState(), FP_NAV_LANDING);

    // Near the ground (default stub), no descent possible: the fallback arms
    // touchdown monitoring after the establish timeout and quiet time disarms.
    for (int i = 0; i < 4; i++) {
        g_stubMicros += 1'000'000;
        flightPlanNavUpdate(g_stubMicros);
    }
    EXPECT_EQ(g_disarmCalls, 0); // still inside establish timeout

    for (int i = 0; i < 4; i++) {
        g_stubMicros += 1'000'000;
        flightPlanNavUpdate(g_stubMicros);
    }
    EXPECT_EQ(g_disarmCalls, 1);
    EXPECT_EQ(g_lastDisarmReason, DISARM_REASON_LANDING);
}

TEST_F(FlightPlanNavSafetyTest, LandingQuietTimerResetsIfDescentResumes)
{
    autopilotConfigMutable()->maxDistanceFromHomeM = 100;
    autopilotConfigMutable()->geofenceAction = AP_GEOFENCE_LAND;
    stateFlags |= GPS_FIX_HOME;
    engageDistantLeg();
    GPS_distanceToHome = 150;
    flightPlanNavUpdate(g_stubMicros + 10'000);
    ASSERT_EQ(flightPlanNavGetState(), FP_NAV_LANDING);

    g_stubEstimate.velocity.v[ENU_U] = -40.0f;
    g_stubMicros += 1'000'000;
    flightPlanNavUpdate(g_stubMicros);

    // Brief quiet period, then descent resumes (e.g. drifted off a bush).
    g_stubEstimate.velocity.v[ENU_U] = 0.0f;
    g_stubMicros += 500'000;
    flightPlanNavUpdate(g_stubMicros);
    g_stubEstimate.velocity.v[ENU_U] = -40.0f;
    g_stubMicros += 500'000;
    flightPlanNavUpdate(g_stubMicros);

    // New quiet period must run the full detection time again.
    g_stubEstimate.velocity.v[ENU_U] = 0.0f;
    g_stubMicros += 600'000;
    flightPlanNavUpdate(g_stubMicros); // quiet timer restarts here
    EXPECT_EQ(g_disarmCalls, 0);

    g_stubMicros += 600'000;
    flightPlanNavUpdate(g_stubMicros); // 0.6 s into the new window — too early
    EXPECT_EQ(g_disarmCalls, 0);

    g_stubMicros += 500'000;
    flightPlanNavUpdate(g_stubMicros); // 1.1 s — past detection time
    EXPECT_EQ(g_disarmCalls, 1);
}
