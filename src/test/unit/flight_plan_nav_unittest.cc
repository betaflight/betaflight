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

#include <float.h>
#include <math.h>
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
    #include "flight/imu.h"
    #include "flight/position_estimator.h"
    #include "flight/position_nav.h"

    #include "io/gps.h"

    #include "pg/autopilot.h"
    #include "pg/flight_plan.h"
    #include "pg/gps_rescue.h"
    #include "pg/pg.h"

    int16_t debug[DEBUG16_VALUE_COUNT];
    uint8_t debugMode;

    uint8_t stateFlags;
    uint16_t GPS_distanceToHome;
    gpsSolutionData_t gpsSol;
    gpsLocation_t GPS_home_llh;
    attitudeEulerAngles_t attitude;
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
vector3_t g_lastDispatchTargetEfM;   // destination from positionNavSetTargetEf only (carrot moves don't touch it)
int g_setTargetCalls;
int g_clearTargetCalls;
int g_moveTargetCalls;

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
    g_lastDispatchTargetEfM = *targetPosEfM;
    g_lastTarget.cruiseSpeedMps = cruiseSpeedMps;
    g_lastTarget.acceptanceRadiusM = acceptanceRadiusM;
    g_lastTarget.completionSpeedMps = completionSpeedMps;
    g_lastTarget.includeAltitude = includeAltitude;
    g_lastTarget.callback = callback;
    g_lastTarget.userData = userData;
    g_lastTarget.valid = true;
    g_setTargetCalls++;
}

void positionNavMoveTargetEf(const vector3_t *targetPosEfM)
{
    if (!g_lastTarget.valid) {
        return;
    }
    g_lastTarget.targetEfM = *targetPosEfM;
    g_moveTargetCalls++;
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

void positionNavSetAccelLimits(float maxAccelMps2, float maxDecelMps2)
{
    (void)maxAccelMps2;
    (void)maxDecelMps2;
}

static bool g_altitudeArrivalRequired;

void positionNavSetAltitudeArrivalRequired(bool required)
{
    g_altitudeArrivalRequired = required;
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

static float g_yawRateLimitDps;

void autopilotSetYawRateLimit(float rateLimitDps)
{
    g_yawRateLimitDps = rateLimitDps;
}

static bool g_forceLevelPark;

void autopilotForceLevelPark(bool request)
{
    g_forceLevelPark = request;
}

static bool g_navHeadingOverrideValid;
static float g_navHeadingOverrideDeg;

void autopilotSetNavHeadingOverride(bool valid, float headingDeg)
{
    g_navHeadingOverrideValid = valid;
    g_navHeadingOverrideDeg = headingDeg;
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
        g_moveTargetCalls = 0;
        g_stubMicros = 0;

        memset(&g_stubEstimate, 0, sizeof(g_stubEstimate));
        g_stubValidXY = true;
        g_stubBelowLandingAltitude = true;
        g_altitudeArrivalRequired = false;
        g_disarmCalls = 0;
        g_yawRateLimitDps = -1.0f; // sentinel: no autopilotSetYawRateLimit call yet
        g_forceLevelPark = false;
        g_navHeadingOverrideValid = false;
        g_navHeadingOverrideDeg = 0.0f;
        memset(&g_lastDispatchTargetEfM, 0, sizeof(g_lastDispatchTargetEfM));

        memset(&attitude, 0, sizeof(attitude));

        stateFlags = 0;
        GPS_distanceToHome = 0;

        // Default GPS origin: equator, prime meridian, 100 m AMSL.
        g_stubGpsOrigin.lat = 0;
        g_stubGpsOrigin.lon = 0;
        g_stubGpsOrigin.altCm = 10000;
        g_stubGpsOriginSet = true;

        // Home at the origin; current GPS altitude at home altitude.
        memset(&GPS_home_llh, 0, sizeof(GPS_home_llh));
        GPS_home_llh.altCm = 10000;
        memset(&gpsSol, 0, sizeof(gpsSol));
        gpsSol.llh.altCm = 10000;

        gpsRescueConfig_t *rescueCfg = gpsRescueConfigMutable();
        memset(rescueCfg, 0, sizeof(*rescueCfg));
        rescueCfg->returnAltitudeM = 30;
        rescueCfg->groundSpeedCmS = 750;

        flightPlanConfig_t *plan = flightPlanConfigMutable();
        memset(plan, 0, sizeof(*plan));

        autopilotConfig_t *cfg = autopilotConfigMutable();
        memset(cfg, 0, sizeof(*cfg));
        cfg->waypointArrivalRadius = 500;  // 5 m
        cfg->waypointHoldRadius = 200;     // 2 m
        cfg->maxVelocity = 1000;           // 10 m/s
        cfg->landingDescentRate = 50;      // 0.5 m/s
        cfg->landingDetectionTime = 10;    // 1 s
        cfg->landingVelocityThreshold = 50; // 0.5 m/s
        // Leg-line carrot tracking (PG reset template is not applied under test).
        cfg->navCornerSpeed = 220;         // 2.2 m/s floor
        cfg->navCornerDeltaV = 440;        // 4.4 m/s per-corner budget
        cfg->navDecel = 250;               // 2.5 m/s^2
        cfg->navAccel = 250;               // 2.5 m/s^2
        cfg->navCarrotLeadTime = 12;       // 1.2 s
        cfg->navCarrotLeadMax = 2500;      // 25 m
        cfg->navPreturnDist = 1500;        // 15 m

        flightPlanNavInit();
    }

    void TearDown() override {
        flightPlanNavSetReachedListener(nullptr);
    }

    static waypoint_t makeWaypoint(int32_t lat, int32_t lon, int32_t altCm,
                                   uint8_t type, uint16_t speed = 0, uint16_t duration = 0)
    {
        waypoint_t wp = {};
        wp.latitude = lat;
        wp.longitude = lon;
        wp.altitude = altCm;
        wp.speed = speed;
        wp.duration = duration;
        wp.type = type;
        wp.pattern = WAYPOINT_PATTERN_NONE;
        return wp;
    }

    void addWaypoint(int32_t lat, int32_t lon, int32_t altCm,
                     uint8_t type, uint16_t speed = 0, uint16_t duration = 0,
                     uint8_t pattern = WAYPOINT_PATTERN_NONE)
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

    // Advance the current leg. Precise point legs (last waypoint, station-keeping,
    // injected LAND) complete via the positionNav callback; en-route pass-through
    // legs have no callback and advance through the executor's carrot gate on a
    // position update. The default test waypoints sit within a metre of the
    // origin, so a craft parked at the origin is already inside any gate radius.
    void arriveAtWaypoint() {
        if (g_lastTarget.callback != nullptr) {
            g_lastTarget.callback(g_lastTarget.userData);
        } else {
            g_stubMicros += 100'000;
            flightPlanNavUpdate(g_stubMicros);
        }
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

TEST_F(FlightPlanNavTest, LegsCompleteOnRadiusEntryAtAnySpeed)
{
    addWaypoint(10, 20, 15000, WAYPOINT_TYPE_FLYOVER);
    flightPlanNavEngage();
    // The completion speed must never gate arrival (see FP_COMPLETION_ANY_MPS).
    EXPECT_GT(g_lastTarget.completionSpeedMps, 100.0f);
}

TEST_F(FlightPlanNavTest, WaypointReachedAdvancesToNext)
{
    addWaypoint(10, 20, 15000, WAYPOINT_TYPE_FLYOVER);
    addWaypoint(30, 40, 20000, WAYPOINT_TYPE_FLYOVER);

    flightPlanNavEngage();
    EXPECT_EQ(flightPlanNavGetCurrentIndex(), 0);
    EXPECT_EQ(g_setTargetCalls, 1);

    arriveAtWaypoint();

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

TEST_F(FlightPlanNavTest, TakeoffDispatchClimbsInPlace)
{
    // Vehicle at (40 E, 30 N) m; TAKEOFF waypoint 100 m away horizontally.
    g_stubEstimate.position.x = 4000.0f;
    g_stubEstimate.position.y = 3000.0f;
    const int32_t unitsFor100m = (int32_t)((100.0f / 111319.49f) * 1.0e7f);
    addWaypoint(unitsFor100m, unitsFor100m, 15000, WAYPOINT_TYPE_TAKEOFF);

    flightPlanNavEngage();

    // Target is the current position at the waypoint altitude, with the
    // station-keeping radius and the climb gated on altitude arrival.
    ASSERT_TRUE(g_lastTarget.valid);
    EXPECT_NEAR(g_lastTarget.targetEfM.x, 40.0f, 0.01f);
    EXPECT_NEAR(g_lastTarget.targetEfM.y, 30.0f, 0.01f);
    EXPECT_NEAR(g_lastTarget.targetEfM.z, 50.0f, 0.01f);
    EXPECT_NEAR(g_lastTarget.acceptanceRadiusM, 2.0f, 0.01f);
    EXPECT_TRUE(g_altitudeArrivalRequired);
}

TEST_F(FlightPlanNavTest, TakeoffAdvancesOnArrival)
{
    addWaypoint(0, 0, 15000, WAYPOINT_TYPE_TAKEOFF);
    addWaypoint(30, 40, 15000, WAYPOINT_TYPE_FLYOVER);

    flightPlanNavEngage();
    triggerReached();

    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
    EXPECT_EQ(flightPlanNavGetCurrentIndex(), 1);
}

TEST_F(FlightPlanNavTest, TakeoffWithDurationLoitersThenAdvancesWithoutPattern)
{
    // A pattern on a TAKEOFF waypoint is meaningless and must not start.
    addWaypoint(0, 0, 15000, WAYPOINT_TYPE_TAKEOFF, 0, 20 /* 2.0 s */, WAYPOINT_PATTERN_ORBIT);
    addWaypoint(30, 40, 15000, WAYPOINT_TYPE_FLYOVER);

    g_stubMicros = 1'000'000;
    flightPlanNavEngage();
    triggerReached();

    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_HOLDING);
    EXPECT_EQ(g_setTargetCalls, 1);   // no carrot command issued

    g_stubMicros += 1'000'000;
    flightPlanNavUpdate(g_stubMicros);
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_HOLDING);
    EXPECT_EQ(g_moveTargetCalls, 0);

    g_stubMicros += 1'500'000;
    flightPlanNavUpdate(g_stubMicros);
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
    EXPECT_EQ(flightPlanNavGetCurrentIndex(), 1);
}

TEST_F(FlightPlanNavTest, HoldPatternNoneKeepsStationTarget)
{
    addWaypoint(10, 20, 15000, WAYPOINT_TYPE_HOLD, 0, 100 /* 10 s */);

    g_stubMicros = 1'000'000;
    flightPlanNavEngage();
    const vector3_t holdTarget = g_lastTarget.targetEfM;
    triggerReached();

    for (int i = 0; i < 5; i++) {
        g_stubMicros += 1'000'000;
        flightPlanNavUpdate(g_stubMicros);
    }

    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_HOLDING);
    EXPECT_EQ(g_setTargetCalls, 1);
    EXPECT_EQ(g_moveTargetCalls, 0);
    EXPECT_EQ(memcmp(&g_lastTarget.targetEfM, &holdTarget, sizeof(holdTarget)), 0);
}

class FlightPlanNavPatternTest : public FlightPlanNavTest {
protected:
    // HOLD waypoint at (10 E, 20 N, +50 U) m, leg cruise 3 m/s. With the
    // default 2 m hold radius the carrot rate cap gives 0.5 m/s path speed,
    // so the phase advances at 0.25 rad/s.
    static constexpr float kCentreE = 10.0f;
    static constexpr float kCentreN = 20.0f;
    static constexpr float kCentreU = 50.0f;
    static constexpr float kRadiusM = 2.0f;

    void engageHoldPattern(uint8_t pattern, uint16_t durationDs = 200)
    {
        const int32_t lonUnitsFor10m = (int32_t)((10.0f / 111319.49f) * 1.0e7f);
        const int32_t latUnitsFor20m = (int32_t)((20.0f / 111319.49f) * 1.0e7f);
        addWaypoint(latUnitsFor20m, lonUnitsFor10m, 15000, WAYPOINT_TYPE_HOLD,
                    300 /* 3 m/s */, durationDs, pattern);
        addWaypoint(30, 40, 15000, WAYPOINT_TYPE_FLYOVER);

        g_stubMicros = 1'000'000;
        flightPlanNavEngage();
    }

    float distanceFromCentre() const
    {
        return sqrtf(sq(g_lastTarget.targetEfM.x - kCentreE) + sq(g_lastTarget.targetEfM.y - kCentreN));
    }
};

TEST_F(FlightPlanNavPatternTest, OrbitIssuesNonCompletingCarrotCommand)
{
    engageHoldPattern(WAYPOINT_PATTERN_ORBIT);
    // Vehicle slightly east of the centre: carrot starts at azimuth 0.
    g_stubEstimate.position.x = (kCentreE + 1.5f) * 100.0f;
    g_stubEstimate.position.y = kCentreN * 100.0f;
    triggerReached();

    // Arrival alone must not start the pattern; the update loop starts it
    // once the arrival braking has settled (stub estimate is at rest).
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_HOLDING);
    EXPECT_EQ(g_setTargetCalls, 1);
    flightPlanNavUpdate(g_stubMicros);
    ASSERT_EQ(g_setTargetCalls, 2);
    // The carrot command must never complete: completion would refire the
    // arrival callback (restarting the hold) and zero the velocity target.
    EXPECT_EQ(g_lastTarget.completionSpeedMps, 0.0f);
    EXPECT_EQ(g_lastTarget.callback, nullptr);
    EXPECT_NEAR(g_lastTarget.cruiseSpeedMps, 3.0f, 0.01f);
    EXPECT_NEAR(g_lastTarget.targetEfM.x, kCentreE + kRadiusM, 0.05f);
    EXPECT_NEAR(g_lastTarget.targetEfM.y, kCentreN, 0.05f);
    EXPECT_NEAR(g_lastTarget.targetEfM.z, kCentreU, 0.01f);
}

TEST_F(FlightPlanNavPatternTest, OrbitCarrotStartsAtVehicleAzimuth)
{
    engageHoldPattern(WAYPOINT_PATTERN_ORBIT);
    // Vehicle west of the centre: the first carrot must be on the west side
    // of the ring (no dash across the circle).
    g_stubEstimate.position.x = (kCentreE - 2.0f) * 100.0f;
    g_stubEstimate.position.y = kCentreN * 100.0f;
    triggerReached();
    flightPlanNavUpdate(g_stubMicros);

    EXPECT_NEAR(g_lastTarget.targetEfM.x, kCentreE - kRadiusM, 0.05f);
    EXPECT_NEAR(g_lastTarget.targetEfM.y, kCentreN, 0.05f);
}

TEST_F(FlightPlanNavPatternTest, OrbitStartDeferredUntilArrivalBrakingSettles)
{
    engageHoldPattern(WAYPOINT_PATTERN_ORBIT);
    g_stubEstimate.position.x = (kCentreE + 1.5f) * 100.0f;
    g_stubEstimate.position.y = kCentreN * 100.0f;
    g_stubEstimate.velocity.x = 400.0f;  // still carrying 4 m/s of leg momentum
    triggerReached();

    g_stubMicros += 1'000'000;
    flightPlanNavUpdate(g_stubMicros);
    EXPECT_EQ(g_setTargetCalls, 1);      // too fast: no carrot command yet

    g_stubEstimate.velocity.x = 0.0f;    // braking has parked the vehicle
    g_stubMicros += 1'000'000;
    flightPlanNavUpdate(g_stubMicros);
    EXPECT_EQ(g_setTargetCalls, 2);
    EXPECT_EQ(g_lastTarget.completionSpeedMps, 0.0f);
}

TEST_F(FlightPlanNavPatternTest, OrbitStartTimeoutOverridesSettleGate)
{
    engageHoldPattern(WAYPOINT_PATTERN_ORBIT);
    g_stubEstimate.position.x = (kCentreE + 1.5f) * 100.0f;
    g_stubEstimate.position.y = kCentreN * 100.0f;
    g_stubEstimate.velocity.x = 400.0f;  // never settles (e.g. holding against wind)
    triggerReached();

    g_stubMicros += 2'000'000;
    flightPlanNavUpdate(g_stubMicros);
    EXPECT_EQ(g_setTargetCalls, 1);

    g_stubMicros += 4'000'000;           // 6 s since arrival: past the 5 s timeout
    flightPlanNavUpdate(g_stubMicros);
    EXPECT_EQ(g_setTargetCalls, 2);
}

TEST_F(FlightPlanNavPatternTest, OrbitCarrotTracksCircleAroundHoldPoint)
{
    engageHoldPattern(WAYPOINT_PATTERN_ORBIT);
    g_stubEstimate.position.x = (kCentreE + 1.5f) * 100.0f;
    g_stubEstimate.position.y = kCentreN * 100.0f;
    triggerReached();
    flightPlanNavUpdate(g_stubMicros);   // settled: pattern starts

    // 0.25 rad/s: after 1 s the azimuth is 0.25 rad, after 2 s 0.5 rad.
    float previousAzimuth = 0.0f;
    for (int step = 1; step <= 2; step++) {
        g_stubMicros += 1'000'000;
        flightPlanNavUpdate(g_stubMicros);

        EXPECT_EQ(g_moveTargetCalls, step);
        EXPECT_NEAR(distanceFromCentre(), kRadiusM, 0.01f);
        EXPECT_NEAR(g_lastTarget.targetEfM.z, kCentreU, 0.01f);
        const float azimuth = atan2f(g_lastTarget.targetEfM.y - kCentreN,
                                     g_lastTarget.targetEfM.x - kCentreE);
        EXPECT_NEAR(azimuth, step * 0.25f, 0.05f);
        EXPECT_GT(azimuth, previousAzimuth);
        previousAzimuth = azimuth;
    }
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_HOLDING);
}

TEST_F(FlightPlanNavPatternTest, Figure8CarrotBoundedAndRecrossesCentre)
{
    engageHoldPattern(WAYPOINT_PATTERN_FIGURE8, 400 /* 40 s */);
    g_stubEstimate.position.x = kCentreE * 100.0f;
    g_stubEstimate.position.y = kCentreN * 100.0f;
    triggerReached();
    flightPlanNavUpdate(g_stubMicros);   // settled: pattern starts

    // The lemniscate starts at the centre, where the vehicle already is.
    EXPECT_NEAR(distanceFromCentre(), 0.0f, 0.01f);

    // Walk a full cycle (2π at 0.25 rad/s ≈ 25 s) in 1 s steps: the carrot
    // stays within the hold radius and returns near the centre mid-cycle.
    float maxDistance = 0.0f;
    float minDistanceAfterLeaving = FLT_MAX;
    bool leftCentre = false;
    for (int step = 0; step < 26; step++) {
        g_stubMicros += 1'000'000;
        flightPlanNavUpdate(g_stubMicros);

        const float distance = distanceFromCentre();
        EXPECT_LT(distance, kRadiusM + 0.01f);
        maxDistance = fmaxf(maxDistance, distance);
        if (distance > 1.5f) {
            leftCentre = true;
        } else if (leftCentre) {
            minDistanceAfterLeaving = fminf(minDistanceAfterLeaving, distance);
        }
    }
    EXPECT_GT(maxDistance, 1.9f);
    EXPECT_LT(minDistanceAfterLeaving, 0.5f);
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_HOLDING);
}

TEST_F(FlightPlanNavPatternTest, PatternCommandReissuedAfterWipedNavCommand)
{
    engageHoldPattern(WAYPOINT_PATTERN_ORBIT);
    g_stubEstimate.position.x = (kCentreE + 1.5f) * 100.0f;
    g_stubEstimate.position.y = kCentreN * 100.0f;
    triggerReached();
    flightPlanNavUpdate(g_stubMicros);   // settled: pattern starts
    ASSERT_EQ(g_setTargetCalls, 2);

    // A position-control re-init wipes the nav command mid-hold.
    g_lastTarget.valid = false;

    g_stubMicros += 1'000'000;
    flightPlanNavUpdate(g_stubMicros);

    EXPECT_EQ(g_setTargetCalls, 3);
    ASSERT_TRUE(g_lastTarget.valid);
    EXPECT_EQ(g_lastTarget.completionSpeedMps, 0.0f);
    EXPECT_NEAR(distanceFromCentre(), kRadiusM, 0.01f);
}

TEST_F(FlightPlanNavPatternTest, PatternClearedOnAdvance)
{
    engageHoldPattern(WAYPOINT_PATTERN_ORBIT, 20 /* 2.0 s */);
    g_stubEstimate.position.x = (kCentreE + 1.5f) * 100.0f;
    g_stubEstimate.position.y = kCentreN * 100.0f;
    triggerReached();
    flightPlanNavUpdate(g_stubMicros);   // settled: pattern starts

    // Expire the hold: the next leg dispatches and the carrot stops.
    g_stubMicros += 2'500'000;
    flightPlanNavUpdate(g_stubMicros);
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
    EXPECT_EQ(flightPlanNavGetCurrentIndex(), 1);

    const vector3_t legTarget = g_lastTarget.targetEfM;
    const int movesAtAdvance = g_moveTargetCalls;
    g_stubMicros += 1'000'000;
    flightPlanNavUpdate(g_stubMicros);

    EXPECT_EQ(g_moveTargetCalls, movesAtAdvance);
    EXPECT_EQ(memcmp(&g_lastTarget.targetEfM, &legTarget, sizeof(legTarget)), 0);
}

TEST_F(FlightPlanNavTest, OrbitPeriodMatchesRateCapAndCruiseLimit)
{
    // Default 2 m radius: the 0.25 rad/s rate cap dominates -> 2π/0.25 ≈ 25.1 s
    EXPECT_EQ(flightPlanNavOrbitPeriodDs(0), 251);
    // 10 m radius, 1 m/s leg: cruise-limited -> 0.1 rad/s -> 2π/0.1 ≈ 62.8 s
    autopilotConfigMutable()->waypointHoldRadius = 1000;
    EXPECT_EQ(flightPlanNavOrbitPeriodDs(100), 628);
}

TEST_F(FlightPlanNavTest, SetCurrentIndexReTargetsWhileActive)
{
    addWaypoint(10, 20, 15000, WAYPOINT_TYPE_FLYOVER);
    addWaypoint(30, 40, 15000, WAYPOINT_TYPE_FLYOVER);
    addWaypoint(50, 60, 15000, WAYPOINT_TYPE_FLYOVER);

    flightPlanNavEngage();
    EXPECT_EQ(flightPlanNavGetCurrentIndex(), 0);
    EXPECT_EQ(g_setTargetCalls, 1);

    flightPlanNavSetCurrentIndex(2);

    EXPECT_EQ(flightPlanNavGetCurrentIndex(), 2);
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
    EXPECT_EQ(g_setTargetCalls, 2);
}

TEST_F(FlightPlanNavTest, SetCurrentIndexStoresStartIndexWhileIdle)
{
    addWaypoint(10, 20, 15000, WAYPOINT_TYPE_FLYOVER);
    addWaypoint(30, 40, 15000, WAYPOINT_TYPE_FLYOVER);

    // Set before engage: no dispatch happens yet.
    flightPlanNavSetCurrentIndex(1);
    EXPECT_EQ(g_setTargetCalls, 0);

    flightPlanNavEngage();

    EXPECT_EQ(flightPlanNavGetCurrentIndex(), 1);
    EXPECT_EQ(g_setTargetCalls, 1);
}

TEST_F(FlightPlanNavTest, SetCurrentIndexIgnoresOutOfRange)
{
    addWaypoint(10, 20, 15000, WAYPOINT_TYPE_FLYOVER);
    addWaypoint(30, 40, 15000, WAYPOINT_TYPE_FLYOVER);

    flightPlanNavEngage();
    flightPlanNavSetCurrentIndex(5); // >= count

    EXPECT_EQ(flightPlanNavGetCurrentIndex(), 0);
    EXPECT_EQ(g_setTargetCalls, 1);
}

TEST_F(FlightPlanNavTest, GeometryAccessorsSentinelWhenIdle)
{
    addWaypoint(10, 20, 15000, WAYPOINT_TYPE_FLYOVER);
    // Not engaged -> no active target.
    EXPECT_LT(flightPlanNavGetDistanceToWaypointM(), 0.0f);
    EXPECT_LT(flightPlanNavGetBearingToWaypointDeciDeg(), 0);
    EXPECT_EQ(flightPlanNavGetEtaSeconds(), 0);
}

TEST_F(FlightPlanNavTest, GeometryAccessorsReflectActiveTarget)
{
    // Waypoint 10 m east, 20 m north, altitude matching the vehicle so the
    // 3D distance is purely horizontal.
    const int32_t lonUnitsFor10m = (int32_t)((10.0f / 111319.49f) * 1.0e7f);
    const int32_t latUnitsFor20m = (int32_t)((20.0f / 111319.49f) * 1.0e7f);
    addWaypoint(latUnitsFor20m, lonUnitsFor10m, 5000, WAYPOINT_TYPE_FLYOVER, 300);

    flightPlanNavEngage();
    ASSERT_TRUE(g_lastTarget.valid);

    // Vehicle at origin, altitude aligned with the target (-50 m ENU up).
    g_stubEstimate.position.v[0] = 0.0f;      // east cm
    g_stubEstimate.position.v[1] = 0.0f;      // north cm
    g_stubEstimate.position.v[2] = -5000.0f;  // up cm (matches target z)
    g_stubEstimate.velocity.v[0] = 500.0f;    // 5 m/s east
    g_stubEstimate.velocity.v[1] = 0.0f;

    EXPECT_NEAR(flightPlanNavGetDistanceToWaypointM(), 22.36f, 0.1f);
    // atan2(E=10, N=20) = 26.57 deg CW from north -> 266 deci-degrees.
    EXPECT_NEAR(flightPlanNavGetBearingToWaypointDeciDeg(), 266, 2);
    // 22.36 m / 5 m/s = 4.47 s -> rounds to 4.
    EXPECT_EQ(flightPlanNavGetEtaSeconds(), 4);
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
    arriveAtWaypoint();
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

TEST_F(FlightPlanNavTest, YawRateModifierCapsAutopilotYawAndPersists)
{
    addWaypoint(0, 0, 0, WAYPOINT_TYPE_YAW_RATE, 45 /* deg/s */);
    addWaypoint(10, 20, 15000, WAYPOINT_TYPE_FLYOVER);
    addWaypoint(30, 40, 15000, WAYPOINT_TYPE_FLYOVER);

    flightPlanNavEngage();
    ASSERT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
    EXPECT_FLOAT_EQ(g_yawRateLimitDps, 45.0f);

    // The cap applies from the modifier onward, not just the next leg.
    arriveAtWaypoint();
    ASSERT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
    EXPECT_FLOAT_EQ(g_yawRateLimitDps, 45.0f);
}

TEST_F(FlightPlanNavTest, YawRateCapClearsOnDisengageAndEngage)
{
    addWaypoint(0, 0, 0, WAYPOINT_TYPE_YAW_RATE, 45);
    addWaypoint(10, 20, 15000, WAYPOINT_TYPE_FLYOVER);

    flightPlanNavEngage();
    EXPECT_FLOAT_EQ(g_yawRateLimitDps, 45.0f);

    flightPlanNavDisengage();
    EXPECT_FLOAT_EQ(g_yawRateLimitDps, 0.0f);
}

// --- Injected runtime plans ---

namespace {
int g_reachedCalls;
void recordReached(uint8_t index) { (void)index; g_reachedCalls++; }
} // namespace

TEST_F(FlightPlanNavTest, InjectedPlanReplacesMissionAndRunsToTermination)
{
    // Single-waypoint PG mission with a yaw-rate cap staged: injection must
    // clear the cap and must not be truncated by the shorter PG count.
    addWaypoint(0, 0, 0, WAYPOINT_TYPE_YAW_RATE, 45);
    addWaypoint(10, 20, 15000, WAYPOINT_TYPE_FLYOVER);
    flightPlanNavEngage();
    ASSERT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
    EXPECT_FLOAT_EQ(g_yawRateLimitDps, 45.0f);
    const int callsBefore = g_setTargetCalls;

    const int32_t lonUnitsFor10m = (int32_t)((10.0f / 111319.49f) * 1.0e7f);
    const waypoint_t plan[] = {
        makeWaypoint(0, lonUnitsFor10m, 12000, WAYPOINT_TYPE_FLYOVER),
        makeWaypoint(0, lonUnitsFor10m, 12000, WAYPOINT_TYPE_LAND),
    };
    ASSERT_TRUE(flightPlanNavInjectPlan(plan, 2));

    EXPECT_TRUE(flightPlanNavIsInjectedPlanActive());
    EXPECT_EQ(flightPlanNavGetCurrentIndex(), 0);
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
    EXPECT_EQ(g_setTargetCalls, callsBefore + 1);
    EXPECT_FLOAT_EQ(g_yawRateLimitDps, 0.0f);
    ASSERT_TRUE(g_lastTarget.valid);
    EXPECT_NEAR(g_lastTarget.targetEfM.x, 10.0f, 0.1f);
    EXPECT_NEAR(g_lastTarget.targetEfM.z, 20.0f, 0.1f); // 120 m AMSL - 100 m origin

    // The injected plan advances past the PG waypoint count (1 positional wp).
    // The FLYOVER return leg keeps the arrival-radius gate, so the craft has to
    // reach the waypoint (10 m east) for it to count.
    g_stubEstimate.position.v[ENU_E] = 10.0f * 100.0f;
    arriveAtWaypoint();
    EXPECT_EQ(flightPlanNavGetCurrentIndex(), 1);
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);

    arriveAtWaypoint();
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_LANDING);
}

TEST_F(FlightPlanNavTest, InjectRejectsInvalidRequests)
{
    addWaypoint(10, 20, 15000, WAYPOINT_TYPE_FLYOVER);
    const waypoint_t wp = makeWaypoint(0, 0, 12000, WAYPOINT_TYPE_LAND);
    const waypoint_t five[5] = { wp, wp, wp, wp, wp };

    // Executor not active.
    EXPECT_FALSE(flightPlanNavInjectPlan(&wp, 1));

    flightPlanNavEngage();
    EXPECT_FALSE(flightPlanNavInjectPlan(&wp, 0));
    EXPECT_FALSE(flightPlanNavInjectPlan(five, 5));
    EXPECT_FALSE(flightPlanNavInjectPlan(NULL, 1));
    EXPECT_FALSE(flightPlanNavIsInjectedPlanActive());

    EXPECT_TRUE(flightPlanNavInjectPlan(&wp, 1));
    EXPECT_TRUE(flightPlanNavIsInjectedPlanActive());
}

TEST_F(FlightPlanNavTest, EngageAfterInjectRevertsToPgPlan)
{
    addWaypoint(10, 20, 15000, WAYPOINT_TYPE_FLYOVER);
    flightPlanNavEngage();
    const waypoint_t wp = makeWaypoint(0, 0, 12000, WAYPOINT_TYPE_LAND);
    ASSERT_TRUE(flightPlanNavInjectPlan(&wp, 1));

    flightPlanNavDisengage();
    EXPECT_FALSE(flightPlanNavIsInjectedPlanActive());

    flightPlanNavEngage();
    EXPECT_FALSE(flightPlanNavIsInjectedPlanActive());
    EXPECT_EQ(flightPlanNavGetCurrentIndex(), 0);
    ASSERT_TRUE(g_lastTarget.valid);
    EXPECT_NEAR(g_lastTarget.targetEfM.z, 50.0f, 0.1f); // PG waypoint: 150 m AMSL - 100 m origin
}

TEST_F(FlightPlanNavTest, ReachedListenerSuppressedWhileInjected)
{
    g_reachedCalls = 0;
    flightPlanNavSetReachedListener(recordReached);

    addWaypoint(10, 20, 15000, WAYPOINT_TYPE_FLYOVER);
    addWaypoint(30, 40, 15000, WAYPOINT_TYPE_FLYOVER);
    flightPlanNavEngage();
    arriveAtWaypoint();
    EXPECT_EQ(g_reachedCalls, 1); // PG mission progress is reported

    const waypoint_t plan[] = {
        makeWaypoint(0, 0, 12000, WAYPOINT_TYPE_FLYOVER),
        makeWaypoint(0, 0, 12000, WAYPOINT_TYPE_LAND),
    };
    ASSERT_TRUE(flightPlanNavInjectPlan(plan, 2));
    arriveAtWaypoint();
    EXPECT_EQ(g_reachedCalls, 1); // injected-plan progress is not
}

// --- LAND waypoint type ---

TEST_F(FlightPlanNavTest, LandWaypointDispatchesWithHoldGate)
{
    addWaypoint(10, 20, 15000, WAYPOINT_TYPE_LAND);
    flightPlanNavEngage();

    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
    ASSERT_TRUE(g_lastTarget.valid);
    // Station-keeping gate: waypointHoldRadius (2 m) with the altitude gate kept.
    EXPECT_NEAR(g_lastTarget.acceptanceRadiusM, 2.0f, 0.01f);
    EXPECT_TRUE(g_altitudeArrivalRequired);
}

TEST_F(FlightPlanNavTest, LandWaypointArrivalDescendsAtTheWaypoint)
{
    // Waypoint 10 m east, 20 m north (same flat-earth conversion as
    // EngageSetsFirstWaypointTarget).
    const int32_t lonUnitsFor10m = (int32_t)((10.0f / 111319.49f) * 1.0e7f);
    const int32_t latUnitsFor20m = (int32_t)((20.0f / 111319.49f) * 1.0e7f);
    addWaypoint(latUnitsFor20m, lonUnitsFor10m, 5000, WAYPOINT_TYPE_LAND);

    flightPlanNavEngage();
    ASSERT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);

    // Arrival gate trips short of the waypoint, 30 m up.
    g_stubEstimate.position.v[ENU_E] = 8.5f * 100.0f;
    g_stubEstimate.position.v[ENU_N] = 19.0f * 100.0f;
    g_stubEstimate.position.v[ENU_U] = 30.0f * 100.0f;
    triggerReached();

    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_LANDING);
    ASSERT_TRUE(g_lastTarget.valid);
    // Descent anchors at the waypoint, not the current position.
    EXPECT_NEAR(g_lastTarget.targetEfM.x, 10.0f, 0.1f);
    EXPECT_NEAR(g_lastTarget.targetEfM.y, 20.0f, 0.1f);
    EXPECT_NEAR(g_lastTarget.targetEfM.z, 30.0f - 200.0f, 0.1f);
    EXPECT_NEAR(g_lastTarget.cruiseSpeedMps, 0.5f, 0.01f);
}

TEST_F(FlightPlanNavTest, LandWaypointTouchdownDisarmsAndCompletes)
{
    addWaypoint(10, 20, 15000, WAYPOINT_TYPE_LAND);
    addWaypoint(30, 40, 15000, WAYPOINT_TYPE_FLYOVER); // must never be flown

    flightPlanNavEngage();
    triggerReached();
    ASSERT_EQ(flightPlanNavGetState(), FP_NAV_LANDING);
    const int dispatchesBeforeTouchdown = g_setTargetCalls;

    // Descent establishes (above 25% of the commanded rate).
    g_stubEstimate.velocity.v[ENU_U] = -40.0f; // cm/s
    g_stubMicros += 1'000'000;
    flightPlanNavUpdate(g_stubMicros);
    EXPECT_EQ(g_disarmCalls, 0);

    // Touchdown: quiet timer starts.
    g_stubEstimate.velocity.v[ENU_U] = 0.0f;
    g_stubMicros += 1'000'000;
    flightPlanNavUpdate(g_stubMicros);
    EXPECT_EQ(g_disarmCalls, 0);

    g_stubMicros += 1'100'000; // past landingDetectionTime (1 s)
    flightPlanNavUpdate(g_stubMicros);
    EXPECT_EQ(g_disarmCalls, 1);
    EXPECT_EQ(g_lastDisarmReason, DISARM_REASON_LANDING);
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_COMPLETE);
    // LAND is terminal: the trailing waypoint is never dispatched.
    EXPECT_EQ(g_setTargetCalls, dispatchesBeforeTouchdown);
}

TEST_F(FlightPlanNavTest, LandWaypointWithDurationLoitersThenDescends)
{
    const int32_t lonUnitsFor10m = (int32_t)((10.0f / 111319.49f) * 1.0e7f);
    addWaypoint(0, lonUnitsFor10m, 5000, WAYPOINT_TYPE_LAND, 0, 20 /* 2.0 s loiter */);

    g_stubMicros = 1'000'000;
    flightPlanNavEngage();
    triggerReached();

    // Pre-descent loiter, exactly like HOLD.
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_HOLDING);

    g_stubMicros += 1'000'000; // 1 s: still loitering
    flightPlanNavUpdate(g_stubMicros);
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_HOLDING);

    g_stubEstimate.position.v[ENU_U] = 30.0f * 100.0f;
    g_stubMicros += 1'500'000; // 2.5 s: loiter expired, descend at the waypoint
    flightPlanNavUpdate(g_stubMicros);
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_LANDING);
    ASSERT_TRUE(g_lastTarget.valid);
    EXPECT_NEAR(g_lastTarget.targetEfM.x, 10.0f, 0.1f);
    EXPECT_NEAR(g_lastTarget.targetEfM.z, 30.0f - 200.0f, 0.1f);
}

TEST_F(FlightPlanNavTest, LandLoiterExpiryWithLostTargetRedispatchesLeg)
{
    const int32_t lonUnitsFor10m = (int32_t)((10.0f / 111319.49f) * 1.0e7f);
    addWaypoint(0, lonUnitsFor10m, 5000, WAYPOINT_TYPE_LAND, 0, 20);

    g_stubMicros = 1'000'000;
    flightPlanNavEngage();
    triggerReached();
    ASSERT_EQ(flightPlanNavGetState(), FP_NAV_HOLDING);

    // The position command is wiped during the loiter (position-control
    // re-init). A zeroed target must not be used as the descent anchor —
    // the leg is re-flown instead.
    positionNavClearTarget();
    const int callsBefore = g_setTargetCalls;

    g_stubMicros += 2'500'000;
    flightPlanNavUpdate(g_stubMicros);
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
    EXPECT_EQ(g_setTargetCalls, callsBefore + 1);
    ASSERT_TRUE(g_lastTarget.valid);
    EXPECT_NEAR(g_lastTarget.targetEfM.x, 10.0f, 0.1f); // the LAND leg again, not a descent
    EXPECT_NEAR(g_lastTarget.targetEfM.z, -50.0f, 0.1f);
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

    // The ~2.2 km leg gives the capped 100 m flyaway margin; drift just past it.
    g_stubEstimate.position.v[ENU_N] = -105.0f * 100.0f; // cm
    flightPlanNavUpdate(g_stubMicros + 10'000);

    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_ABORTED);
    EXPECT_EQ(flightPlanNavGetAbortReason(), FP_ABORT_FLYAWAY);
}

TEST_F(FlightPlanNavSafetyTest, HeadingFaultParksWingsLevel)
{
    engageDistantLeg();

    // Nose on command (target due north, heading north) and moving at 5 m/s, but
    // course-over-ground is 90 deg off the heading: a magnetometer fault.
    attitude.values.yaw = 0;    // heading 0 deg (north), on the commanded bearing
    gpsSol.groundSpeed = 500;   // 5 m/s, above the 3 m/s gate
    gpsSol.groundCourse = 900;  // 90 deg: disagreement with heading exceeds 70 deg

    // The integrator needs > 2 s of sustained disagreement; per-cycle dt is
    // capped at 0.25 s, so step well past the threshold.
    for (int i = 0; i < 12; i++) {
        g_stubMicros += 250'000;
        flightPlanNavUpdate(g_stubMicros);
    }

    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_ABORTED);
    EXPECT_EQ(flightPlanNavGetAbortReason(), FP_ABORT_MAG_FAULT);
    EXPECT_TRUE(g_forceLevelPark);   // angle-mode self-level, never position hold
    EXPECT_GE(g_clearTargetCalls, 1);
}

TEST_F(FlightPlanNavSafetyTest, CourseDisagreementOffCommandDoesNotTrip)
{
    engageDistantLeg();

    // The same course-over-ground disagreement, but the nose is 60 deg off the
    // commanded bearing (a deliberate manoeuvre, not a fault): the detector is
    // ineligible and must not integrate toward a trip.
    attitude.values.yaw = 600;   // 60 deg, off the northward commanded bearing
    gpsSol.groundSpeed = 500;
    gpsSol.groundCourse = 1500;  // 150 deg: 90 deg from heading, still > 70

    for (int i = 0; i < 12; i++) {
        g_stubMicros += 250'000;
        flightPlanNavUpdate(g_stubMicros);
    }

    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
    EXPECT_FALSE(g_forceLevelPark);
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
    g_stubEstimate.position.v[ENU_E] = 12.0f * 100.0f; // cm
    g_stubEstimate.position.v[ENU_N] = 34.0f * 100.0f;
    g_stubEstimate.position.v[ENU_U] = 30.0f * 100.0f; // 30 m up
    flightPlanNavUpdate(g_stubMicros + 10'000);

    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_LANDING);
    ASSERT_TRUE(g_lastTarget.valid);
    // Landing target: current position, far below current altitude.
    EXPECT_NEAR(g_lastTarget.targetEfM.x, 12.0f, 0.1f);
    EXPECT_NEAR(g_lastTarget.targetEfM.y, 34.0f, 0.1f);
    EXPECT_NEAR(g_lastTarget.targetEfM.z, 30.0f - 200.0f, 0.1f);
    EXPECT_NEAR(g_lastTarget.cruiseSpeedMps, 0.5f, 0.01f);
    EXPECT_FALSE(flightPlanNavIsInjectedPlanActive());
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

TEST_F(FlightPlanNavSafetyTest, GeofenceBreachWithRthActionInjectsReturnPlan)
{
    autopilotConfigMutable()->maxDistanceFromHomeM = 100;
    autopilotConfigMutable()->geofenceAction = AP_GEOFENCE_RTH;
    stateFlags |= GPS_FIX_HOME;
    engageDistantLeg();
    const int callsBeforeBreach = g_setTargetCalls;

    GPS_distanceToHome = 150;
    g_stubEstimate.position.v[ENU_N] = 150.0f * 100.0f; // 150 m out, en route
    flightPlanNavUpdate(g_stubMicros + 10'000);

    // The return plan replaces the mission: fly to home at the rescue return
    // altitude (30 m above the 100 m home altitude) at the rescue ground speed.
    // The return leg is a pass-through leg, so positionNav is fed a carrot
    // marching toward home — assert the dispatched destination, not the carrot.
    EXPECT_TRUE(flightPlanNavIsInjectedPlanActive());
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
    EXPECT_EQ(g_setTargetCalls, callsBeforeBreach + 1);
    EXPECT_NEAR(g_lastDispatchTargetEfM.x, 0.0f, 0.1f);
    EXPECT_NEAR(g_lastDispatchTargetEfM.y, 0.0f, 0.1f);
    EXPECT_NEAR(g_lastDispatchTargetEfM.z, 30.0f, 0.1f);
    EXPECT_NEAR(g_lastTarget.cruiseSpeedMps, 7.5f, 0.01f);

    // Still outside the fence on the way home: no re-injection.
    g_stubMicros += 1'000'000;
    flightPlanNavUpdate(g_stubMicros);
    EXPECT_EQ(g_setTargetCalls, callsBeforeBreach + 1);

    // Reach home: the pass-through leg advances through the carrot gate and
    // dispatches the plan's LAND leg to home; its arrival descends there.
    GPS_distanceToHome = 0;
    g_stubEstimate.position.v[ENU_N] = 0.0f;
    g_stubMicros += 100'000;
    flightPlanNavUpdate(g_stubMicros);
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
    EXPECT_NEAR(g_lastDispatchTargetEfM.x, 0.0f, 0.1f);
    EXPECT_NEAR(g_lastDispatchTargetEfM.y, 0.0f, 0.1f);

    triggerReached();  // the LAND leg is a precise point target: its callback advances it
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_LANDING);

    // No resume: the injected plan dies with disengagement.
    flightPlanNavDisengage();
    EXPECT_FALSE(flightPlanNavIsInjectedPlanActive());
}

TEST_F(FlightPlanNavSafetyTest, GeofenceRthAboveReturnAltReturnsAtCurrentAltitude)
{
    autopilotConfigMutable()->maxDistanceFromHomeM = 100;
    autopilotConfigMutable()->geofenceAction = AP_GEOFENCE_RTH;
    stateFlags |= GPS_FIX_HOME;
    gpsSol.llh.altCm = 15000; // 150 m AMSL: 20 m above home + returnAltitudeM
    engageDistantLeg();

    GPS_distanceToHome = 150;
    flightPlanNavUpdate(g_stubMicros + 10'000);

    ASSERT_TRUE(flightPlanNavIsInjectedPlanActive());
    // Never descend en route: the plan returns at the current GPS altitude,
    // which in the estimator's feedback frame is its reading at engage (the
    // stub reads 0 while GPS says 150 m AMSL — a mid-flight engagement).
    EXPECT_NEAR(g_lastTarget.targetEfM.z, 0.0f, 0.1f);
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

// --- Leg-line carrot tracking and turn-angle cornering ---

class FlightPlanNavCarrotTest : public FlightPlanNavTest {
protected:
    static constexpr float kUnitsPerMetre = 1.0e7f / 111319.49f;

    void addWaypointMetres(float eastM, float northM, int32_t altCm, uint8_t type) {
        addWaypoint((int32_t)lrintf(northM * kUnitsPerMetre),
                    (int32_t)lrintf(eastM * kUnitsPerMetre), altCm, type);
    }
    void setCraftMetres(float eastM, float northM) {
        g_stubEstimate.position.v[ENU_E] = eastM * 100.0f;
        g_stubEstimate.position.v[ENU_N] = northM * 100.0f;
    }
    void step(uint32_t us = 100'000) {
        g_stubMicros += us;
        flightPlanNavUpdate(g_stubMicros);
    }
};

TEST_F(FlightPlanNavCarrotTest, CarrotTracksLegLineNotCraftCrossTrack)
{
    addWaypointMetres(0.0f, 100.0f, 15000, WAYPOINT_TYPE_FLYOVER); // wp0 (pass-through)
    addWaypointMetres(0.0f, 200.0f, 15000, WAYPOINT_TYPE_FLYOVER); // wp1 (last)
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();   // craft at the origin: leg0 is origin -> (0,100), due north
    step();                  // anchor the leg and march

    // Wind pushes the craft 20 m east of the leg line, halfway along. The
    // along-track measurement is PT1-filtered (0.2 s), so give it a few cycles
    // to converge on the displaced position.
    setCraftMetres(20.0f, 50.0f);
    for (int i = 0; i < 8; i++) {
        step();
    }

    // The carrot rides the leg line (E = 0), not the craft (E = 20), so the
    // position controller is commanded to pull back onto the drawn line. It sits
    // near the craft's along-track progress (N ~ 50), not stuck at the origin or
    // the far waypoint.
    ASSERT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
    ASSERT_EQ(flightPlanNavGetCurrentIndex(), 0);
    EXPECT_NEAR(g_lastTarget.targetEfM.x, 0.0f, 0.5f);
    EXPECT_NEAR(g_lastTarget.targetEfM.y, 50.0f, 5.0f);
}

TEST_F(FlightPlanNavCarrotTest, StraightThroughFlybyAdvancesAtWideGate)
{
    addWaypointMetres(0.0f, 100.0f, 15000, WAYPOINT_TYPE_FLYBY); // wp0
    addWaypointMetres(0.0f, 200.0f, 15000, WAYPOINT_TYPE_FLYBY); // wp1: straight on (last)
    setCraftMetres(0.0f, 89.0f);   // 11 m short of wp0
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();
    step();

    // Straight-through FLYBY: corner speed = cruise, gate radius clamps to 12 m,
    // so an 11 m approach already crosses the gate and advances.
    EXPECT_EQ(flightPlanNavGetCurrentIndex(), 1);
}

TEST_F(FlightPlanNavCarrotTest, SharpTurnFlybyAdvancesOnlyCloseIn)
{
    addWaypointMetres(0.0f, 100.0f, 15000, WAYPOINT_TYPE_FLYBY); // wp0
    addWaypointMetres(0.0f, 50.0f, 15000, WAYPOINT_TYPE_FLYBY);  // wp1: 180 deg reversal (last)
    setCraftMetres(0.0f, 89.0f);   // 11 m short of wp0
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();
    step();

    // Hairpin FLYBY: corner speed drops to the floor, gate radius shrinks to 5 m,
    // so an 11 m approach is not yet a crossing — the leg keeps tracking.
    EXPECT_EQ(flightPlanNavGetCurrentIndex(), 0);
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
}

TEST_F(FlightPlanNavCarrotTest, FlyoverUsesArrivalRadiusGate)
{
    addWaypointMetres(0.0f, 100.0f, 15000, WAYPOINT_TYPE_FLYOVER); // fly over the point
    addWaypointMetres(0.0f, 200.0f, 15000, WAYPOINT_TYPE_FLYOVER); // straight on (last)
    setCraftMetres(0.0f, 92.0f);   // 8 m short: inside a FLYBY 12 m gate, outside FLYOVER's 5 m
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();
    step();

    // FLYOVER keeps the arrival-radius gate, so an 8 m straight-through approach
    // has not crossed yet (a FLYBY would have).
    EXPECT_EQ(flightPlanNavGetCurrentIndex(), 0);
}

TEST_F(FlightPlanNavCarrotTest, EngageNoseBackwardsRotatesOntoLeg)
{
    addWaypointMetres(0.0f, 100.0f, 15000, WAYPOINT_TYPE_FLYBY); // leg due north
    addWaypointMetres(0.0f, 200.0f, 15000, WAYPOINT_TYPE_FLYBY); // last
    setCraftMetres(0.0f, 0.0f);
    attitude.values.yaw = 1800;   // nose pointing south, 180 deg off the north leg
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();
    step();

    // Nothing else steers yaw in VELOCITY mode with no course developed, so the
    // executor must command the nose onto the leg (north) rather than deadlocking
    // on a frozen carrot.
    ASSERT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
    EXPECT_TRUE(g_navHeadingOverrideValid);
    EXPECT_NEAR(g_navHeadingOverrideDeg, 0.0f, 1.0f);   // leg bearing, north
}

TEST_F(FlightPlanNavCarrotTest, BrakingCarrotBehindKeepsNoseForward)
{
    addWaypointMetres(0.0f, 100.0f, 15000, WAYPOINT_TYPE_FLYBY);
    addWaypointMetres(0.0f, 200.0f, 15000, WAYPOINT_TYPE_FLYBY);
    setCraftMetres(0.0f, 0.0f);
    attitude.values.yaw = 0;   // nose north, along the leg
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();
    step();
    setCraftMetres(0.0f, 60.0f);   // overrun: shoot well past the trailing carrot
    step();

    // The carrot is now behind the craft (braking); the nose stays pointed forward
    // along the leg (north), never swung 180 back at the trailing carrot.
    ASSERT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
    EXPECT_TRUE(g_navHeadingOverrideValid);
    EXPECT_NEAR(g_navHeadingOverrideDeg, 0.0f, 5.0f);
}

TEST_F(FlightPlanNavCarrotTest, OverrunFallbackAdvancesPastWaypoint)
{
    addWaypointMetres(0.0f, 100.0f, 15000, WAYPOINT_TYPE_FLYOVER); // wp0
    addWaypointMetres(0.0f, 50.0f, 15000, WAYPOINT_TYPE_FLYOVER);  // wp1: hairpin, tight 5 m gate
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();   // craft at origin: anchors leg0 origin -> (0,100)
    step();

    // The craft shoots past wp0 along-track (to N = 108) inside the lateral
    // corridor without ever entering the 5 m hairpin bubble: the overrun
    // fallback still counts the gate.
    setCraftMetres(0.5f, 108.0f);
    step();

    EXPECT_EQ(flightPlanNavGetCurrentIndex(), 1);
}

TEST_F(FlightPlanNavCarrotTest, PreTurnBlendsNoseTowardNextLeg)
{
    addWaypointMetres(0.0f, 100.0f, 15000, WAYPOINT_TYPE_FLYOVER);   // wp0: turn here
    addWaypointMetres(100.0f, 100.0f, 15000, WAYPOINT_TYPE_FLYOVER); // wp1: 90 deg east (last)
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();
    step();   // anchor leg0 (origin -> (0,100), heading north)

    // Approach the gate into the pre-turn zone (7 m of leg left past the gate).
    setCraftMetres(0.0f, 88.0f);
    step();

    ASSERT_EQ(flightPlanNavGetCurrentIndex(), 0);
    EXPECT_TRUE(g_navHeadingOverrideValid);
    // Nose commanded between this leg (0 deg, north) and the next (90 deg, east).
    EXPECT_GT(g_navHeadingOverrideDeg, 0.0f);
    EXPECT_LT(g_navHeadingOverrideDeg, 90.0f);
}

TEST_F(FlightPlanNavCarrotTest, CornerSkipsModifierBetweenLegs)
{
    addWaypointMetres(0.0f, 100.0f, 15000, WAYPOINT_TYPE_FLYOVER);    // wp0: turn here
    addWaypointMetres(-100.0f, 100.0f, 0, WAYPOINT_TYPE_YAW_RATE);    // modifier: bogus west coords, must be ignored
    addWaypointMetres(100.0f, 100.0f, 15000, WAYPOINT_TYPE_FLYOVER);  // next positional leg: 90 deg east
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();
    step();
    setCraftMetres(0.0f, 88.0f);   // into the pre-turn zone before wp0
    step();

    ASSERT_EQ(flightPlanNavGetCurrentIndex(), 0);
    ASSERT_TRUE(g_navHeadingOverrideValid);
    // The corner blends toward the next positional leg (east, +90), not the
    // modifier's west coordinates, which would drive the override negative.
    EXPECT_GT(g_navHeadingOverrideDeg, 0.0f);
    EXPECT_LT(g_navHeadingOverrideDeg, 90.0f);
}

TEST_F(FlightPlanNavCarrotTest, OverspeedGovernorHoldsCarrotWithHysteresis)
{
    addWaypointMetres(0.0f, 300.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    addWaypointMetres(0.0f, 600.0f, 15000, WAYPOINT_TYPE_FLYOVER); // straight on (last)
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();
    step();   // anchor the leg; craft parked at the origin

    // On profile: the carrot marches away from the parked craft. Keep this
    // phase short so the hold below happens well inside the minimum 6 m lead
    // cap — a carrot parked ON the cap would mask a broken governor.
    for (int i = 0; i < 3; i++) {
        step();
    }
    const float marchingN = g_lastTarget.targetEfM.y;
    EXPECT_GT(marchingN, 0.05f);

    // Craft carries 15 m/s against the 10 m/s cruise profile (tailwind /
    // catch-up): the governor holds the carrot so braking authority returns.
    g_stubEstimate.velocity.v[ENU_N] = 1500.0f;
    for (int i = 0; i < 20; i++) {
        step();   // speed filter converges, carrot speed slews to a stop
    }
    const float heldN = g_lastTarget.targetEfM.y;
    EXPECT_LT(heldN, 5.5f);   // held by the governor, not parked on the lead cap
    for (int i = 0; i < 10; i++) {
        step();
    }
    EXPECT_NEAR(g_lastTarget.targetEfM.y, heldN, 0.01f);

    // Speed drops into the hysteresis band (10.9 m/s: below the 11.5 entry,
    // above the 10.75 release). A single-threshold governor resumes marching
    // here and chatters cruise/freeze at fix rate; the hold must stick.
    g_stubEstimate.velocity.v[ENU_N] = 1090.0f;
    for (int i = 0; i < 20; i++) {
        step();
    }
    EXPECT_NEAR(g_lastTarget.targetEfM.y, heldN, 0.01f);

    // Fully back on profile (craft moving gently up the leg so the lead window
    // opens ahead): the carrot marches again.
    g_stubEstimate.velocity.v[ENU_N] = 200.0f;
    for (int i = 0; i < 20; i++) {
        setCraftMetres(0.0f, (i + 1) * 0.2f);
        step();
    }
    EXPECT_GT(g_lastTarget.targetEfM.y, heldN + 0.5f);
}

TEST_F(FlightPlanNavCarrotTest, ChaseLagCompensationCrossesGateNearCornerSpeed)
{
    addWaypointMetres(0.0f, 300.0f, 15000, WAYPOINT_TYPE_FLYBY);   // 90 deg corner here
    addWaypointMetres(300.0f, 300.0f, 15000, WAYPOINT_TYPE_FLYBY); // east leg (last)
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();
    step();

    // First-order pursuit: the craft chases the commanded carrot with the same
    // time constant as the configured carrot lead (1.2 s) — the chase lag the
    // brake compensation exists for. Without the compensation the carrot itself
    // reaches corner speed at the gate but the craft, answering ~1.2 s late,
    // crosses hot by roughly lag * decel.
    const float dt = 0.1f;
    const float tauS = 1.2f;
    float craftE = 0.0f;
    float craftN = 0.0f;
    float crossingSpeedMps = -1.0f;
    float maxSpeedMps = 0.0f;
    for (int i = 0; i < 1500 && crossingSpeedMps < 0.0f; i++) {
        const float velE = (g_lastTarget.targetEfM.x - craftE) / tauS;
        const float velN = (g_lastTarget.targetEfM.y - craftN) / tauS;
        craftE += velE * dt;
        craftN += velN * dt;
        setCraftMetres(craftE, craftN);
        g_stubEstimate.velocity.v[ENU_E] = velE * 100.0f;
        g_stubEstimate.velocity.v[ENU_N] = velN * 100.0f;
        const float speedMps = sqrtf(velE * velE + velN * velN);
        maxSpeedMps = fmaxf(maxSpeedMps, speedMps);
        step();
        if (flightPlanNavGetCurrentIndex() == 1) {
            crossingSpeedMps = speedMps;
        }
    }

    // The leg actually cruised, and the CRAFT (not just the carrot) crossed the
    // corner gate near the 90-degree corner speed (4.4 m/s delta-v budget ->
    // 3.1 m/s), instead of several m/s hot.
    EXPECT_GT(maxSpeedMps, 8.0f);
    ASSERT_GE(crossingSpeedMps, 0.0f);
    EXPECT_LT(crossingSpeedMps, 4.3f);
}
