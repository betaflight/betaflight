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

// Flag-on companion to flight_plan_nav_unittest.cc: ENABLE_RESCUE_PLAN=1,
// exercising the failsafe rescue mission synthesised by
// flightPlanNavStageRescuePlan(). flight_plan_nav_unittest.cc stays the
// flag-off regression guard and is not touched by this binary.

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
    #include "flight/gps_rescue.h"
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
// Scaffolding mirrors flight_plan_nav_unittest.cc; extended with the
// rescue-controller/IMU seams ENABLE_RESCUE_PLAN adds (the rescue
// CONTROLLER flight/gps_rescue_multirotor.c is not linked here, only its
// config pg/gps_rescue_multirotor.c, so gpsRescueGetMaxAltitudeCm() is
// stubbed rather than driven by real rescue state).

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

bool g_altitudeArrivalRequired;
float g_yawRateLimitDps;

float g_stubMaxAltitudeCm;
bool g_stubHeadingValid;
int g_pitchForwardCalls;
bool g_lastPitchForward;

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

void positionNavMoveTargetEf(const vector3_t *targetPosEfM)
{
    if (!g_lastTarget.valid) {
        return;
    }
    g_lastTarget.targetEfM = *targetPosEfM;
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

void autopilotSetYawRateLimit(float rateLimitDps)
{
    g_yawRateLimitDps = rateLimitDps;
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

// --- ENABLE_RESCUE_PLAN-specific seams ---
// The rescue controller (flight/gps_rescue_multirotor.c) is not linked; only
// its config (pg/gps_rescue_multirotor.c) is. gpsRescueGetMaxAltitudeCm()
// normally reads the controller's running max-altitude-since-arming state.
float gpsRescueGetMaxAltitudeCm(void)
{
    return g_stubMaxAltitudeCm;
}

bool imuIsHeadingValid(void)
{
    return g_stubHeadingValid;
}

void pitchForwardOverride(bool request)
{
    g_pitchForwardCalls++;
    g_lastPitchForward = request;
}

void autopilotForceLevelPark(bool) {}
void autopilotSetNavHeadingOverride(bool, float) {}

} // extern "C"

class FlightPlanRescueTest : public ::testing::Test {
protected:
    void SetUp() override {
        memset(&g_lastTarget, 0, sizeof(g_lastTarget));
        g_setTargetCalls = 0;
        g_clearTargetCalls = 0;
        g_stubMicros = 0;

        memset(&g_stubEstimate, 0, sizeof(g_stubEstimate));
        g_stubValidXY = true;
        g_stubBelowLandingAltitude = true;
        g_altitudeArrivalRequired = false;
        g_disarmCalls = 0;
        g_yawRateLimitDps = -1.0f;

        g_stubMaxAltitudeCm = 0.0f;
        g_stubHeadingValid = true; // heading trusted unless a test says otherwise
        g_pitchForwardCalls = 0;
        g_lastPitchForward = false;

        // Home and GPS origin at the equator/prime meridian, 100 m AMSL;
        // current position 30 m east of home so the rescue climb waypoint
        // (current position) and the return leg (home) are distinguishable.
        stateFlags = GPS_FIX_HOME | GPS_FIX;
        GPS_distanceToHome = 100; // clear of the close-range branch (minStartDistM = 15)

        g_stubGpsOrigin.lat = 0;
        g_stubGpsOrigin.lon = 0;
        g_stubGpsOrigin.altCm = 10000;
        g_stubGpsOriginSet = true;

        memset(&GPS_home_llh, 0, sizeof(GPS_home_llh));
        GPS_home_llh.altCm = 10000;

        memset(&gpsSol, 0, sizeof(gpsSol));
        gpsSol.llh.lon = metresToLonUnits(30.0f);
        gpsSol.llh.altCm = 10000;

        gpsRescueConfig_t *rescueCfg = gpsRescueConfigMutable();
        memset(rescueCfg, 0, sizeof(*rescueCfg));
        rescueCfg->returnAltitudeM = 30;
        rescueCfg->groundSpeedCmS = 750;
        rescueCfg->initialClimbM = 10;
        rescueCfg->minStartDistM = 15;
        rescueCfg->altitudeMode = GPS_RESCUE_ALT_MODE_MAX;

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

        flightPlanNavInit();
    }

    void TearDown() override {
        flightPlanNavSetReachedListener(nullptr);
    }

    static int32_t metresToLonUnits(float metres)
    {
        return (int32_t)((metres / 111319.49f) * 1.0e7f);
    }

    void addWaypoint(int32_t lat, int32_t lon, int32_t altCm,
                     uint8_t type, uint16_t speed = 0, uint16_t duration = 0)
    {
        flightPlanConfig_t *plan = flightPlanConfigMutable();
        waypoint_t *wp = &plan->waypoints[plan->waypointCount++];
        wp->latitude = lat;
        wp->longitude = lon;
        wp->altitude = altCm;
        wp->type = type;
        wp->speed = speed;
        wp->duration = duration;
        wp->pattern = WAYPOINT_PATTERN_NONE;
    }

    void triggerReached() {
        ASSERT_NE(g_lastTarget.callback, nullptr);
        g_lastTarget.callback(g_lastTarget.userData);
    }

    // Default-config expected rescue return altitude (MAX mode, 0 cm stubbed
    // max, 10 m climb, home == current == 100 m AMSL): home + 10 m climb.
    static constexpr float kDefaultReturnAltM = 10.0f;
};

// --- Staging and dispatch ---

TEST_F(FlightPlanRescueTest, StageThenEngageDispatchesRescuePlan)
{
    // A PG waypoint at a wildly different place/altitude: must never be flown.
    addWaypoint(500000, 500000, 99999, WAYPOINT_TYPE_FLYOVER);

    ASSERT_TRUE(flightPlanNavStageRescuePlan());
    flightPlanNavEngage();

    EXPECT_TRUE(flightPlanNavIsInjectedPlanActive());
    EXPECT_TRUE(flightPlanNavIsRescuePlanActive());
    EXPECT_EQ(flightPlanNavGetCurrentIndex(), 0);
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);

    ASSERT_TRUE(g_lastTarget.valid);
    // Rescue wp0: HOLD at the current GPS position (30 m east of home/origin).
    EXPECT_NEAR(g_lastTarget.targetEfM.x, 30.0f, 0.1f);
    EXPECT_NEAR(g_lastTarget.targetEfM.y, 0.0f, 0.1f);
    EXPECT_NEAR(g_lastTarget.targetEfM.z, kDefaultReturnAltM, 0.1f);
}

TEST_F(FlightPlanRescueTest, StageFailsWithoutHomeFix)
{
    stateFlags &= ~GPS_FIX_HOME;
    EXPECT_FALSE(flightPlanNavStageRescuePlan());
}

TEST_F(FlightPlanRescueTest, StageFailsWithoutGpsFix)
{
    stateFlags &= ~GPS_FIX;
    EXPECT_FALSE(flightPlanNavStageRescuePlan());
}

// --- Return-altitude computation ---

TEST_F(FlightPlanRescueTest, ReturnAltPerMode)
{
    // FIXED: home (100 m) + returnAltitudeM (30 m) = 130 m.
    gpsRescueConfigMutable()->altitudeMode = GPS_RESCUE_ALT_MODE_FIXED;
    ASSERT_TRUE(flightPlanNavStageRescuePlan());
    flightPlanNavEngage();
    EXPECT_NEAR(g_lastTarget.targetEfM.z, 30.0f, 0.01f);
    flightPlanNavDisengage();

    // CURRENT: current + initialClimbM. In the estimator's feedback frame the
    // target is 10 m above the current reading (the stub reads 0 while GPS
    // says 110 m AMSL — zBias reconciles the frames at engage).
    gpsRescueConfigMutable()->altitudeMode = GPS_RESCUE_ALT_MODE_CURRENT;
    gpsSol.llh.altCm = 11000;
    ASSERT_TRUE(flightPlanNavStageRescuePlan());
    flightPlanNavEngage();
    EXPECT_NEAR(g_lastTarget.targetEfM.z, 10.0f, 0.01f);
    flightPlanNavDisengage();

    // MAX: home (100 m) + stubbed max (45 m) + initialClimbM (10 m) = 155 m.
    gpsRescueConfigMutable()->altitudeMode = GPS_RESCUE_ALT_MODE_MAX;
    gpsSol.llh.altCm = 10000;
    g_stubMaxAltitudeCm = 4500;
    ASSERT_TRUE(flightPlanNavStageRescuePlan());
    flightPlanNavEngage();
    EXPECT_NEAR(g_lastTarget.targetEfM.z, 55.0f, 0.01f);
    flightPlanNavDisengage();

    // Floored at current altitude: FIXED computes 130 m but current is 140 m,
    // so the plan returns at the current altitude — which in the estimator
    // frame is its own reading at engage (0 here).
    gpsRescueConfigMutable()->altitudeMode = GPS_RESCUE_ALT_MODE_FIXED;
    gpsSol.llh.altCm = 14000;
    ASSERT_TRUE(flightPlanNavStageRescuePlan());
    flightPlanNavEngage();
    EXPECT_NEAR(g_lastTarget.targetEfM.z, 0.0f, 0.01f);
}

TEST_F(FlightPlanRescueTest, CloseRangeReturnAltitudeUsesModestHeadroom)
{
    // Close range (< minStartDistM): MAX(home + 7.5 m, current + climb).
    // A 2 m climb keeps home+750 (7.5 m) the larger term: 107.5 m -> +7.5 m.
    GPS_distanceToHome = 10;
    gpsRescueConfigMutable()->initialClimbM = 2;

    ASSERT_TRUE(flightPlanNavStageRescuePlan());
    flightPlanNavEngage();

    EXPECT_NEAR(g_lastTarget.targetEfM.z, 7.5f, 0.01f);
}

// --- Staging while active / already engaged ---

TEST_F(FlightPlanRescueTest, StageWhileActiveInjectsImmediately)
{
    addWaypoint(200000, 0, 15000, WAYPOINT_TYPE_FLYOVER); // ~2.2 km PG mission
    flightPlanNavEngage();
    ASSERT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
    ASSERT_FALSE(flightPlanNavIsInjectedPlanActive());
    const int callsBefore = g_setTargetCalls;

    ASSERT_TRUE(flightPlanNavStageRescuePlan());

    EXPECT_TRUE(flightPlanNavIsInjectedPlanActive());
    EXPECT_TRUE(flightPlanNavIsRescuePlanActive());
    EXPECT_EQ(flightPlanNavGetCurrentIndex(), 0);
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
    EXPECT_EQ(g_setTargetCalls, callsBefore + 1);
    ASSERT_TRUE(g_lastTarget.valid);
    EXPECT_NEAR(g_lastTarget.targetEfM.x, 30.0f, 0.1f); // rescue wp0, not the PG leg
}

// --- Climb-leg altitude gate ---

TEST_F(FlightPlanRescueTest, ClimbLegAltitudeGated)
{
    ASSERT_TRUE(flightPlanNavStageRescuePlan());
    flightPlanNavEngage();

    ASSERT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
    EXPECT_TRUE(g_altitudeArrivalRequired);
    ASSERT_TRUE(g_lastTarget.valid);
    // The climb leg targets the current position, not home.
    EXPECT_NEAR(g_lastTarget.targetEfM.x, 30.0f, 0.1f);
    EXPECT_NEAR(g_lastTarget.targetEfM.y, 0.0f, 0.1f);
}

// --- Heading gate ---

TEST_F(FlightPlanRescueTest, HeadingGateHoldsAndRecovers)
{
    ASSERT_TRUE(flightPlanNavStageRescuePlan());
    flightPlanNavEngage();
    ASSERT_EQ(flightPlanNavGetCurrentIndex(), 0);
    const int callsAtWp0 = g_setTargetCalls;

    g_stubHeadingValid = false;
    triggerReached(); // wp0 (climb) reached with heading untrusted

    EXPECT_EQ(g_pitchForwardCalls, 1);
    EXPECT_TRUE(g_lastPitchForward);
    EXPECT_EQ(flightPlanNavGetCurrentIndex(), 0); // no advance
    EXPECT_EQ(g_setTargetCalls, callsAtWp0);      // target unchanged

    g_stubHeadingValid = true;
    flightPlanNavUpdate(g_stubMicros);

    EXPECT_EQ(g_lastPitchForward, false);
    EXPECT_EQ(flightPlanNavGetCurrentIndex(), 1);
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
    ASSERT_TRUE(g_lastTarget.valid);
    // wp1: FLYOVER home.
    EXPECT_NEAR(g_lastTarget.targetEfM.x, 0.0f, 0.1f);
    EXPECT_NEAR(g_lastTarget.targetEfM.y, 0.0f, 0.1f);
}

TEST_F(FlightPlanRescueTest, HeadingGateTimesOutToAbort)
{
    g_stubMicros = 1'000'000;
    ASSERT_TRUE(flightPlanNavStageRescuePlan());
    flightPlanNavEngage();

    g_stubHeadingValid = false;
    triggerReached();
    ASSERT_EQ(flightPlanNavGetCurrentIndex(), 0);

    g_stubMicros += 15'000'000; // FP_RESCUE_HEADING_TIMEOUT_US
    flightPlanNavUpdate(g_stubMicros);

    EXPECT_EQ(g_lastPitchForward, false);
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_ABORTED);
    EXPECT_EQ(flightPlanNavGetAbortReason(), FP_ABORT_HEADING);
}

TEST_F(FlightPlanRescueTest, DisengageDuringHeadingHoldReleasesPitch)
{
    addWaypoint(200000, 0, 15000, WAYPOINT_TYPE_FLYOVER); // PG mission for the re-engage check

    ASSERT_TRUE(flightPlanNavStageRescuePlan());
    flightPlanNavEngage();

    g_stubHeadingValid = false;
    triggerReached();
    ASSERT_TRUE(g_lastPitchForward);

    flightPlanNavDisengage();
    EXPECT_EQ(g_lastPitchForward, false);
    EXPECT_FALSE(flightPlanNavIsActive());

    // No staged rescue survives disengage: re-engage flies the PG mission.
    flightPlanNavEngage();
    EXPECT_FALSE(flightPlanNavIsRescuePlanActive());
    EXPECT_FALSE(flightPlanNavIsInjectedPlanActive());
    EXPECT_EQ(flightPlanNavGetCurrentIndex(), 0);
}

// --- Full mission run ---

TEST_F(FlightPlanRescueTest, FullRescueRunToLanding)
{
    ASSERT_TRUE(flightPlanNavStageRescuePlan());
    flightPlanNavEngage();
    ASSERT_EQ(flightPlanNavGetCurrentIndex(), 0);

    // Heading trusted throughout: wp0 (climb) -> wp1 (home) -> wp2 (land).
    triggerReached();
    ASSERT_EQ(flightPlanNavGetCurrentIndex(), 1);
    ASSERT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);

    // wp1 (fly home) is an en-route pass-through leg: it advances through the
    // executor's carrot gate on a position update, not the positionNav callback.
    // The estimator places the craft at home (ENU origin), already inside the gate.
    g_stubMicros += 100'000;
    flightPlanNavUpdate(g_stubMicros);
    ASSERT_EQ(flightPlanNavGetCurrentIndex(), 2);
    ASSERT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);

    triggerReached(); // LAND waypoint: starts the descent
    ASSERT_EQ(flightPlanNavGetState(), FP_NAV_LANDING);

    // Descent establishes.
    g_stubEstimate.velocity.v[ENU_U] = -40.0f; // cm/s
    g_stubMicros += 1'000'000;
    flightPlanNavUpdate(g_stubMicros);
    EXPECT_EQ(g_disarmCalls, 0);

    // Touchdown: quiet timer starts, then expires.
    g_stubEstimate.velocity.v[ENU_U] = 0.0f;
    g_stubMicros += 1'000'000;
    flightPlanNavUpdate(g_stubMicros);
    EXPECT_EQ(g_disarmCalls, 0);

    g_stubMicros += 1'100'000; // past landingDetectionTime (1 s)
    flightPlanNavUpdate(g_stubMicros);

    EXPECT_EQ(g_disarmCalls, 1);
    EXPECT_EQ(g_lastDisarmReason, DISARM_REASON_LANDING);
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_COMPLETE);
}

// --- No staging ---

TEST_F(FlightPlanRescueTest, EngageWithoutStagedRunsPgMission)
{
    addWaypoint(200000, 0, 15000, WAYPOINT_TYPE_FLYOVER); // ~2.2 km PG mission

    flightPlanNavEngage();

    EXPECT_FALSE(flightPlanNavIsRescuePlanActive());
    EXPECT_FALSE(flightPlanNavIsInjectedPlanActive());
    EXPECT_EQ(flightPlanNavGetCurrentIndex(), 0);
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
    ASSERT_TRUE(g_lastTarget.valid);
    // The PG waypoint's altitude (50 m AMSL) is nowhere near the rescue climb.
    EXPECT_NEAR(g_lastTarget.targetEfM.z, 50.0f, 0.1f);
}
