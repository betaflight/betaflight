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

float g_altHoldClimbRateCmS;
float g_lastVertRateMps;
float g_lastVertStartAltM;
vector2_t g_stubPositionErrorCm;
int g_setVerticalProfileCalls;
float g_stubCommandedAltCm;
bool g_stubCommandedAltSet;
vector3_t g_stubTargetVelCmS;

gpsLocation_t g_stubGpsOrigin;
bool g_stubGpsOriginSet;

timeUs_t g_stubMicros;
timeUs_t g_targetWalkFromUs;   // positionNav walks a target flown at a stated velocity from when it was last placed

positionEstimate3d_t g_stubEstimate;
bool g_stubValidXY;
bool g_stubBelowLandingAltitude;

int g_disarmCalls;
flightLogDisarmReason_e g_lastDisarmReason;

vector2_t g_lastFfEfMps;
bool g_ffValid;
float g_lastAccelLimitMps2;
float g_lastDecelLimitMps2;

// The speed of the velocity the carrot last commanded.
float lastFfSpeedMps(void)
{
    return sqrtf(g_lastFfEfMps.x * g_lastFfEfMps.x + g_lastFfEfMps.y * g_lastFfEfMps.y);
}

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
    g_ffValid = false;
    memset(&g_lastFfEfMps, 0, sizeof(g_lastFfEfMps));
    g_setTargetCalls++;
    g_targetWalkFromUs = g_stubMicros;
}

float altHoldGetClimbRateCmS(void)
{
    return g_altHoldClimbRateCmS;
}

void positionNavSetVerticalProfile(float rateMps, float startAltM)
{
    g_lastVertRateMps = rateMps;
    g_lastVertStartAltM = startAltM;
    g_setVerticalProfileCalls++;
}

// The altitude the active command is walking its ramp through: the leg altitude, as if the ramp
// had already got there, unless a test states otherwise.
float positionNavGetTargetAltitudeCm(void)
{
    return g_stubCommandedAltSet ? g_stubCommandedAltCm : g_lastTarget.targetEfM.z * 100.0f;
}

vector3_t positionNavGetTargetVelocityCmS(void)
{
    return g_stubTargetVelCmS;
}

// Where positionNav has walked the target to by now.
static vector3_t walkedTargetEfM(void)
{
    vector3_t targetEfM = g_lastTarget.targetEfM;
    if (g_ffValid) {
        const float walkS = (g_stubMicros - g_targetWalkFromUs) * 1e-6f;
        targetEfM.x += g_lastFfEfMps.x * walkS;
        targetEfM.y += g_lastFfEfMps.y * walkS;
    }
    return targetEfM;
}

void positionNavMoveTargetEf(const vector3_t *targetPosEfM)
{
    if (!g_lastTarget.valid) {
        return;
    }
    g_lastTarget.targetEfM = *targetPosEfM;
    g_targetWalkFromUs = g_stubMicros;
    g_moveTargetCalls++;
}

void positionNavClearTarget(void)
{
    g_clearTargetCalls++;
    g_lastTarget.valid = false;
}

static int g_startAfreshCalls;

void positionNavStartAfresh(void)
{
    g_startAfreshCalls++;
}

void positionNavSetAutoClearOnReach(bool autoClear)
{
    (void)autoClear;
}

void positionNavSetAccelLimits(float maxAccelMps2, float maxDecelMps2)
{
    g_lastAccelLimitMps2 = maxAccelMps2;
    g_lastDecelLimitMps2 = maxDecelMps2;
}

float g_lastApproachSlowdownM;
float g_lastApproachStillRadiusM;

void positionNavSetApproachSlowdown(float slowdownM, float stillRadiusM)
{
    g_lastApproachSlowdownM = slowdownM;
    g_lastApproachStillRadiusM = stillRadiusM;
}

float positionNavApproachTaperMps(float cruiseSpeedMps, float slowdownM, float stillRadiusM, float distM)
{
    const float spanM = fmaxf(slowdownM - stillRadiusM, 0.01f);
    return cruiseSpeedMps * fminf(fmaxf((distM - stillRadiusM) / spanM, 0.0f), 1.0f);
}

void positionNavSetVelocityFeedforward(const vector2_t *velEfMps)
{
    if (!g_lastTarget.valid) {
        return;
    }
    g_lastTarget.targetEfM = walkedTargetEfM();
    g_targetWalkFromUs = g_stubMicros;
    g_lastFfEfMps = *velEfMps;
    g_ffValid = true;
}

static bool g_altitudeArrivalRequired;

void positionNavSetAltitudeArrivalRequired(bool required)
{
    g_altitudeArrivalRequired = required;
}

static float g_settleTimeoutS;

void positionNavSetSettleTimeout(float timeoutS)
{
    g_settleTimeoutS = timeoutS;
}

void positionNavSetMaxAngle(float angleDeg)
{
    UNUSED(angleDeg);
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
    cmd.targetPosEfM = walkedTargetEfM();
    cmd.includeAltitude = g_lastTarget.includeAltitude;
    cmd.cruiseSpeedMps = g_lastTarget.cruiseSpeedMps;
    cmd.velocityFfValid = g_ffValid;
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

vector2_t autopilotGetPositionErrorCm(void)
{
    return g_stubPositionErrorCm;
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
        g_lastApproachSlowdownM = 0.0f;
        g_lastApproachStillRadiusM = 0.0f;
        memset(&g_lastFfEfMps, 0, sizeof(g_lastFfEfMps));
        g_ffValid = false;
        g_lastAccelLimitMps2 = -1.0f;
        g_lastDecelLimitMps2 = -1.0f;
        g_setVerticalProfileCalls = 0;
        g_altHoldClimbRateCmS = 500.0f;   // alt_hold_climb_rate default, 5 m/s
        g_lastVertRateMps = 0.0f;
        g_lastVertStartAltM = 0.0f;
        memset(&g_stubPositionErrorCm, 0, sizeof(g_stubPositionErrorCm));
        g_stubCommandedAltCm = 0.0f;
        g_stubCommandedAltSet = false;
        memset(&g_stubTargetVelCmS, 0, sizeof(g_stubTargetVelCmS));
        g_clearTargetCalls = 0;
        g_startAfreshCalls = 0;
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
        cfg->maxAngle = 50;                // ap_max_angle default
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
                     uint8_t pattern = WAYPOINT_PATTERN_NONE,
                     uint8_t yawBehaviour = WAYPOINT_YAW_DEFAULT, uint16_t vertRate = 0)
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
        wp->yawBehaviour = yawBehaviour;
        wp->vertRate = vertRate;
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

TEST_F(FlightPlanNavTest, LongHoldDurationSurvivesMicrosWrap)
{
    addWaypoint(10, 20, 15000, WAYPOINT_TYPE_HOLD, 0, 43200 /* 4320 s */);
    addWaypoint(30, 40, 15000, WAYPOINT_TYPE_FLYOVER);

    g_stubMicros = 1'000'000;
    flightPlanNavEngage();
    triggerReached();

    g_stubMicros += 33'000'000;
    flightPlanNavUpdate(g_stubMicros);
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_HOLDING);

    // Advance in realistic sub-wrap intervals.  The 32-bit clock wraps while
    // the accumulated duration continues to 4319.9 seconds.
    for (int i = 0; i < 428; i++) {
        g_stubMicros += 10'000'000;
        flightPlanNavUpdate(g_stubMicros);
    }
    g_stubMicros += 6'900'000;
    flightPlanNavUpdate(g_stubMicros);
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_HOLDING);

    g_stubMicros += 100'000;
    flightPlanNavUpdate(g_stubMicros);
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
    EXPECT_EQ(flightPlanNavGetCurrentIndex(), 1);
}

TEST_F(FlightPlanNavTest, ShortHoldAcrossMicrosWrapExpiresNormally)
{
    addWaypoint(10, 20, 15000, WAYPOINT_TYPE_HOLD, 0, 20 /* 2 s */);
    addWaypoint(30, 40, 15000, WAYPOINT_TYPE_FLYOVER);

    g_stubMicros = UINT32_MAX - 1'000'000;
    flightPlanNavEngage();
    triggerReached();

    g_stubMicros += 1'500'000;
    flightPlanNavUpdate(g_stubMicros);
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_HOLDING);

    g_stubMicros += 500'000;
    flightPlanNavUpdate(g_stubMicros);
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
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

TEST_F(FlightPlanNavPatternTest, PatternCarriesTheHoldsAltitudeRampOn)
{
    // The pattern's altitude ramp carries on from the one the hold was walking rather than
    // restarting on the craft.
    engageHoldPattern(WAYPOINT_PATTERN_ORBIT);
    g_stubEstimate.position.x = (kCentreE + 1.5f) * 100.0f;
    g_stubEstimate.position.y = kCentreN * 100.0f;
    g_stubEstimate.position.z = (kCentreU - 0.4f) * 100.0f;
    g_stubCommandedAltCm = (kCentreU - 0.1f) * 100.0f;
    g_stubCommandedAltSet = true;
    triggerReached();
    flightPlanNavUpdate(g_stubMicros);
    ASSERT_EQ(g_setTargetCalls, 2);
    EXPECT_NEAR(g_lastVertStartAltM, kCentreU - 0.1f, 0.001f);
}

TEST_F(FlightPlanNavPatternTest, PatternIsAMovingTargetFromItsFirstCycle)
{
    // The pattern's carrot is a moving target from its first cycle, so the position controller
    // never acquires it as a fixed one and then steps P onto it.
    engageHoldPattern(WAYPOINT_PATTERN_ORBIT);
    triggerReached();
    const int movesBefore = g_moveTargetCalls;
    flightPlanNavUpdate(g_stubMicros);
    ASSERT_EQ(g_setTargetCalls, 2);
    EXPECT_EQ(g_moveTargetCalls, movesBefore + 1);
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
    const int movesAtStart = g_moveTargetCalls;

    // 0.25 rad/s: after 1 s the azimuth is 0.25 rad, after 2 s 0.5 rad.
    float previousAzimuth = 0.0f;
    for (int step = 1; step <= 2; step++) {
        g_stubMicros += 1'000'000;
        flightPlanNavUpdate(g_stubMicros);

        EXPECT_EQ(g_moveTargetCalls, movesAtStart + step);
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
    // A carrot leg is dispatched anchored at the craft, not at the waypoint: the carrot marches
    // out from here on the next update. The leg altitude is commanded from the outset.
    EXPECT_NEAR(g_lastTarget.targetEfM.x, g_stubEstimate.position.v[ENU_E] * 0.01f, 0.1f);
    EXPECT_NEAR(g_lastTarget.targetEfM.z, 20.0f, 0.1f); // 120 m AMSL - 100 m origin
    EXPECT_EQ(g_setVerticalProfileCalls, g_setTargetCalls);

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

TEST_F(FlightPlanNavTest, PointLegFromRestRampsItsVelocity)
{
    // A fixed target is approached through a reference walked at the commanded velocity, so from
    // rest that velocity ramps at the carrot's acceleration rather than stepping to the leg's cruise.
    addWaypoint(10, 20, 15000, WAYPOINT_TYPE_HOLD);
    flightPlanNavEngage();
    EXPECT_NEAR(g_lastAccelLimitMps2, 2.5f, 0.001f);
    EXPECT_NEAR(g_lastDecelLimitMps2, 0.3f, 0.001f);
}

TEST_F(FlightPlanNavTest, PointLegRampNeverLagsItsBrakingCurve)
{
    // nav_accel turned right down to soften the carrot's corners: a point leg still sheds speed as
    // fast as its own braking curve asks, or it reaches the point still moving and overshoots.
    autopilotConfigMutable()->navAccel = 20;
    addWaypoint(10, 20, 15000, WAYPOINT_TYPE_HOLD);
    flightPlanNavEngage();
    EXPECT_NEAR(g_lastAccelLimitMps2, 0.3f, 0.001f);
    EXPECT_NEAR(g_lastDecelLimitMps2, 0.3f, 0.001f);
}

TEST_F(FlightPlanNavTest, PointLegTakingOverRampsOutOfWhatWasCommanded)
{
    // A completed HOLD is still the active command, but the craft is at rest on it: the next point
    // leg must ramp out of that, not step straight onto its law's cruise.
    addWaypoint(10, 20, 15000, WAYPOINT_TYPE_HOLD);
    addWaypoint(30, 40, 15000, WAYPOINT_TYPE_HOLD);
    flightPlanNavEngage();
    triggerReached();
    ASSERT_EQ(flightPlanNavGetCurrentIndex(), 1);
    EXPECT_NEAR(g_lastAccelLimitMps2, 2.5f, 0.001f);
    EXPECT_NEAR(g_lastDecelLimitMps2, 0.3f, 0.001f);
}

TEST_F(FlightPlanNavTest, PointLegAfterATakeoffRampsOutOfTheClimb)
{
    addWaypoint(0, 0, 15000, WAYPOINT_TYPE_TAKEOFF);
    addWaypoint(900000, 0, 15000, WAYPOINT_TYPE_FLYOVER);   // the last waypoint: a point leg
    flightPlanNavEngage();
    triggerReached();
    ASSERT_EQ(flightPlanNavGetCurrentIndex(), 1);
    ASSERT_NE(g_lastTarget.callback, nullptr);
    EXPECT_NEAR(g_lastAccelLimitMps2, 2.5f, 0.001f);
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
    EXPECT_NEAR(g_lastApproachSlowdownM, 0.0f, 0.001f);       // no rescue taper on a mission landing
    EXPECT_NEAR(g_lastDecelLimitMps2, 0.3f, 0.001f);
    // Ramping out of what the LAND leg was commanding as it entered its radius, as a point leg does.
    EXPECT_NEAR(g_lastAccelLimitMps2, 2.5f, 0.001f);
}

TEST_F(FlightPlanNavTest, LandingTakesOverTheVerticalChannelWhereTheLegLeftIt)
{
    // A geofence landing called mid-climb: the descent starts at the altitude the leg was
    // commanding rather than snapping the altitude target onto the craft.
    autopilotConfigMutable()->maxDistanceFromHomeM = 100;
    autopilotConfigMutable()->geofenceAction = AP_GEOFENCE_LAND;
    stateFlags |= GPS_FIX_HOME;
    addWaypoint(200000, 0, 15000, WAYPOINT_TYPE_FLYOVER);
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();
    ASSERT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);

    g_stubEstimate.position.v[ENU_U] = 2000.0f;        // craft at 20 m
    g_stubCommandedAltCm = 2150.0f;                    // leg commanding 21.5 m
    g_stubCommandedAltSet = true;
    GPS_distanceToHome = 150;
    flightPlanNavUpdate(g_stubMicros + 10'000);

    ASSERT_EQ(flightPlanNavGetState(), FP_NAV_LANDING);
    EXPECT_NEAR(g_lastVertStartAltM, 21.5f, 0.01f);
    EXPECT_NEAR(g_lastVertRateMps, 0.5f, 0.01f);
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

TEST_F(FlightPlanNavSafetyTest, FlyawayMarginCoversTheSpeedTheLegWasDispatchedAt)
{
    // The rescue case: a leg dispatched while the craft is doing 17 m/s the other way. It cannot
    // help travelling its braking distance before the controller can turn it around, and a fixed
    // 20 m fence calls that a flyaway and aborts a rescue that was working.
    addWaypoint(0, 0, 11000, WAYPOINT_TYPE_HOLD);   // where the craft is, 10 m above it
    g_stubEstimate.velocity.v[ENU_N] = 1700.0f;
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();
    flightPlanNavUpdate(g_stubMicros);
    ASSERT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);

    // 30 m further out: past the old floor, inside the distance this entry speed needs.
    g_stubEstimate.position.v[ENU_N] = 3000.0f;
    flightPlanNavUpdate(g_stubMicros + 100'000);
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);

    // And well past anything the entry speed explains.
    g_stubEstimate.position.v[ENU_N] = 6000.0f;
    flightPlanNavUpdate(g_stubMicros + 200'000);
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_ABORTED);
    EXPECT_EQ(flightPlanNavGetAbortReason(), FP_ABORT_FLYAWAY);
}

TEST_F(FlightPlanNavSafetyTest, DebugReportsStateAbortAndLeg)
{
    debugMode = DEBUG_FLIGHT_PLAN;
    addWaypoint(0, 0, 11000, WAYPOINT_TYPE_HOLD);
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();
    flightPlanNavUpdate(g_stubMicros);
    EXPECT_EQ(debug[0], FP_NAV_TARGETING);
    EXPECT_EQ(debug[1], FP_ABORT_NONE);
    EXPECT_EQ(debug[2], 0);

    g_stubEstimate.position.v[ENU_N] = 3200.0f;    // drift out past the fence
    flightPlanNavUpdate(g_stubMicros + 100'000);
    flightPlanNavUpdate(g_stubMicros + 200'000);
    EXPECT_EQ(debug[0], FP_NAV_ABORTED);
    EXPECT_EQ(debug[1], FP_ABORT_FLYAWAY);
    debugMode = DEBUG_NONE;
}

TEST_F(FlightPlanNavSafetyTest, FlyawayMarginKeepsItsFloorAtRest)
{
    // A craft with no speed to shed has no braking distance to allow for, so the fence stays where
    // it was: drifting away from a target it was parked on is a flyaway.
    addWaypoint(0, 0, 11000, WAYPOINT_TYPE_HOLD);
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();
    flightPlanNavUpdate(g_stubMicros);
    ASSERT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);

    g_stubEstimate.position.v[ENU_N] = 3200.0f;
    flightPlanNavUpdate(g_stubMicros + 100'000);
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
    // A re-target rather than a continuation: the leg's velocity is not carried into the stop.
    EXPECT_EQ(g_startAfreshCalls, 1);
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
    // Dispatched anchored at the craft (150 m out) at the return altitude; the carrot carries it
    // home from there, so the destination shows up in the carrot rather than in the dispatch.
    EXPECT_NEAR(g_lastDispatchTargetEfM.x, 0.0f, 0.1f);
    EXPECT_NEAR(g_lastDispatchTargetEfM.y, 150.0f, 0.1f);
    EXPECT_NEAR(g_lastDispatchTargetEfM.z, 30.0f, 0.1f);
    EXPECT_NEAR(g_lastTarget.cruiseSpeedMps, 7.5f, 0.01f);
    EXPECT_EQ(g_startAfreshCalls, 1);   // started from the craft's motion, not the mission leg's command

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
    EXPECT_EQ(g_startAfreshCalls, 1);

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

    void addWaypointMetres(float eastM, float northM, int32_t altCm, uint8_t type,
                           uint8_t yawBehaviour = WAYPOINT_YAW_DEFAULT) {
        addWaypoint((int32_t)lrintf(northM * kUnitsPerMetre),
                    (int32_t)lrintf(eastM * kUnitsPerMetre), altCm, type,
                    0, 0, WAYPOINT_PATTERN_NONE, yawBehaviour);
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

TEST_F(FlightPlanNavCarrotTest, FaceTargetLegHoldsStationUntilTheNoseComesRound)
{
    // The rescue's return leg. Nose 180 degrees out: the position controller must be given nothing
    // to translate to until the craft is pointing at the target, or it flies home tail first.
    addWaypointMetres(0.0f, 100.0f, 15000, WAYPOINT_TYPE_FLYOVER, WAYPOINT_YAW_FACE_TARGET);
    addWaypointMetres(0.0f, 200.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    setCraftMetres(0.0f, 0.0f);
    attitude.values.yaw = 1800;   // nose south, leg due north
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();

    for (int i = 0; i < 10; i++) {
        step();
    }
    ASSERT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
    EXPECT_TRUE(g_navHeadingOverrideValid);
    EXPECT_NEAR(g_navHeadingOverrideDeg, 0.0f, 1.0f);      // nose commanded at the target
    EXPECT_NEAR(g_lastTarget.targetEfM.y, 0.0f, 0.5f);     // and the target has not moved off the craft

    attitude.values.yaw = 200;    // nose swings to within 20 degrees of the leg
    for (int i = 0; i < 10; i++) {
        step();
    }
    EXPECT_GT(g_lastTarget.targetEfM.y, 1.0f);             // now it translates
}

TEST_F(FlightPlanNavCarrotTest, FaceTargetLegSwingsTheNoseOnlyOnceTheCraftHasBraked)
{
    // Dispatched flying away from its target (a geofence return): the nose holds where it is while
    // the craft brakes, and only then swings round, rather than sweeping the brake sideways.
    addWaypointMetres(0.0f, 100.0f, 15000, WAYPOINT_TYPE_FLYOVER, WAYPOINT_YAW_FACE_TARGET);
    addWaypointMetres(0.0f, 200.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    setCraftMetres(0.0f, 0.0f);
    g_stubEstimate.velocity.v[ENU_N] = -500.0f;   // 5 m/s away from the leg
    attitude.values.yaw = 1800;
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();
    step();
    ASSERT_TRUE(g_navHeadingOverrideValid);
    EXPECT_NEAR(fabsf(g_navHeadingOverrideDeg), 180.0f, 1.0f);

    g_stubEstimate.velocity.v[ENU_N] = -100.0f;   // braked
    step();
    EXPECT_NEAR(g_navHeadingOverrideDeg, 0.0f, 1.0f);
}

TEST_F(FlightPlanNavCarrotTest, FaceTargetLegGivesUpWaitingForANoseThatWillNotTurn)
{
    // A compass that cannot deliver the heading must not leave the craft parked in the air: the
    // gate times out and the leg is flown anyway, as the legacy rotate phase did.
    addWaypointMetres(0.0f, 100.0f, 15000, WAYPOINT_TYPE_FLYOVER, WAYPOINT_YAW_FACE_TARGET);
    addWaypointMetres(0.0f, 200.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    setCraftMetres(0.0f, 0.0f);
    attitude.values.yaw = 1800;   // and it never moves
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();
    step();
    EXPECT_NEAR(g_lastTarget.targetEfM.y, 0.0f, 0.5f);

    for (int i = 0; i < 110; i++) {
        step();                   // 11 s, past the 10 s gate timeout
    }
    EXPECT_GT(g_lastTarget.targetEfM.y, 1.0f);
}

TEST_F(FlightPlanNavCarrotTest, FaceTargetLegGivesUpWaitingWhileTheCraftKeepsMoving)
{
    // A bad compass that keeps the position hold circling never lets the craft slow to where the
    // nose would swing: the nose is held only as long as the gate would wait, and then the leg is
    // flown with the nose on the target, where the heading-fault check can judge the compass.
    addWaypointMetres(0.0f, 100.0f, 15000, WAYPOINT_TYPE_FLYOVER, WAYPOINT_YAW_FACE_TARGET);
    addWaypointMetres(0.0f, 200.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    setCraftMetres(0.0f, 0.0f);
    g_stubEstimate.velocity.v[ENU_E] = 300.0f;
    attitude.values.yaw = 1800;
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();
    step();
    EXPECT_NEAR(fabsf(g_navHeadingOverrideDeg), 180.0f, 1.0f);
    EXPECT_NEAR(lastFfSpeedMps(), 0.0f, 0.01f);

    for (int i = 0; i < 110; i++) {
        step();                   // 11 s, past the 10 s gate timeout
    }
    EXPECT_NEAR(g_navHeadingOverrideDeg, 0.0f, 1.0f);
    EXPECT_GT(lastFfSpeedMps(), 1.0f);
}

TEST_F(FlightPlanNavCarrotTest, FaceTargetLegInsideTheArrivalRadiusStillWaitsForTheNose)
{
    // Dispatched 4 m out, inside the 5 m arrival radius: the leg must not count as arrived on the
    // first cycle, or a close waypoint skips the nose gate altogether.
    addWaypointMetres(0.0f, 4.0f, 15000, WAYPOINT_TYPE_FLYOVER, WAYPOINT_YAW_FACE_TARGET);
    addWaypointMetres(0.0f, 200.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    setCraftMetres(0.0f, 0.0f);
    attitude.values.yaw = 1800;   // nose south, target due north
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();
    step();

    EXPECT_EQ(flightPlanNavGetCurrentIndex(), 0);

    attitude.values.yaw = 100;    // nose comes round onto the leg
    step();
    EXPECT_EQ(flightPlanNavGetCurrentIndex(), 1);
}

TEST_F(FlightPlanNavCarrotTest, FaceTargetHoldLegStationKeepsUntilTheNoseComesRound)
{
    // Station-keeping legs are not carrot legs, so the gate holds them by commanding a station keep
    // at the craft and only issuing the real target, with its arrival gate, once the nose is round.
    addWaypointMetres(0.0f, 100.0f, 15000, WAYPOINT_TYPE_HOLD, WAYPOINT_YAW_FACE_TARGET);
    setCraftMetres(0.0f, 0.0f);
    attitude.values.yaw = 1800;   // nose south, target due north
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();
    step();

    ASSERT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
    EXPECT_NEAR(g_lastTarget.targetEfM.y, 0.0f, 0.1f);     // held at the craft
    EXPECT_EQ(g_lastTarget.callback, nullptr);             // and cannot count as arrived
    EXPECT_TRUE(g_navHeadingOverrideValid);
    EXPECT_NEAR(g_navHeadingOverrideDeg, 0.0f, 1.0f);
    EXPECT_NEAR(g_lastAccelLimitMps2, 0.0f, 0.001f);          // stopped there at once

    attitude.values.yaw = 100;    // nose comes round onto the leg
    step();
    EXPECT_NEAR(g_lastTarget.targetEfM.y, 100.0f, 0.1f);   // now the real target
    EXPECT_NE(g_lastTarget.callback, nullptr);
    EXPECT_NEAR(g_lastAccelLimitMps2, 2.5f, 0.001f);
}

TEST_F(FlightPlanNavCarrotTest, PlanCompletionHandsTheNoseBack)
{
    // A leg that commanded a heading must not keep the autopilot steering to it once the plan is
    // over and there is nothing left to fly.
    addWaypoint(0, 0, 15000, WAYPOINT_TYPE_FLYOVER, 0, 0, WAYPOINT_PATTERN_NONE, WAYPOINT_YAW_HOLD);
    attitude.values.yaw = 900;
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();
    step();
    ASSERT_TRUE(g_navHeadingOverrideValid);

    ASSERT_NE(g_lastTarget.callback, nullptr);
    g_lastTarget.callback(g_lastTarget.userData);          // arrive at the only waypoint
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_COMPLETE);
    EXPECT_FALSE(g_navHeadingOverrideValid);
}

TEST_F(FlightPlanNavCarrotTest, FaceNextLegPointsAtTheFollowingWaypointWithoutGating)
{
    // The rescue's climb-in-place leg: turn toward home while climbing, and do not gate on it.
    addWaypointMetres(0.0f, 0.0f, 15000, WAYPOINT_TYPE_HOLD, WAYPOINT_YAW_FACE_NEXT);
    addWaypointMetres(100.0f, 0.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    setCraftMetres(0.0f, 0.0f);
    attitude.values.yaw = 1800;
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();
    step();

    ASSERT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
    EXPECT_TRUE(g_navHeadingOverrideValid);
    EXPECT_NEAR(g_navHeadingOverrideDeg, 90.0f, 1.0f);     // the next waypoint is due east
}

TEST_F(FlightPlanNavCarrotTest, LegVerticalRateIsDispatchedWithTheLeg)
{
    addWaypoint(0, 0, 15000, WAYPOINT_TYPE_FLYOVER, 0, 0, WAYPOINT_PATTERN_NONE,
                WAYPOINT_YAW_DEFAULT, 250 /* cm/s */);
    addWaypointMetres(0.0f, 100.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();

    EXPECT_EQ(g_setVerticalProfileCalls, g_setTargetCalls);
    EXPECT_NEAR(g_lastVertRateMps, 2.5f, 0.01f);
    EXPECT_NEAR(g_lastVertStartAltM, g_stubEstimate.position.v[ENU_U] * 0.01f, 0.01f);
}

TEST_F(FlightPlanNavCarrotTest, CarrotTracksLegLineNotCraftCrossTrack)
{
    addWaypointMetres(0.0f, 100.0f, 15000, WAYPOINT_TYPE_FLYOVER); // wp0 (pass-through)
    addWaypointMetres(0.0f, 200.0f, 15000, WAYPOINT_TYPE_FLYOVER); // wp1 (last)
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();   // craft at the origin: leg0 is origin -> (0,100), due north
    for (int i = 0; i < 20; i++) {
        setCraftMetres(g_lastTarget.targetEfM.x, g_lastTarget.targetEfM.y);
        step();
    }

    // Wind pushes the craft 20 m east of the leg line. The carrot stays within the position
    // controller's reach of it, on the line's side, and is flown back toward the drawn line, while
    // still making way along the leg.
    const float pushedNorthM = g_lastTarget.targetEfM.y;
    setCraftMetres(20.0f, pushedNorthM);
    for (int i = 0; i < 8; i++) {
        step();
    }
    ASSERT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
    ASSERT_EQ(flightPlanNavGetCurrentIndex(), 0);
    const float gapE = g_lastTarget.targetEfM.x - 20.0f;
    const float gapN = g_lastTarget.targetEfM.y - pushedNorthM;
    EXPECT_LE(sqrtf(gapE * gapE + gapN * gapN), 5.0f + 0.01f);
    EXPECT_LT(gapE, -2.0f);
    EXPECT_LT(g_lastFfEfMps.x, 0.0f);
    EXPECT_GT(g_lastFfEfMps.y, 0.0f);

    // Riding it back, the carrot settles onto the line.
    for (int i = 0; i < 60; i++) {
        setCraftMetres(g_lastTarget.targetEfM.x, g_lastTarget.targetEfM.y);
        step();
    }
    ASSERT_EQ(flightPlanNavGetCurrentIndex(), 0);
    EXPECT_NEAR(g_lastTarget.targetEfM.x, 0.0f, 0.5f);
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
    for (int i = 0; i < 10; i++) {
        setCraftMetres(0.0f, g_lastTarget.targetEfM.y);
        step();
    }
    setCraftMetres(0.0f, 60.0f);   // overrun: shoot well past the carrot
    step();

    // The carrot is now behind the craft (braking). Nothing turns the nose back at it: the nose is
    // left to the configured yaw mode, and the carrot is still flown forward along the leg, which is
    // the bearing a bearing-steered nose follows.
    ASSERT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
    EXPECT_LT(g_lastTarget.targetEfM.y, 60.0f);
    EXPECT_FALSE(g_navHeadingOverrideValid);
    ASSERT_TRUE(g_ffValid);
    EXPECT_GT(g_lastFfEfMps.y, 0.5f);
    EXPECT_NEAR(g_lastFfEfMps.x, 0.0f, 0.01f);
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

TEST_F(FlightPlanNavCarrotTest, CarrotSpeedDoesNotDependOnMeasuredSpeed)
{
    // The carrot marches on its own trapezoid. Feeding measured ground speed back into the
    // commanded speed - a governor that stalled the carrot whenever the craft was over profile -
    // closed a loop through the vehicle: the craft ran fast, the carrot stopped, the craft ate the
    // pursuit lead and the commanded velocity collapsed a second later, apparently uncommanded.
    auto carrotSpeedAfter = [this](float craftSpeedCmS) {
        flightPlanNavDisengage();
        setCraftMetres(0.0f, 0.0f);
        g_stubEstimate.velocity.v[ENU_N] = 0.0f;
        g_stubMicros += 1'000'000;
        flightPlanNavEngage();
        step();   // anchor the leg; craft parked at the origin
        g_stubEstimate.velocity.v[ENU_N] = craftSpeedCmS;
        for (int i = 0; i < 20; i++) {
            step();
        }
        return lastFfSpeedMps();
    };

    addWaypointMetres(0.0f, 300.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    addWaypointMetres(0.0f, 600.0f, 15000, WAYPOINT_TYPE_FLYOVER); // straight on (last)

    const float onProfile = carrotSpeedAfter(200.0f);
    const float overProfile = carrotSpeedAfter(1500.0f);   // 5 m/s over the 10 m/s profile

    EXPECT_GT(onProfile, 4.0f);
    EXPECT_NEAR(overProfile, onProfile, 0.01f);
}

TEST_F(FlightPlanNavCarrotTest, CarrotStaysWithinReachOfACraftThatCannotKeepUp)
{
    // A craft held back (a headwind it cannot beat) must not leave the carrot running away down
    // the leg for the position error to wind up against: it is dragged along at the controller's
    // reach.
    addWaypointMetres(0.0f, 300.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    addWaypointMetres(0.0f, 600.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();
    for (int i = 0; i < 200; i++) {
        setCraftMetres(0.0f, i * 0.05f);   // crawling at 0.5 m/s
        step();
        const float gapE = g_lastTarget.targetEfM.x - g_stubEstimate.position.v[ENU_E] * 0.01f;
        const float gapN = g_lastTarget.targetEfM.y - g_stubEstimate.position.v[ENU_N] * 0.01f;
        ASSERT_LE(sqrtf(gapE * gapE + gapN * gapN), 5.0f + 0.01f);
    }
}

TEST_F(FlightPlanNavCarrotTest, CommandedVelocityIsTheCarrotTrapezoid)
{
    // The leg's trapezoid is the speed profile, so the carrot's velocity along the leg is what the
    // craft is commanded to fly, however far behind the carrot it sits.
    addWaypointMetres(0.0f, 300.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    addWaypointMetres(0.0f, 600.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();
    step();

    step();
    ASSERT_TRUE(g_ffValid);
    EXPECT_GT(lastFfSpeedMps(), 0.0f);
    EXPECT_LT(lastFfSpeedMps(), 10.0f);   // slewing in from a standstill, not stepping to cruise

    for (int i = 0; i < 100; i++) {
        setCraftMetres(0.0f, g_lastTarget.targetEfM.y - 3.0f);   // lagging 3 m behind the carrot
        step();
    }
    ASSERT_EQ(flightPlanNavGetCurrentIndex(), 0);
    EXPECT_GT(g_lastTarget.targetEfM.y - g_stubEstimate.position.v[ENU_N] * 0.01f, 2.0f);
    EXPECT_NEAR(g_lastFfEfMps.x, 0.0f, 0.01f);
    EXPECT_NEAR(g_lastFfEfMps.y, 10.0f, 0.01f);   // flat on the leg cruise, along the leg
}

TEST_F(FlightPlanNavCarrotTest, CarrotVelocityChangesAtTheCarrotAcceleration)
{
    // The carrot's velocity is commanded as it stands, so the carrot itself is what keeps it
    // continuous: from rest, through the corner, at no more than the carrot acceleration.
    addWaypointMetres(0.0f, 100.0f, 15000, WAYPOINT_TYPE_FLYBY);
    addWaypointMetres(100.0f, 100.0f, 15000, WAYPOINT_TYPE_FLYBY);
    addWaypointMetres(200.0f, 100.0f, 15000, WAYPOINT_TYPE_FLYBY);
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();
    EXPECT_NEAR(g_lastAccelLimitMps2, 0.0f, 0.001f);
    EXPECT_NEAR(g_lastDecelLimitMps2, 0.0f, 0.001f);
    ASSERT_TRUE(g_ffValid);
    EXPECT_NEAR(lastFfSpeedMps(), 0.0f, 0.01f);   // from rest: nothing to fly yet

    vector2_t previous = g_lastFfEfMps;
    for (int i = 0; i < 600 && g_lastTarget.targetEfM.x < 50.0f; i++) {
        setCraftMetres(g_lastTarget.targetEfM.x, g_lastTarget.targetEfM.y);
        if (g_navHeadingOverrideValid) {
            attitude.values.yaw = lrintf(g_navHeadingOverrideDeg * 10.0f);   // the nose follows its command
        }
        step();
        const float stepMps = sqrtf(sq(g_lastFfEfMps.x - previous.x) + sq(g_lastFfEfMps.y - previous.y));
        ASSERT_LE(stepMps, 2.5f * 0.1f * sqrtf(2.0f) + 0.001f) << "after " << i * 0.1f << " s";
        previous = g_lastFfEfMps;
    }
    EXPECT_EQ(flightPlanNavGetCurrentIndex(), 1);
    EXPECT_GT(g_lastFfEfMps.x, 5.0f);             // round the corner and on down the next leg
}

TEST_F(FlightPlanNavCarrotTest, DelayExpiryCarriesTheCarrotOnFromWhereItWalkedTo)
{
    // The delay's cruise cap lifted mid-leg re-issues the leg: the carrot carries on from where
    // positionNav has walked it to, not from where the executor last left it a cycle back.
    addWaypoint(0, 0, 15000, WAYPOINT_TYPE_DELAY, 0, 60);
    addWaypointMetres(0.0f, 300.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    addWaypointMetres(0.0f, 600.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();
    const int setTargetCallsAtEngage = g_setTargetCalls;
    for (int i = 0; i < 100 && g_setTargetCalls == setTargetCallsAtEngage; i++) {
        setCraftMetres(0.0f, positionNavGetActiveCommand()->targetPosEfM.y);
        g_stubMicros += 100'000;
        const float walkedNorthM = positionNavGetActiveCommand()->targetPosEfM.y;
        flightPlanNavUpdate(g_stubMicros);
        if (g_setTargetCalls != setTargetCallsAtEngage) {
            ASSERT_GT(g_lastFfEfMps.y, 1.0f);
            EXPECT_NEAR(g_lastDispatchTargetEfM.y, walkedNorthM, 0.001f);
        }
    }
    EXPECT_EQ(g_setTargetCalls, setTargetCallsAtEngage + 1);
    EXPECT_EQ(flightPlanNavGetCurrentIndex(), 1);
}

TEST_F(FlightPlanNavCarrotTest, FreshAnchorStartsAtTheSpeedTheCraftIsMakingAlongTheLeg)
{
    // A mission engaged mid-flight: the carrot starts at the craft's speed along the leg, not at
    // rest, or the craft brakes to a stop only to set off again.
    addWaypointMetres(0.0f, 300.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    addWaypointMetres(0.0f, 600.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    g_stubEstimate.velocity.v[ENU_E] = 300.0f;    // drifting east across the leg
    g_stubEstimate.velocity.v[ENU_N] = 600.0f;    // and making 6 m/s along it
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();

    ASSERT_TRUE(g_ffValid);
    EXPECT_NEAR(g_lastFfEfMps.x, 0.0f, 0.01f);    // nothing commanded across the leg
    EXPECT_NEAR(g_lastFfEfMps.y, 6.0f, 0.01f);
    // Held to where braking brings the drift across the leg to rest, not pulled back onto the craft.
    const float brakeMps2 = 9.80665f * tanf(50.0f * M_PIf / 180.0f);
    EXPECT_NEAR(g_lastTarget.targetEfM.x, 3.0f * (3.0f / (2.0f * brakeMps2) + 0.15f), 0.01f);
    EXPECT_NEAR(g_lastTarget.targetEfM.y, 0.0f, 0.01f);
    step();
    step();
    EXPECT_GT(g_lastFfEfMps.y, 6.0f);             // and carries on building from there
}

TEST_F(FlightPlanNavCarrotTest, FreshAnchorFasterThanTheLegStartsWhereTheCraftSlowsOntoIt)
{
    // Engaged at 12 m/s on a 4 m/s leg: the carrot sets off at 4 m/s from where the craft, braking,
    // comes down to that speed, so it is neither overrun nor held back below 4 m/s.
    addWaypoint(lrintf(300.0f * kUnitsPerMetre), 0, 15000, WAYPOINT_TYPE_FLYOVER, 400);
    addWaypointMetres(0.0f, 600.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    g_stubEstimate.velocity.v[ENU_N] = 1200.0f;
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();

    ASSERT_TRUE(g_ffValid);
    EXPECT_NEAR(g_lastFfEfMps.y, 4.0f, 0.01f);
    const float brakeMps2 = 9.80665f * tanf(50.0f * M_PIf / 180.0f);
    EXPECT_NEAR(g_lastTarget.targetEfM.y, 8.0f * (8.0f / (2.0f * brakeMps2) + 0.15f), 0.01f);
    EXPECT_NEAR(g_lastTarget.targetEfM.x, 0.0f, 0.01f);
}

TEST_F(FlightPlanNavCarrotTest, FreshAnchorNeverStartsFlyingTheCraftBackwards)
{
    addWaypointMetres(0.0f, 300.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    addWaypointMetres(0.0f, 600.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    g_stubEstimate.velocity.v[ENU_N] = -800.0f;   // flying away from the leg at 8 m/s
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();

    ASSERT_TRUE(g_ffValid);
    EXPECT_NEAR(lastFfSpeedMps(), 0.0f, 0.01f);
}

TEST_F(FlightPlanNavCarrotTest, CursorJumpStartsTheCommandedVelocityAfresh)
{
    // A jump back to a waypoint behind the craft: the carrot states nothing along the new leg, and
    // positionNav does not carry the old leg's command on.
    addWaypointMetres(0.0f, 300.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    addWaypointMetres(0.0f, 600.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    addWaypointMetres(0.0f, 900.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    g_stubEstimate.velocity.v[ENU_N] = 1000.0f;
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();
    setCraftMetres(0.0f, 297.0f);
    step();
    ASSERT_EQ(flightPlanNavGetCurrentIndex(), 1);
    EXPECT_EQ(g_startAfreshCalls, 0);             // a gate carries the velocity on

    setCraftMetres(0.0f, 400.0f);
    ASSERT_TRUE(flightPlanNavSetCurrentIndex(0));
    EXPECT_EQ(g_startAfreshCalls, 1);
    ASSERT_TRUE(g_ffValid);
    EXPECT_NEAR(lastFfSpeedMps(), 0.0f, 0.01f);
}

TEST_F(FlightPlanNavCarrotTest, CarrotAfterAPointLegStartsWhereTheCraftIsHeld)
{
    // A HOLD between two carrot legs, the craft held a little downwind of it: the leg after it
    // starts on the point the position controller was holding the craft to, so P carries on
    // holding against the wind rather than dropping to nothing at the hand-over. Not from the gate
    // of the carrot leg before the hold, and not at that leg's speed.
    addWaypointMetres(0.0f, 100.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    addWaypointMetres(50.0f, 100.0f, 15000, WAYPOINT_TYPE_HOLD);
    addWaypointMetres(50.0f, 300.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    addWaypointMetres(50.0f, 600.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();
    for (int i = 0; i < 60; i++) {
        step();
    }
    setCraftMetres(0.0f, 97.0f);                  // through the first gate
    step();
    ASSERT_EQ(flightPlanNavGetCurrentIndex(), 1);

    setCraftMetres(49.58f, 100.0f);               // held downwind of the HOLD point
    g_stubPositionErrorCm.x = 42.0f;
    triggerReached();
    ASSERT_EQ(flightPlanNavGetCurrentIndex(), 2);
    EXPECT_NEAR(g_lastTarget.targetEfM.x, 50.0f, 0.01f);
    EXPECT_NEAR(g_lastTarget.targetEfM.y, 100.0f, 0.01f);
    EXPECT_NEAR(lastFfSpeedMps(), 0.0f, 0.01f);
    step();
    EXPECT_NEAR(g_lastTarget.targetEfM.x, 50.0f, 0.1f);
    EXPECT_LT(lastFfSpeedMps(), 1.0f);
}

TEST_F(FlightPlanNavCarrotTest, CarrotAfterAPointLegTurnsOutOfWhatItCommanded)
{
    // A HOLD completed on entering its radius, still closing on it at right angles to the leg after
    // it: the carrot sets off at the velocity the hold was commanding and turns onto the leg within
    // the carrot acceleration, rather than dropping the part across the leg in one cycle.
    addWaypointMetres(50.0f, 100.0f, 15000, WAYPOINT_TYPE_HOLD);
    addWaypointMetres(50.0f, 300.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    addWaypointMetres(50.0f, 600.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    setCraftMetres(48.0f, 100.0f);
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();
    g_stubTargetVelCmS.x = 110.0f;                // the hold's chase law, 2 m out
    triggerReached();
    ASSERT_EQ(flightPlanNavGetCurrentIndex(), 1);
    EXPECT_NEAR(g_lastFfEfMps.x, 1.1f, 0.01f);
    EXPECT_NEAR(g_lastFfEfMps.y, 0.0f, 0.01f);

    const float budgetMps = autopilotConfig()->navAccel * 0.01f * 0.1f;
    vector2_t previousMps = g_lastFfEfMps;
    for (int i = 0; i < 30; i++) {
        step();
        const vector2_t deltaMps = { .x = g_lastFfEfMps.x - previousMps.x, .y = g_lastFfEfMps.y - previousMps.y };
        EXPECT_LE(vector2Norm(&deltaMps), 1.5f * budgetMps) << "at " << i;
        previousMps = g_lastFfEfMps;
    }
    EXPECT_GT(g_lastFfEfMps.y, 1.0f);
    EXPECT_LT(fabsf(g_lastFfEfMps.x), 0.2f);
}

TEST_F(FlightPlanNavCarrotTest, CarrotAfterATakeoffStartsWhereTheCraftIsHeld)
{
    addWaypointMetres(0.0f, 0.0f, 15000, WAYPOINT_TYPE_TAKEOFF);
    addWaypointMetres(0.0f, 100.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    addWaypointMetres(0.0f, 200.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    setCraftMetres(0.0f, 0.0f);
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();
    ASSERT_NEAR(g_lastTarget.targetEfM.x, 0.0f, 0.01f);

    setCraftMetres(-0.42f, 0.0f);                 // held downwind of the point
    g_stubPositionErrorCm.x = 42.0f;
    triggerReached();
    ASSERT_EQ(flightPlanNavGetCurrentIndex(), 1);
    ASSERT_TRUE(g_ffValid);
    EXPECT_NEAR(g_lastTarget.targetEfM.x, 0.0f, 0.01f);
    EXPECT_NEAR(g_lastTarget.targetEfM.y, 0.0f, 0.01f);
}

TEST_F(FlightPlanNavCarrotTest, CarrotStartedAfreshIgnoresTheLegItReplaces)
{
    // A cursor jump: the carrot starts on the craft at what it is making along the new leg, not on
    // the point the old leg held it to or at what that leg commanded.
    addWaypointMetres(0.0f, 300.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    addWaypointMetres(0.0f, 600.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    addWaypointMetres(0.0f, 900.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();
    step();
    setCraftMetres(3.0f, 100.0f);
    g_stubEstimate.velocity.v[ENU_N] = 200.0f;
    g_stubPositionErrorCm.x = -300.0f;
    g_stubTargetVelCmS.y = 900.0f;
    ASSERT_TRUE(flightPlanNavSetCurrentIndex(1));
    EXPECT_NEAR(g_lastTarget.targetEfM.x, 3.0f, 0.01f);
    EXPECT_NEAR(g_lastTarget.targetEfM.y, 100.0f, 0.01f);
    EXPECT_NEAR(g_lastFfEfMps.y, 2.0f, 0.05f);
}

TEST_F(FlightPlanNavCarrotTest, LagCompensationCrossesGateNearCornerSpeed)
{
    addWaypointMetres(0.0f, 300.0f, 15000, WAYPOINT_TYPE_FLYBY);   // 90 deg corner here
    addWaypointMetres(300.0f, 300.0f, 15000, WAYPOINT_TYPE_FLYBY); // east leg (last)
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();
    step();

    // The craft flies the commanded velocity a little late: a first-order lag on the carrot's
    // velocity, the position controller's share of the lag the brake compensation exists for.
    // Without it the carrot reaches corner speed at the gate but the craft, answering late, crosses
    // hot by about lag * decel.
    const float dt = 0.1f;
    const float tauS = 0.3f;
    float craftE = 0.0f;
    float craftN = 0.0f;
    float velE = 0.0f;
    float velN = 0.0f;
    float crossingSpeedMps = -1.0f;
    float maxSpeedMps = 0.0f;
    for (int i = 0; i < 1500 && crossingSpeedMps < 0.0f; i++) {
        velE += (g_lastFfEfMps.x - velE) * dt / tauS;
        velN += (g_lastFfEfMps.y - velN) * dt / tauS;
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
    EXPECT_LT(crossingSpeedMps, 3.6f);
    EXPECT_GT(crossingSpeedMps, 2.6f);   // nor crawling in for having braked far too early
}

// A craft that flies the commanded velocity a little late, with the position error pulling it in
// behind: enough of the controller to see what the executor's commands do to the position error.
struct LaggingCraft {
    float e = 0.0f, n = 0.0f, ve = 0.0f, vn = 0.0f;
    void follow(const vector3_t &carrotM, const vector2_t &ffMps, float dt) {
        const float tauS = 0.3f;
        const float pullPerS = 0.5f;
        ve += (ffMps.x + pullPerS * (carrotM.x - e) - ve) * dt / tauS;
        vn += (ffMps.y + pullPerS * (carrotM.y - n) - vn) * dt / tauS;
        e += ve * dt;
        n += vn * dt;
    }
};

TEST_F(FlightPlanNavCarrotTest, FaceTargetCornerSwingsTheNoseAtOnce)
{
    // Two face-the-target legs meeting at a 90 degree corner. The leg handed over at the gate sheds
    // its speed at the carrot's own acceleration, not at full brake, so its nose swings for the new
    // leg straight away rather than holding until the craft has all but stopped.
    addWaypointMetres(0.0f, 100.0f, 15000, WAYPOINT_TYPE_FLYOVER, WAYPOINT_YAW_FACE_TARGET);
    addWaypointMetres(100.0f, 100.0f, 15000, WAYPOINT_TYPE_FLYOVER, WAYPOINT_YAW_FACE_TARGET);
    addWaypointMetres(200.0f, 100.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();

    const float dt = 0.02f;
    LaggingCraft craft;
    for (int i = 0; i < 20000 && flightPlanNavGetCurrentIndex() == 0; i++) {
        craft.follow(g_lastTarget.targetEfM, g_lastFfEfMps, dt);
        setCraftMetres(craft.e, craft.n);
        g_stubEstimate.velocity.v[ENU_E] = craft.ve * 100.0f;
        g_stubEstimate.velocity.v[ENU_N] = craft.vn * 100.0f;
        step(20'000);
    }
    ASSERT_EQ(flightPlanNavGetCurrentIndex(), 1);
    ASSERT_GT(sqrtf(sq(craft.ve) + sq(craft.vn)), 2.0f);   // crossing the gate at speed
    EXPECT_NEAR(g_navHeadingOverrideDeg, 90.0f, 15.0f);    // and the nose already sent round
}

TEST_F(FlightPlanNavCarrotTest, FaceTargetHoldTakenOverAtAGateSwingsTheNoseOnlyOnceBraked)
{
    // A face-the-target HOLD well off the nose, taken over at a carrot leg's gate, stops the craft
    // where it is rather than shedding speed at the carrot's acceleration, so its nose waits for the
    // brake as a leg taking over afresh does.
    addWaypointMetres(0.0f, 100.0f, 15000, WAYPOINT_TYPE_FLYOVER, WAYPOINT_YAW_FACE_TARGET);
    addWaypointMetres(60.0f, 100.0f, 15000, WAYPOINT_TYPE_HOLD, WAYPOINT_YAW_FACE_TARGET);
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();

    const float dt = 0.02f;
    LaggingCraft craft;
    for (int i = 0; i < 20000 && flightPlanNavGetCurrentIndex() == 0; i++) {
        craft.follow(g_lastTarget.targetEfM, g_lastFfEfMps, dt);
        setCraftMetres(craft.e, craft.n);
        g_stubEstimate.velocity.v[ENU_E] = craft.ve * 100.0f;
        g_stubEstimate.velocity.v[ENU_N] = craft.vn * 100.0f;
        step(20'000);
    }
    ASSERT_EQ(flightPlanNavGetCurrentIndex(), 1);
    ASSERT_GT(sqrtf(sq(craft.ve) + sq(craft.vn)), 2.0f);   // crossing the gate at speed
    EXPECT_NEAR(g_lastAccelLimitMps2, 0.0f, 0.001f);          // stopped where it is
    EXPECT_NEAR(g_navHeadingOverrideDeg, 0.0f, 1.0f);      // with the nose held on the leg it came off

    g_stubEstimate.velocity.v[ENU_E] = 0.0f;
    g_stubEstimate.velocity.v[ENU_N] = 100.0f;             // braked
    step(20'000);
    EXPECT_NEAR(g_navHeadingOverrideDeg, RADIANS_TO_DEGREES(atan2f(60.0f - craft.e, 100.0f - craft.n)), 1.0f);
}

TEST_F(FlightPlanNavCarrotTest, CarrotKeepsTimeThroughAGate)
{
    // The update that crosses a gate still moves the carrot on by the time since the last one, or
    // every gate leaves it that far behind for good.
    addWaypointMetres(0.0f, 100.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    addWaypointMetres(0.0f, 200.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    addWaypointMetres(0.0f, 300.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();
    for (int i = 0; i < 1000 && flightPlanNavGetCurrentIndex() == 0; i++) {
        const vector3_t carrot = g_lastTarget.targetEfM;
        const vector2_t ff = g_lastFfEfMps;
        setCraftMetres(carrot.x, carrot.y - 1.0f);
        step(50'000);
        if (flightPlanNavGetCurrentIndex() == 1) {
            ASSERT_GT(ff.y, 5.0f);
            EXPECT_NEAR(g_lastTarget.targetEfM.y, carrot.y + ff.y * 0.05f, 0.001f);
        }
    }
    EXPECT_EQ(flightPlanNavGetCurrentIndex(), 1);
}

TEST_F(FlightPlanNavCarrotTest, CarrotCarriesStraightOnThroughAGate)
{
    // Two carrot legs meeting at a 90 degree FLYBY corner. The craft sits close behind a carrot it is
    // flying the velocity of, so a carrot that started the next leg back on the waypoint - a whole
    // gate radius ahead of the craft - would step the position error by that much at every corner.
    // Neither the carrot the craft is held to nor the velocity it is commanded may step, at the
    // gate crossing or anywhere else.
    addWaypointMetres(0.0f, 300.0f, 15000, WAYPOINT_TYPE_FLYBY);
    addWaypointMetres(300.0f, 300.0f, 15000, WAYPOINT_TYPE_FLYBY);
    addWaypointMetres(300.0f, 600.0f, 15000, WAYPOINT_TYPE_FLYBY);
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();

    const float dt = 0.02f;
    LaggingCraft craft;
    vector3_t previousCarrot = g_lastTarget.targetEfM;
    vector2_t previousFf = g_lastFfEfMps;
    float largestGapM = 0.0f;
    float largestCarrotStepM = 0.0f;
    float largestFfStepMps = 0.0f;
    bool crossed = false;
    for (int i = 0; i < 20000 && flightPlanNavGetCurrentIndex() < 2; i++) {
        craft.follow(g_lastTarget.targetEfM, g_lastFfEfMps, dt);
        setCraftMetres(craft.e, craft.n);
        g_stubEstimate.velocity.v[ENU_E] = craft.ve * 100.0f;
        g_stubEstimate.velocity.v[ENU_N] = craft.vn * 100.0f;
        attitude.values.yaw = lrintf(10.0f * RADIANS_TO_DEGREES(atan2f(craft.ve, craft.vn)));
        step(20'000);
        if (flightPlanNavGetCurrentIndex() == 2) {
            break;   // the last leg is a point leg: its target is the waypoint itself
        }
        crossed = crossed || flightPlanNavGetCurrentIndex() == 1;
        const vector3_t carrot = g_lastTarget.targetEfM;
        largestCarrotStepM = fmaxf(largestCarrotStepM, sqrtf(sq(carrot.x - previousCarrot.x) + sq(carrot.y - previousCarrot.y)));
        largestFfStepMps = fmaxf(largestFfStepMps, sqrtf(sq(g_lastFfEfMps.x - previousFf.x) + sq(g_lastFfEfMps.y - previousFf.y)));
        if (crossed) {
            largestGapM = fmaxf(largestGapM, sqrtf(sq(carrot.x - craft.e) + sq(carrot.y - craft.n)));
        }
        previousCarrot = carrot;
        previousFf = g_lastFfEfMps;
    }
    ASSERT_TRUE(crossed);
    EXPECT_LT(largestCarrotStepM, 10.0f * dt + 0.01f);        // never faster than the leg cruise
    EXPECT_LT(largestFfStepMps, 2.5f * dt * 1.5f + 0.01f);    // never faster than the carrot acceleration
    EXPECT_LT(largestGapM, 2.0f);                              // and the craft is never left behind it
}

TEST_F(FlightPlanNavCarrotTest, LegIntoAPointAtTheSameSpotBrakesToItsApproachSpeed)
{
    // A FLYOVER followed by a LAND on the same point (the geofence return): the point leg picks up
    // at its own approach speed at the gate, so the carrot brakes into that rather than handing it
    // cruise speed to shed past the point.
    addWaypointMetres(0.0f, 200.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    addWaypointMetres(0.0f, 200.0f, 15000, WAYPOINT_TYPE_LAND);
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();

    const float dt = 0.02f;
    LaggingCraft craft;
    float lastCarrotSpeedMps = 0.0f;
    float fastestMps = 0.0f;
    for (int i = 0; i < 20000 && flightPlanNavGetCurrentIndex() == 0; i++) {
        lastCarrotSpeedMps = lastFfSpeedMps();
        fastestMps = fmaxf(fastestMps, lastCarrotSpeedMps);
        craft.follow(g_lastTarget.targetEfM, g_lastFfEfMps, dt);
        setCraftMetres(craft.e, craft.n);
        g_stubEstimate.velocity.v[ENU_N] = craft.vn * 100.0f;
        step(20'000);
    }
    ASSERT_EQ(flightPlanNavGetCurrentIndex(), 1);
    EXPECT_GT(fastestMps, 9.0f);
    EXPECT_NEAR(lastCarrotSpeedMps, sqrtf(2.0f * 0.3f * 5.0f), 0.3f);   // sqrt(2 * approach decel * gate)
}

TEST_F(FlightPlanNavCarrotTest, HeadingFaultOnACarrotLegIsJudgedAgainstTheWaypoint)
{
    // The carrot rides with the craft, so the bearing to it is noise; eligibility for the heading
    // fault check is judged against the waypoint the leg is flying to.
    addWaypointMetres(0.0f, 2000.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    addWaypointMetres(0.0f, 4000.0f, 15000, WAYPOINT_TYPE_FLYOVER);
    g_stubMicros = 1'000'000;
    flightPlanNavEngage();
    for (int i = 0; i < 20; i++) {
        setCraftMetres(0.0f, g_lastTarget.targetEfM.y - 0.2f);
        step();
    }
    ASSERT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);

    attitude.values.yaw = 0;    // nose north, along the leg
    gpsSol.groundSpeed = 500;
    gpsSol.groundCourse = 900;  // but the ground course is east: a heading gone wrong
    for (int i = 0; i < 30 && flightPlanNavGetState() == FP_NAV_TARGETING; i++) {
        setCraftMetres(0.3f * ((i % 2) ? 1.0f : -1.0f), g_lastTarget.targetEfM.y + 0.1f);   // scatter about the carrot
        step();
    }
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_ABORTED);
    EXPECT_EQ(flightPlanNavGetAbortReason(), FP_ABORT_MAG_FAULT);
}

// MAVLink MISSION_SET_CURRENT — flightPlanNavSetCurrentIndex().
TEST_F(FlightPlanNavTest, SetCurrentIndexRejectsOutOfRange)
{
    addWaypoint(10, 20, 15000, WAYPOINT_TYPE_FLYOVER);
    addWaypoint(30, 40, 15000, WAYPOINT_TYPE_FLYOVER);

    EXPECT_FALSE(flightPlanNavSetCurrentIndex(2));   // count is 2 -> valid 0..1
    EXPECT_FALSE(flightPlanNavSetCurrentIndex(99));
}

TEST_F(FlightPlanNavTest, SetCurrentIndexRejectsWhenNoMission)
{
    EXPECT_FALSE(flightPlanNavSetCurrentIndex(0));   // empty plan: nothing to select
}

TEST_F(FlightPlanNavTest, SetCurrentIndexJumpsActiveMission)
{
    addWaypoint(10, 20, 15000, WAYPOINT_TYPE_FLYOVER);
    addWaypoint(30, 40, 15000, WAYPOINT_TYPE_FLYOVER);
    addWaypoint(50, 60, 15000, WAYPOINT_TYPE_FLYOVER);

    flightPlanNavEngage();
    ASSERT_EQ(flightPlanNavGetCurrentIndex(), 0);
    const int dispatchesBefore = g_setTargetCalls;

    EXPECT_TRUE(flightPlanNavSetCurrentIndex(2));
    EXPECT_EQ(flightPlanNavGetCurrentIndex(), 2);
    EXPECT_EQ(flightPlanNavGetState(), FP_NAV_TARGETING);
    // Jumping re-dispatches the new leg immediately.
    EXPECT_GT(g_setTargetCalls, dispatchesBefore);
}
