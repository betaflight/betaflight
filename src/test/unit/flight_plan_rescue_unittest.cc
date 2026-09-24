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

bool g_navHeadingOverrideValid;
float g_navHeadingOverrideDeg;
bool g_emergencyDescentActive;
float g_emergencyDescentRateCmS;
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

bool g_altitudeArrivalRequired;
float g_yawRateLimitDps;

float g_stubMaxAltitudeCm;
bool g_stubHeadingValid;
int g_pitchForwardCalls;
bool g_lastPitchForward;

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

void altHoldSetEmergencyDescent(bool active, float rateCmS)
{
    g_emergencyDescentActive = active;
    g_emergencyDescentRateCmS = rateCmS;
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

void autopilotSetNavHeadingOverride(bool valid, float headingDeg)
{
    g_navHeadingOverrideValid = valid;
    g_navHeadingOverrideDeg = headingDeg;
}

vector2_t autopilotGetPositionErrorCm(void)
{
    return g_stubPositionErrorCm;
}

} // extern "C"

class FlightPlanRescueTest : public ::testing::Test {
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
        g_navHeadingOverrideValid = false;
        g_navHeadingOverrideDeg = 0.0f;
        g_emergencyDescentActive = false;
        g_emergencyDescentRateCmS = 0.0f;
        g_altHoldClimbRateCmS = 500.0f;   // alt_hold_climb_rate default, 5 m/s
        g_lastVertRateMps = 0.0f;
        g_lastVertStartAltM = 0.0f;
        memset(&g_stubPositionErrorCm, 0, sizeof(g_stubPositionErrorCm));
        g_stubCommandedAltCm = 0.0f;
        g_stubCommandedAltSet = false;
        memset(&g_stubTargetVelCmS, 0, sizeof(g_stubTargetVelCmS));
        g_clearTargetCalls = 0;
        g_startAfreshCalls = 0;
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
        cfg->maxAngle = 50;                // ap_max_angle default
        // Leg-line carrot tracking (PG reset template is not applied under test): without these the
        // carrot cannot accelerate and every pass-gate leg sits still.
        cfg->navCornerSpeed = 220;
        cfg->navCornerDeltaV = 440;
        cfg->navDecel = 250;
        cfg->navAccel = 250;
        cfg->navCarrotLeadTime = 12;
        cfg->navCarrotLeadMax = 2500;
        cfg->navPreturnDist = 1500;

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

TEST_F(FlightPlanRescueTest, RescuePlanStatesItsOwnRatesAndNoseBehaviour)
{
    gpsRescueConfigMutable()->ascendRate = 200;
    gpsRescueConfigMutable()->descendRate = 150;

    ASSERT_TRUE(flightPlanNavStageRescuePlan());
    flightPlanNavEngage();

    // The climb leg carries the configured ascend rate and turns the nose toward home while it
    // climbs; nothing downstream has to work out that a rescue is what is flying.
    EXPECT_NEAR(g_lastVertRateMps, gpsRescueConfig()->ascendRate * 0.01f, 0.01f);
    EXPECT_NEAR(g_lastVertStartAltM, g_stubEstimate.position.v[ENU_U] * 0.01f, 0.01f);
    EXPECT_EQ(g_setVerticalProfileCalls, g_setTargetCalls);

    g_stubEstimate.position.v[ENU_E] = 30.0f * 100.0f;   // out east, where the rescue was called
    triggerReached();                       // climb done, return leg dispatched
    ASSERT_EQ(flightPlanNavGetCurrentIndex(), 1);
    EXPECT_TRUE(g_navHeadingOverrideValid); // nose commanded at home, not left where it was
    EXPECT_NEAR(g_navHeadingOverrideDeg, -90.0f, 1.0f);  // home is due west of the craft

    g_stubEstimate.position.v[ENU_E] = 0.0f;             // home reached
    g_stubMicros += 100'000;
    flightPlanNavUpdate(g_stubMicros);      // carrot gate: arrive home, dispatch the LAND leg
    ASSERT_EQ(flightPlanNavGetCurrentIndex(), 2);
    triggerReached();
    ASSERT_EQ(flightPlanNavGetState(), FP_NAV_LANDING);

    // And the descent is flown at the configured descend rate, rather than at whatever a target
    // 200 m below the ground drives the altitude controller to.
    EXPECT_NEAR(g_lastVertRateMps, gpsRescueConfig()->descendRate * 0.01f, 0.01f);
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

TEST_F(FlightPlanRescueTest, HeadingRecoveryStartsTheReturnWhereTheCraftComesToRest)
{
    // Pitched forward for up to 15 s to find its heading, the craft is nowhere near the climb's hold
    // point by the time it has one, and still flying fast. The return leg starts from where it comes
    // to rest braking out of the pitch-forward, not from that point, nor from the craft.
    g_stubEstimate.position.v[ENU_E] = 30.0f * 100.0f;
    ASSERT_TRUE(flightPlanNavStageRescuePlan());
    flightPlanNavEngage();
    g_stubHeadingValid = false;
    triggerReached();
    ASSERT_TRUE(g_lastPitchForward);

    const float brakeMps2 = 9.80665f * tanf(50.0f * M_PIf / 180.0f);
    float northM = 25.0f;                               // pitched forward, north
    float speedMps = 15.0f;
    g_stubEstimate.position.v[ENU_N] = northM * 100.0f;
    g_stubEstimate.velocity.v[ENU_N] = speedMps * 100.0f;
    attitude.values.yaw = 0;                            // nose north, well off home
    g_stubHeadingValid = true;
    g_stubMicros += 100'000;
    flightPlanNavUpdate(g_stubMicros);
    ASSERT_EQ(flightPlanNavGetCurrentIndex(), 1);
    EXPECT_FALSE(g_lastPitchForward);
    const float restNorthM = northM + speedMps * (speedMps / (2.0f * brakeMps2) + 0.15f);
    EXPECT_NEAR(g_lastTarget.targetEfM.x, 30.0f, 0.01f);
    EXPECT_NEAR(g_lastTarget.targetEfM.y, restNorthM, 0.01f);
    ASSERT_GT(restNorthM - northM, 5.0f);               // further on than the controller's reach

    // Nothing is commanded until the nose is round, and the craft braking out of the pitch-forward
    // is held to that point all the way in, not dragged back from where its brake ends.
    ASSERT_TRUE(g_ffValid);
    EXPECT_NEAR(lastFfSpeedMps(), 0.0f, 0.001f);
    const float dtS = 0.05f;
    for (float t = 0.0f; speedMps > 0.0f; t += dtS) {
        speedMps = (t < 0.15f) ? speedMps : fmaxf(speedMps - brakeMps2 * dtS, 0.0f);
        northM += speedMps * dtS;
        g_stubEstimate.position.v[ENU_N] = northM * 100.0f;
        g_stubEstimate.velocity.v[ENU_N] = speedMps * 100.0f;
        g_stubMicros += 50'000;
        flightPlanNavUpdate(g_stubMicros);
        EXPECT_NEAR(lastFfSpeedMps(), 0.0f, 0.001f);
        EXPECT_NEAR(g_lastTarget.targetEfM.y, restNorthM, 0.01f) << "at " << t << " s";
    }
    EXPECT_NEAR(northM, restNorthM, 0.5f);
}

TEST_F(FlightPlanRescueTest, HeadingRecoveryCarriesTheCommandedAltitudeOn)
{
    // Pitching forward, the craft sagged below the altitude the climb had brought its target to.
    // The return leg's ramp starts from that target, not from the craft.
    ASSERT_TRUE(flightPlanNavStageRescuePlan());
    flightPlanNavEngage();
    g_stubHeadingValid = false;
    triggerReached();
    ASSERT_TRUE(g_lastPitchForward);

    g_stubCommandedAltCm = kDefaultReturnAltM * 100.0f;
    g_stubCommandedAltSet = true;
    g_stubEstimate.position.v[ENU_U] = (kDefaultReturnAltM - 0.6f) * 100.0f;
    const int clearsBefore = g_clearTargetCalls;
    g_stubHeadingValid = true;
    g_stubMicros += 100'000;
    flightPlanNavUpdate(g_stubMicros);
    ASSERT_EQ(flightPlanNavGetCurrentIndex(), 1);
    EXPECT_NEAR(g_lastVertStartAltM, kDefaultReturnAltM, 0.001f);
    EXPECT_EQ(g_clearTargetCalls, clearsBefore);
    EXPECT_EQ(g_startAfreshCalls, 1);   // and does not carry the climb's command on
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

TEST_F(FlightPlanRescueTest, DescentStartsAtTheConfiguredDescentDistance)
{
    // Legacy rescue starts down at gps_rescue_descent_dist and closes the last stretch while
    // descending. Arriving overhead first and only then sinking puts the craft over whatever it was
    // trying to get away from, and the pilot loses the approach they were expecting.
    gpsRescueConfigMutable()->descentDistanceM = 15;
    g_stubEstimate.position.v[ENU_N] = 2500.0f;   // 25 m north of home
    attitude.values.yaw = 1800;                   // nose south, pointing home
    ASSERT_TRUE(flightPlanNavStageRescuePlan());
    flightPlanNavEngage();
    triggerReached();                             // climb done, fly home
    ASSERT_EQ(flightPlanNavGetCurrentIndex(), 1);

    g_stubMicros += 100'000;
    flightPlanNavUpdate(g_stubMicros);
    EXPECT_EQ(flightPlanNavGetCurrentIndex(), 1);  // 25 m out: still on the return leg

    g_stubEstimate.position.v[ENU_N] = 1400.0f;    // inside the descent distance
    g_stubMicros += 100'000;
    flightPlanNavUpdate(g_stubMicros);
    EXPECT_EQ(flightPlanNavGetCurrentIndex(), 2);  // hands over to the landing leg out here
    EXPECT_NEAR(g_lastTarget.acceptanceRadiusM, 15.0f, 0.01f);
}

TEST_F(FlightPlanRescueTest, LandingLegStartsDownAtTheDescentDistanceBelowTheReturnAltitude)
{
    // An approach that has sunk 0.7 m below the return altitude still starts down at the descent
    // distance, rather than carrying on toward home until it is back at that altitude.
    gpsRescueConfigMutable()->descentDistanceM = 7;
    g_stubEstimate.position.v[ENU_N] = 2000.0f;   // 20 m north of home
    attitude.values.yaw = 1800;                   // nose south, pointing home
    ASSERT_TRUE(flightPlanNavStageRescuePlan());
    flightPlanNavEngage();
    triggerReached();                             // climb done, fly home
    ASSERT_EQ(flightPlanNavGetCurrentIndex(), 1);
    const float returnAltM = g_lastTarget.targetEfM.z;

    g_stubEstimate.position.v[ENU_N] = 650.0f;
    g_stubEstimate.position.v[ENU_U] = (returnAltM - 0.7f) * 100.0f;
    g_stubMicros += 100'000;
    flightPlanNavUpdate(g_stubMicros);
    ASSERT_EQ(flightPlanNavGetCurrentIndex(), 2);

    // Arrives on the spot: inside its radius, and nothing waiting on the altitude.
    EXPECT_FALSE(g_altitudeArrivalRequired);
    EXPECT_GE(g_lastTarget.acceptanceRadiusM, 6.5f);
    // Nor has the hand-over dropped the altitude target onto the craft.
    EXPECT_NEAR(g_lastVertStartAltM, returnAltM, 0.01f);

    // Its ramp had been held on its leash below the return altitude and was climbing back.
    g_stubCommandedAltCm = (returnAltM - 0.5f) * 100.0f;
    g_stubCommandedAltSet = true;
    triggerReached();
    ASSERT_EQ(flightPlanNavGetState(), FP_NAV_LANDING);
    // The descent starts from the altitude being commanded.
    EXPECT_NEAR(g_lastVertStartAltM, returnAltM - 0.5f, 0.01f);
}

TEST_F(FlightPlanRescueTest, ReturnLegBleedsSpeedFromTwiceTheDescentDistance)
{
    // Legacy slowed from twice the descent distance so it arrived slow at the point it starts down
    // at. Arriving at full return speed and braking on the doorstep overruns home when the craft
    // comes in hot and the descent distance is short.
    gpsRescueConfigMutable()->descentDistanceM = 10;   // 20 m slowdown range
    g_stubEstimate.position.v[ENU_N] = 1500.0f;        // 15 m north of home: three quarters out
    attitude.values.yaw = 1800;                        // nose south, pointing home
    ASSERT_TRUE(flightPlanNavStageRescuePlan());
    flightPlanNavEngage();
    triggerReached();
    ASSERT_EQ(flightPlanNavGetCurrentIndex(), 1);

    for (int i = 0; i < 30; i++) {
        g_stubMicros += 100'000;
        flightPlanNavUpdate(g_stubMicros);
    }
    ASSERT_EQ(flightPlanNavGetCurrentIndex(), 1);
    // 15 m out on a taper from 20 m down to nothing 1 m from home, against the 7.5 m/s return
    // speed, commanded along the leg home.
    ASSERT_TRUE(g_ffValid);
    EXPECT_NEAR(g_lastFfEfMps.x, 0.0f, 0.01f);
    EXPECT_NEAR(g_lastFfEfMps.y, -7.5f * (15.0f - 1.0f) / (20.0f - 1.0f), 0.01f);
}

TEST_F(FlightPlanRescueTest, DescentCarriesOnTheReturnTaper)
{
    // One speed law all the way in, as legacy flew: the return speed scaled by the distance left,
    // from twice the descent distance down. The landing leg and the descent carry on that same law
    // from the hand-over at the descent distance, so the target velocity does not step there.
    struct { uint16_t speedCmS; uint16_t descentDistM; uint16_t descendRateCmS; } configs[] = {
        { 400, 7, 200 },     // the tester's
        { 750, 20, 150 },    // defaults
        { 1500, 5, 500 },
    };
    for (const auto &c : configs) {
        SetUp();
        gpsRescueConfigMutable()->groundSpeedCmS = c.speedCmS;
        gpsRescueConfigMutable()->descentDistanceM = c.descentDistM;
        gpsRescueConfigMutable()->descendRate = c.descendRateCmS;
        const float speedMps = c.speedCmS * 0.01f;
        const float slowdownM = 2.0f * c.descentDistM;
        const auto taperMps = [&](float distM) {
            return speedMps * fminf(fmaxf((distM - 1.0f) / (slowdownM - 1.0f), 0.0f), 1.0f);
        };

        g_stubEstimate.position.v[ENU_N] = 3.0f * c.descentDistM * 100.0f;
        attitude.values.yaw = 1800;               // nose south, pointing home
        ASSERT_TRUE(flightPlanNavStageRescuePlan());
        flightPlanNavEngage();
        triggerReached();                         // climb done, fly home
        ASSERT_EQ(flightPlanNavGetCurrentIndex(), 1);

        const float outsideM = c.descentDistM + 0.3f;
        g_stubEstimate.position.v[ENU_N] = outsideM * 100.0f;
        for (int i = 0; i < 100; i++) {
            g_stubMicros += 100'000;
            flightPlanNavUpdate(g_stubMicros);
        }
        ASSERT_EQ(flightPlanNavGetCurrentIndex(), 1);
        EXPECT_NEAR(-g_lastFfEfMps.y, taperMps(outsideM), 0.01f);

        g_stubEstimate.position.v[ENU_N] = (c.descentDistM - 0.1f) * 100.0f;
        g_stubMicros += 100'000;
        flightPlanNavUpdate(g_stubMicros);
        ASSERT_EQ(flightPlanNavGetCurrentIndex(), 2);
        // The landing leg flies positionNav's copy of the taper: same speed, range and still radius,
        // as it stands, with no braking curve under it or ramp in front of it.
        EXPECT_NEAR(g_lastTarget.cruiseSpeedMps, speedMps, 0.001f);
        EXPECT_NEAR(g_lastApproachSlowdownM, slowdownM, 0.001f);
        EXPECT_NEAR(g_lastApproachStillRadiusM, 1.0f, 0.001f);
        EXPECT_NEAR(g_lastDecelLimitMps2, 0.0f, 0.001f);
        EXPECT_NEAR(g_lastAccelLimitMps2, 0.0f, 0.001f);

        triggerReached();
        ASSERT_EQ(flightPlanNavGetState(), FP_NAV_LANDING);
        // And so does the descent, however slowly it is coming down.
        EXPECT_NEAR(g_lastTarget.cruiseSpeedMps, speedMps, 0.001f);
        EXPECT_NEAR(g_lastApproachSlowdownM, slowdownM, 0.001f);
        EXPECT_NEAR(g_lastApproachStillRadiusM, 1.0f, 0.001f);
        EXPECT_NEAR(g_lastDecelLimitMps2, 0.0f, 0.001f);
        EXPECT_NEAR(g_lastAccelLimitMps2, 0.0f, 0.001f);
        EXPECT_NEAR(g_lastVertRateMps, c.descendRateCmS * 0.01f, 0.001f);
        flightPlanNavDisengage();
    }
}

TEST_F(FlightPlanRescueTest, SteepReturnTaperIsFlownAsItFalls)
{
    // However steep the taper - a fast return, a short descent distance, a gentle carrot
    // acceleration - the commanded speed follows it down rather than slewing at nav_accel and
    // reaching the descent distance hot.
    struct { uint16_t speedCmS; uint16_t descentDistM; uint16_t navAccelCmSS; } configs[] = {
        { 750, 7, 250 },
        { 1500, 20, 250 },
        { 1500, 5, 250 },
        { 750, 20, 20 },
    };
    for (const auto &c : configs) {
        SetUp();
        gpsRescueConfigMutable()->groundSpeedCmS = c.speedCmS;
        gpsRescueConfigMutable()->descentDistanceM = c.descentDistM;
        autopilotConfigMutable()->navAccel = c.navAccelCmSS;
        const float speedMps = c.speedCmS * 0.01f;
        const float slowdownM = 2.0f * c.descentDistM;
        const auto taperMps = [&](float distM) {
            return speedMps * fminf(fmaxf((distM - 1.0f) / (slowdownM - 1.0f), 0.0f), 1.0f);
        };

        // Already at the return speed well outside the taper, heading home (south).
        float northM = slowdownM + 20.0f;
        g_stubEstimate.position.v[ENU_N] = northM * 100.0f;
        g_stubEstimate.velocity.v[ENU_N] = -speedMps * 100.0f;
        attitude.values.yaw = 1800;
        ASSERT_TRUE(flightPlanNavStageRescuePlan());
        flightPlanNavEngage();
        triggerReached();
        ASSERT_EQ(flightPlanNavGetCurrentIndex(), 1);

        float velMps = -speedMps;
        for (int i = 0; i < 2000 && flightPlanNavGetCurrentIndex() == 1; i++) {
            g_stubMicros += 20'000;
            flightPlanNavUpdate(g_stubMicros);
            if (flightPlanNavGetCurrentIndex() != 1) {
                break;
            }
            EXPECT_LE(lastFfSpeedMps(), fmaxf(taperMps(northM), 0.0f) + 0.05f) << "at " << northM << " m";
            velMps += (g_lastFfEfMps.y - velMps) * 0.02f / 0.3f;   // craft following with a lag
            northM += velMps * 0.02f;
            g_stubEstimate.position.v[ENU_N] = northM * 100.0f;
            g_stubEstimate.velocity.v[ENU_N] = velMps * 100.0f;
        }
        ASSERT_EQ(flightPlanNavGetCurrentIndex(), 2);
        EXPECT_NEAR(lastFfSpeedMps(), 0.0f, 0.001f);   // the landing leg's own command takes over
        flightPlanNavDisengage();
    }
}

// --- Fallback emergency descent (switch rescue: no fix, or plan aborted) ---

TEST_F(FlightPlanRescueTest, RescueDescentDrivesLandingAndDisarms)
{
    g_stubMicros = 1'000'000;
    EXPECT_FALSE(flightPlanNavIsRescueDescentActive());

    // Runs independently of the executor: alt-hold owns the throttle, this only
    // detects touchdown. Descent established.
    g_stubEstimate.velocity.v[ENU_U] = -40.0f; // cm/s
    flightPlanNavRescueDescent(true, g_stubMicros);
    EXPECT_TRUE(flightPlanNavIsRescueDescentActive());
    EXPECT_FALSE(flightPlanNavIsActive());
    EXPECT_EQ(g_disarmCalls, 0);

    // Touchdown: descent stops and the vehicle is quiet, then the timer expires.
    g_stubEstimate.velocity.v[ENU_U] = 0.0f;
    g_stubMicros += 100'000;
    flightPlanNavRescueDescent(true, g_stubMicros);
    EXPECT_EQ(g_disarmCalls, 0);

    g_stubMicros += 1'100'000; // past landingDetectionTime (1 s)
    flightPlanNavRescueDescent(true, g_stubMicros);
    EXPECT_EQ(g_disarmCalls, 1);
    EXPECT_EQ(g_lastDisarmReason, DISARM_REASON_LANDING);
}

TEST_F(FlightPlanRescueTest, RescueDescentReleaseClears)
{
    flightPlanNavRescueDescent(true, g_stubMicros);
    EXPECT_TRUE(flightPlanNavIsRescueDescentActive());
    flightPlanNavRescueDescent(false, g_stubMicros);
    EXPECT_FALSE(flightPlanNavIsRescueDescentActive());
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
