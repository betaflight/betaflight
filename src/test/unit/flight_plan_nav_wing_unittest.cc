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

    #include "flight/autopilot.h"
    #include "flight/flight_plan_nav.h"
    #include "flight/gps_rescue.h"
    #include "flight/imu.h"
    #include "flight/landing_wing.h"
    #include "flight/position_estimator.h"
    #include "flight/position_nav.h"

    #include "io/gps.h"

    #include "pg/autopilot.h"
    #include "pg/autopilot_wing.h"
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

namespace {

// The nav command as the flight plan last left it.
struct NavCommand {
    bool active;
    vector3_t targetEfM;
    float acceptanceRadiusM;
    uint8_t track;
    vector2_t trackStartEfM;
    float loiterRadiusM;
    int8_t loiterDirection;
    float vertRateMps;
    float vertStartAltM;
};

NavCommand g_nav;
int g_setTargetCalls;
int g_headingOverrideCalls;

timeUs_t g_micros;
positionEstimate3d_t g_estimate;
bool g_validXY;
gpsLocation_t g_origin;
float g_commandedAltCm;

float g_l1M;
float g_minTurnRadiusM;
bool g_limitsSet;
autopilotWingLimits_t g_limits;

int g_disarmCalls;

int g_landingStarts;
landingWingSite_t g_landingSite;
bool g_landingActive;
bool g_landed;

float g_maxAltitudeCm;
bool g_headingValid;
bool g_emergencyDescent;
float g_emergencyDescentRateCmS;

const float METRES_PER_LAT_UNIT = 111319.49f / 1.0e7f;

int32_t latUnits(float metres)
{
    return lrintf(metres / METRES_PER_LAT_UNIT);
}

} // namespace

extern "C" {

void positionNavSetTargetEf(const vector3_t *targetPosEfM, float, float acceptanceRadiusM, float, bool,
                            positionNavReachedCallbackFn, void *)
{
    g_nav.active = true;
    g_nav.targetEfM = *targetPosEfM;
    g_nav.acceptanceRadiusM = acceptanceRadiusM;
    g_nav.track = NAV_TRACK_POINT;
    g_setTargetCalls++;
}

void positionNavSetTrackLine(const vector2_t *startEfM)
{
    g_nav.track = NAV_TRACK_LINE;
    g_nav.trackStartEfM = *startEfM;
}

void positionNavSetTrackLoiter(float radiusM, int8_t direction)
{
    g_nav.track = NAV_TRACK_LOITER;
    g_nav.loiterRadiusM = radiusM;
    g_nav.loiterDirection = direction;
}

void positionNavSetVerticalProfile(float rateMps, float startAltM)
{
    g_nav.vertRateMps = rateMps;
    g_nav.vertStartAltM = startAltM;
}

void positionNavClearTarget(void) { g_nav.active = false; }
bool positionNavHasActiveTarget(void) { return g_nav.active; }
float positionNavGetTargetAltitudeCm(void) { return g_commandedAltCm; }

const positionNavCommand_t *positionNavGetActiveCommand(void)
{
    static positionNavCommand_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.active = g_nav.active;
    cmd.targetPosEfM = g_nav.targetEfM;
    cmd.includeAltitude = true;
    return &cmd;
}

void positionNavMoveTargetEf(const vector3_t *) {}
void positionNavLowerTargetAltitude(float) {}
void positionNavStartAfresh(void) {}
void positionNavSetAutoClearOnReach(bool) {}
void positionNavSetAccelLimits(float, float) {}
void positionNavSetApproachBrake(float, float) {}
float positionNavApproachSpeedMps(float cruiseSpeedMps, float, float, float) { return cruiseSpeedMps; }
void positionNavSetVelocityFeedforward(const vector2_t *) {}
void positionNavSetAltitudeArrivalRequired(bool) {}
void positionNavSetSettleTimeout(float) {}
void positionNavSetMaxAngle(float) {}
vector3_t positionNavGetTargetVelocityCmS(void) { return (vector3_t){{ 0.0f, 0.0f, 0.0f }}; }

bool positionEstimatorGetGpsOrigin(gpsLocation_t *out)
{
    *out = g_origin;
    return true;
}

// the estimator's altitude frame is the GPS one
float positionEstimatorGetAltitudeCm(void) { return gpsSol.llh.altCm - g_origin.altCm; }
const positionEstimate3d_t *positionEstimatorGetEstimate(void) { return &g_estimate; }
bool positionEstimatorIsValidXY(void) { return g_validXY; }

float altHoldGetClimbRateCmS(void) { return 200.0f; }

void altHoldSetEmergencyDescent(bool active, float rateCmS)
{
    g_emergencyDescent = active;
    g_emergencyDescentRateCmS = rateCmS;
}

float gpsRescueGetMaxAltitudeCm(void) { return g_maxAltitudeCm; }
bool imuIsHeadingValid(void) { return g_headingValid; }
void autopilotHeadingRecovery(bool, float) {}

void disarm(flightLogDisarmReason_e) { g_disarmCalls++; }
bool isBelowLandingAltitude(void) { return false; }

void autopilotSetYawRateLimit(float) {}
void autopilotForceLevelPark(bool) {}
void autopilotSetNavHeadingOverride(bool, float) { g_headingOverrideCalls++; }
vector2_t autopilotGetPositionErrorCm(void) { return (vector2_t){{ 0.0f, 0.0f }}; }

float autopilotWingL1DistanceM(void) { return g_l1M; }
float autopilotWingMinTurnRadiusM(void) { return g_minTurnRadiusM; }
float autopilotWingTurnDistanceM(float turnDeg) { return g_l1M * fminf(fabsf(turnDeg) / 90.0f, 1.0f); }
float autopilotWingLoiterRadiusM(void) { return fmaxf(autopilotWingConfig()->loiterRadius, g_minTurnRadiusM); }

void autopilotWingDefaultLimits(autopilotWingLimits_t *limits)
{
    memset(limits, 0, sizeof(*limits));
    limits->bankLimitDeg = 35.0f;
}

void autopilotWingSetLimits(autopilotWingLimitsOwner_e, const autopilotWingLimits_t *limits)
{
    g_limits = *limits;
    g_limitsSet = true;
}

void autopilotWingClearLimits(autopilotWingLimitsOwner_e) { g_limitsSet = false; }

void landingWingStart(const landingWingSite_t *site, timeUs_t)
{
    g_landingSite = *site;
    g_landingStarts++;
    g_landingActive = true;
}

bool landingWingUpdate(timeUs_t) { return g_landed; }
float g_homeGroundM;
float landingWingHomeGroundM(void) { return g_homeGroundM; }
float landingWingHomeGroundRadiusM(void) { return gpsRescueConfig()->minStartDistM; }
bool g_rangefinderHealthy;
bool rangefinderIsHealthy(void) { return g_rangefinderHealthy; }
float getAltitudeCmControl(void) { return g_estimate.position.v[ENU_U]; }
void landingWingStop(void) { g_landingActive = false; }

void GPS_distance2d(const gpsLocation_t *from, const gpsLocation_t *to, vector2_t *distance)
{
    distance->x = (to->lon - from->lon) * METRES_PER_LAT_UNIT * 100.0f;
    distance->y = (to->lat - from->lat) * METRES_PER_LAT_UNIT * 100.0f;
}

timeUs_t micros(void) { return g_micros; }
uint32_t millis(void) { return g_micros / 1000; }

} // extern "C"

static const float ALT_M = 40.0f;      // waypoint altitude above the origin

class FlightPlanNavWingTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        memset(&g_nav, 0, sizeof(g_nav));
        g_setTargetCalls = 0;
        g_headingOverrideCalls = 0;
        g_micros = 1000000;
        memset(&g_estimate, 0, sizeof(g_estimate));
        g_validXY = true;
        g_origin = (gpsLocation_t){ .lat = 0, .lon = 0, .altCm = 10000 };
        g_l1M = 40.0f;
        g_minTurnRadiusM = 43.0f;
        g_limitsSet = false;
        g_disarmCalls = 0;
        g_landingStarts = 0;
        memset(&g_landingSite, 0, sizeof(g_landingSite));
        g_landingActive = false;
        g_landed = false;
        g_homeGroundM = 0.0f;
        g_rangefinderHealthy = false;
        g_maxAltitudeCm = 0.0f;
        g_headingValid = false;     // a wing flies by its course
        g_emergencyDescent = false;
        memset(&attitude, 0, sizeof(attitude));
        stateFlags = 0;
        GPS_distanceToHome = 0;
        memset(&GPS_home_llh, 0, sizeof(GPS_home_llh));
        GPS_home_llh.altCm = 10000;
        memset(&gpsSol, 0, sizeof(gpsSol));
        gpsSol.llh.altCm = 10000;

        memset(flightPlanConfigMutable(), 0, sizeof(flightPlanConfig_t));

        autopilotConfig_t *cfg = autopilotConfigMutable();
        memset(cfg, 0, sizeof(*cfg));
        cfg->waypointArrivalRadius = 1000;

        autopilotWingConfig_t *wingCfg = autopilotWingConfigMutable();
        memset(wingCfg, 0, sizeof(*wingCfg));
        wingCfg->cruiseSpeed = 150;
        wingCfg->loiterRadius = 60;
        wingCfg->loiterDirection = WING_LOITER_RIGHT;
        wingCfg->maxClimbRate = 30;

        gpsRescueConfig_t *rescueCfg = gpsRescueConfigMutable();
        memset(rescueCfg, 0, sizeof(*rescueCfg));
        rescueCfg->minStartDistM = 100;
        rescueCfg->altitudeMode = GPS_RESCUE_ALT_MODE_FIXED;
        rescueCfg->initialClimbM = 10;
        rescueCfg->ascendRate = 300;
        rescueCfg->returnAltitudeM = 50;
        rescueCfg->descendRate = 200;

        flyAt(0.0f, 0.0f, ALT_M);
        setVelocity(0.0f, 15.0f);
        flightPlanNavInit();
    }

    void addWaypoint(float eastM, float northM, uint8_t type, uint16_t duration = 0,
                     uint8_t pattern = WAYPOINT_PATTERN_NONE, uint8_t yaw = WAYPOINT_YAW_DEFAULT, float altM = ALT_M)
    {
        flightPlanConfig_t *plan = flightPlanConfigMutable();
        waypoint_t *wp = &plan->waypoints[plan->waypointCount++];
        wp->latitude = latUnits(northM);
        wp->longitude = latUnits(eastM);
        wp->altitude = 10000 + lrintf(altM * 100.0f);
        wp->type = type;
        wp->duration = duration;
        wp->pattern = pattern;
        wp->yawBehaviour = yaw;
    }

    void addModifier(uint8_t type, uint16_t duration)
    {
        flightPlanConfig_t *plan = flightPlanConfigMutable();
        waypoint_t *wp = &plan->waypoints[plan->waypointCount++];
        memset(wp, 0, sizeof(*wp));
        wp->type = type;
        wp->duration = duration;
        wp->speed = 30;
    }

    static void flyAt(float eastM, float northM, float upM = ALT_M)
    {
        g_estimate.position.v[ENU_E] = eastM * 100.0f;
        g_estimate.position.v[ENU_N] = northM * 100.0f;
        g_estimate.position.v[ENU_U] = upM * 100.0f;
        g_commandedAltCm = upM * 100.0f;
    }

    static void setVelocity(float eastMps, float northMps)
    {
        g_estimate.velocity.v[ENU_E] = eastMps * 100.0f;
        g_estimate.velocity.v[ENU_N] = northMps * 100.0f;
    }

    static void tick(float seconds = 0.1f)
    {
        g_micros += lrintf(seconds * 1e6f);
        flightPlanNavUpdate(g_micros);
    }

    // Fly to a point and run the executor there.
    static void flyTo(float eastM, float northM, float upM = ALT_M)
    {
        flyAt(eastM, northM, upM);
        tick();
    }

    static float targetEastM(void) { return g_nav.targetEfM.v[ENU_E]; }
    static float targetNorthM(void) { return g_nav.targetEfM.v[ENU_N]; }
};

// Legs

TEST_F(FlightPlanNavWingTest, TheFirstLegIsALineFromTheAircraft)
{
    flyAt(-30.0f, -20.0f);
    addWaypoint(0.0f, 400.0f, WAYPOINT_TYPE_FLYOVER);
    flightPlanNavEngage();
    EXPECT_EQ(FP_NAV_TARGETING, flightPlanNavGetState());
    EXPECT_EQ(NAV_TRACK_LINE, g_nav.track);
    EXPECT_NEAR(-30.0f, g_nav.trackStartEfM.x, 0.1f);
    EXPECT_NEAR(-20.0f, g_nav.trackStartEfM.y, 0.1f);
    EXPECT_NEAR(400.0f, targetNorthM(), 0.1f);
    EXPECT_NEAR(ALT_M, g_nav.targetEfM.v[ENU_U], 0.1f);
    EXPECT_LT(g_nav.acceptanceRadiusM, 0.0f);    // positionNav never completes it
}

TEST_F(FlightPlanNavWingTest, AFlybyTurnsOntoTheNextLegItsTurnDistanceEarly)
{
    addWaypoint(0.0f, 400.0f, WAYPOINT_TYPE_FLYBY);
    addWaypoint(400.0f, 400.0f, WAYPOINT_TYPE_FLYOVER);     // a 90 degree turn: the full look-ahead
    flightPlanNavEngage();

    flyTo(0.0f, 359.0f);
    EXPECT_EQ(0, flightPlanNavGetCurrentIndex());
    flyTo(0.0f, 361.0f);
    EXPECT_EQ(1, flightPlanNavGetCurrentIndex());

    // and the next line starts on the waypoint, not where the turn began
    EXPECT_EQ(NAV_TRACK_LINE, g_nav.track);
    EXPECT_NEAR(0.0f, g_nav.trackStartEfM.x, 0.1f);
    EXPECT_NEAR(400.0f, g_nav.trackStartEfM.y, 0.1f);
}

TEST_F(FlightPlanNavWingTest, AShallowFlybyTurnsLater)
{
    addWaypoint(0.0f, 400.0f, WAYPOINT_TYPE_FLYBY);
    addWaypoint(400.0f, 800.0f, WAYPOINT_TYPE_FLYOVER);     // 45 degrees: half the look-ahead
    flightPlanNavEngage();

    flyTo(0.0f, 379.0f);
    EXPECT_EQ(0, flightPlanNavGetCurrentIndex());
    flyTo(0.0f, 381.0f);
    EXPECT_EQ(1, flightPlanNavGetCurrentIndex());
}

TEST_F(FlightPlanNavWingTest, AFlybyStraightThroughGoesOnAtTheArrivalRadius)
{
    addWaypoint(0.0f, 400.0f, WAYPOINT_TYPE_FLYBY);
    addWaypoint(0.0f, 800.0f, WAYPOINT_TYPE_FLYOVER);
    flightPlanNavEngage();

    flyTo(0.0f, 385.0f);
    EXPECT_EQ(0, flightPlanNavGetCurrentIndex());
    flyTo(0.0f, 391.0f);
    EXPECT_EQ(1, flightPlanNavGetCurrentIndex());
}

TEST_F(FlightPlanNavWingTest, AFlyoverIsReachedAbeam)
{
    addWaypoint(0.0f, 400.0f, WAYPOINT_TYPE_FLYOVER);
    addWaypoint(400.0f, 400.0f, WAYPOINT_TYPE_FLYOVER);
    flightPlanNavEngage();

    flyTo(30.0f, 399.0f);
    EXPECT_EQ(0, flightPlanNavGetCurrentIndex());
    flyTo(30.0f, 400.5f);
    EXPECT_EQ(1, flightPlanNavGetCurrentIndex());
}

TEST_F(FlightPlanNavWingTest, AFlyoverIsReachedInsideTheArrivalRadius)
{
    addWaypoint(0.0f, 400.0f, WAYPOINT_TYPE_FLYOVER);
    addWaypoint(400.0f, 400.0f, WAYPOINT_TYPE_FLYOVER);
    flightPlanNavEngage();

    flyTo(-8.0f, 395.0f);
    EXPECT_EQ(1, flightPlanNavGetCurrentIndex());
}

TEST_F(FlightPlanNavWingTest, ItNeverWaitsForTheNoseAndNeverStealsIt)
{
    flyAt(0.0f, 0.0f);
    attitude.values.yaw = 1800;     // nose south, course north
    addWaypoint(0.0f, 400.0f, WAYPOINT_TYPE_FLYOVER, 0, WAYPOINT_PATTERN_NONE, WAYPOINT_YAW_FACE_TARGET);
    addWaypoint(400.0f, 400.0f, WAYPOINT_TYPE_FLYOVER, 0, WAYPOINT_PATTERN_NONE, WAYPOINT_YAW_HOLD);
    flightPlanNavEngage();
    EXPECT_EQ(NAV_TRACK_LINE, g_nav.track);
    EXPECT_NEAR(400.0f, targetNorthM(), 0.1f);

    for (int i = 0; i < 100; i++) {
        flyTo(0.0f, 4.0f * i);
    }
    flyTo(0.0f, 400.5f);
    EXPECT_EQ(1, flightPlanNavGetCurrentIndex());
    EXPECT_EQ(FP_NAV_TARGETING, flightPlanNavGetState());       // no heading fault, however far apart they are
    EXPECT_EQ(FP_ABORT_NONE, flightPlanNavGetAbortReason());
    int overrides = g_headingOverrideCalls;
    flyTo(100.0f, 400.0f);
    EXPECT_EQ(overrides, g_headingOverrideCalls);
}

// Holds

TEST_F(FlightPlanNavWingTest, AHoldLoitersAndIsReachedOnceOnTheCircleAtHeight)
{
    addWaypoint(0.0f, 400.0f, WAYPOINT_TYPE_HOLD, 300);
    flightPlanNavEngage();
    EXPECT_EQ(NAV_TRACK_LOITER, g_nav.track);
    EXPECT_FLOAT_EQ(60.0f, g_nav.loiterRadiusM);
    EXPECT_EQ(1, g_nav.loiterDirection);

    flyTo(0.0f, 400.0f - 105.0f);                   // R + L1 = 100 m
    EXPECT_EQ(FP_NAV_TARGETING, flightPlanNavGetState());
    flyTo(0.0f, 400.0f - 95.0f, ALT_M - 4.0f);      // close enough but still climbing
    EXPECT_EQ(FP_NAV_TARGETING, flightPlanNavGetState());
    flyTo(0.0f, 400.0f - 95.0f, ALT_M - 2.0f);
    EXPECT_EQ(FP_NAV_HOLDING, flightPlanNavGetState());
    EXPECT_EQ(NAV_TRACK_LOITER, g_nav.track);
    EXPECT_NEAR(400.0f, targetNorthM(), 0.1f);
}

TEST_F(FlightPlanNavWingTest, AHoldLoitersForItsDurationThenGoesOn)
{
    addWaypoint(0.0f, 400.0f, WAYPOINT_TYPE_HOLD, 300);
    addWaypoint(400.0f, 400.0f, WAYPOINT_TYPE_FLYOVER);
    flightPlanNavEngage();
    flyTo(0.0f, 340.0f);
    ASSERT_EQ(FP_NAV_HOLDING, flightPlanNavGetState());
    for (int i = 0; i < 295; i++) {
        tick();
    }
    EXPECT_EQ(FP_NAV_HOLDING, flightPlanNavGetState());
    for (int i = 0; i < 10; i++) {
        tick();
    }
    EXPECT_EQ(1, flightPlanNavGetCurrentIndex());
    EXPECT_EQ(NAV_TRACK_LINE, g_nav.track);
    EXPECT_NEAR(400.0f, g_nav.trackStartEfM.y, 0.1f);       // from the hold's centre
}

TEST_F(FlightPlanNavWingTest, PatternsFlyAsALoiterNoTighterThanTheAircraftCanTurn)
{
    autopilotWingConfigMutable()->loiterRadius = 20;
    autopilotWingConfigMutable()->loiterDirection = WING_LOITER_LEFT;
    addWaypoint(0.0f, 400.0f, WAYPOINT_TYPE_HOLD, 300, WAYPOINT_PATTERN_ORBIT);
    addWaypoint(0.0f, 400.0f, WAYPOINT_TYPE_HOLD, 300, WAYPOINT_PATTERN_FIGURE8);
    flightPlanNavEngage();
    EXPECT_EQ(NAV_TRACK_LOITER, g_nav.track);
    EXPECT_FLOAT_EQ(g_minTurnRadiusM, g_nav.loiterRadiusM);
    EXPECT_EQ(-1, g_nav.loiterDirection);

    flyTo(0.0f, 400.0f);
    for (int i = 0; i < 305; i++) {
        tick();
    }
    ASSERT_EQ(1, flightPlanNavGetCurrentIndex());
    EXPECT_EQ(NAV_TRACK_LOITER, g_nav.track);
    EXPECT_FLOAT_EQ(g_minTurnRadiusM, g_nav.loiterRadiusM);
}

TEST_F(FlightPlanNavWingTest, OrbitPeriodIsOneLoiterAtTheCruiseSpeed)
{
    EXPECT_EQ(lrintf(10.0f * 2.0f * M_PIf * 60.0f / 15.0f), flightPlanNavOrbitPeriodDs(500));
    g_minTurnRadiusM = 80.0f;
    EXPECT_EQ(lrintf(10.0f * 2.0f * M_PIf * 80.0f / 15.0f), flightPlanNavOrbitPeriodDs(0));
}

// Takeoff

TEST_F(FlightPlanNavWingTest, ATakeoffClimbsStraightOutOnItsCourse)
{
    flyAt(10.0f, 20.0f, 5.0f);
    setVelocity(15.0f, 0.0f);               // flying east
    attitude.values.yaw = 0;                // nose north: the course decides
    addWaypoint(500.0f, 500.0f, WAYPOINT_TYPE_TAKEOFF, 100, WAYPOINT_PATTERN_NONE, WAYPOINT_YAW_DEFAULT, 30.0f);
    addWaypoint(0.0f, 400.0f, WAYPOINT_TYPE_FLYOVER);
    flightPlanNavEngage();

    EXPECT_EQ(NAV_TRACK_LINE, g_nav.track);
    EXPECT_NEAR(10.0f, g_nav.trackStartEfM.x, 0.1f);
    EXPECT_NEAR(20.0f, g_nav.trackStartEfM.y, 0.1f);
    EXPECT_NEAR(2010.0f, targetEastM(), 0.5f);
    EXPECT_NEAR(20.0f, targetNorthM(), 0.5f);
    EXPECT_NEAR(30.0f, g_nav.targetEfM.v[ENU_U], 0.1f);
    EXPECT_NEAR(2.0f, g_nav.vertRateMps, 0.01f);
    ASSERT_TRUE(g_limitsSet);
    EXPECT_FLOAT_EQ(15.0f, g_limits.bankLimitDeg);

    flyTo(200.0f, 20.0f, 26.0f);
    EXPECT_EQ(FP_NAV_TARGETING, flightPlanNavGetState());
    flyTo(300.0f, 20.0f, 28.0f);
    EXPECT_EQ(FP_NAV_HOLDING, flightPlanNavGetState());

    // its duration loiters where the climb ended, at the full bank
    EXPECT_EQ(NAV_TRACK_LOITER, g_nav.track);
    EXPECT_NEAR(300.0f, targetEastM(), 0.1f);
    EXPECT_NEAR(30.0f, g_nav.targetEfM.v[ENU_U], 0.1f);
    EXPECT_FALSE(g_limitsSet);
    for (int i = 0; i < 105; i++) {
        tick();
    }
    EXPECT_EQ(1, flightPlanNavGetCurrentIndex());
    EXPECT_NEAR(300.0f, g_nav.trackStartEfM.x, 0.1f);
}

TEST_F(FlightPlanNavWingTest, ASlowTakeoffClimbsOutAlongTheNose)
{
    setVelocity(0.0f, 2.0f);
    attitude.values.yaw = 900;              // nose east
    addWaypoint(0.0f, 0.0f, WAYPOINT_TYPE_TAKEOFF, 0, WAYPOINT_PATTERN_NONE, WAYPOINT_YAW_DEFAULT, 60.0f);
    flightPlanNavEngage();
    EXPECT_NEAR(2000.0f, targetEastM(), 1.0f);
    EXPECT_NEAR(0.0f, targetNorthM(), 1.0f);
}

// Modifiers

TEST_F(FlightPlanNavWingTest, ADelayHoldsAtTheWaypointUntilItsTime)
{
    addModifier(WAYPOINT_TYPE_DELAY, 300);
    addWaypoint(0.0f, 100.0f, WAYPOINT_TYPE_FLYOVER);
    addWaypoint(400.0f, 100.0f, WAYPOINT_TYPE_FLYOVER);
    flightPlanNavEngage();
    EXPECT_EQ(1, flightPlanNavGetCurrentIndex());
    EXPECT_EQ(NAV_TRACK_LINE, g_nav.track);

    for (int i = 0; i < 100; i++) {
        tick();
    }
    flyTo(0.0f, 101.0f);         // 10 s into a 30 s delay
    EXPECT_EQ(FP_NAV_HOLDING, flightPlanNavGetState());
    EXPECT_EQ(NAV_TRACK_LOITER, g_nav.track);
    EXPECT_NEAR(100.0f, targetNorthM(), 0.1f);
    for (int i = 0; i < 195; i++) {
        tick();
    }
    EXPECT_EQ(1, flightPlanNavGetCurrentIndex());
    for (int i = 0; i < 10; i++) {
        tick();
    }
    EXPECT_EQ(2, flightPlanNavGetCurrentIndex());
}

TEST_F(FlightPlanNavWingTest, AYawRateChangesNothing)
{
    addModifier(WAYPOINT_TYPE_YAW_RATE, 0);
    addWaypoint(0.0f, 400.0f, WAYPOINT_TYPE_FLYOVER);
    flightPlanNavEngage();
    EXPECT_EQ(1, flightPlanNavGetCurrentIndex());
    EXPECT_EQ(NAV_TRACK_LINE, g_nav.track);
    EXPECT_NEAR(400.0f, targetNorthM(), 0.1f);
}

// Land and completion

TEST_F(FlightPlanNavWingTest, ALandWaypointIsTurnedOntoAtTheHeightItArrivesAtThenLanded)
{
    addWaypoint(0.0f, 400.0f, WAYPOINT_TYPE_FLYOVER);
    addWaypoint(0.0f, 600.0f, WAYPOINT_TYPE_LAND, 0, WAYPOINT_PATTERN_NONE, WAYPOINT_YAW_DEFAULT, 0.0f);
    flightPlanNavEngage();
    flyTo(0.0f, 401.0f);
    ASSERT_EQ(1, flightPlanNavGetCurrentIndex());
    EXPECT_EQ(NAV_TRACK_LOITER, g_nav.track);
    EXPECT_NEAR(600.0f, targetNorthM(), 0.1f);
    EXPECT_NEAR(ALT_M, g_nav.targetEfM.v[ENU_U], 0.1f);
    EXPECT_EQ(0, g_landingStarts);

    flyTo(0.0f, 550.0f);
    EXPECT_EQ(FP_NAV_LANDING, flightPlanNavGetState());
    EXPECT_EQ(1, g_landingStarts);
    for (int i = 0; i < 100; i++) {
        tick();
    }
    EXPECT_EQ(FP_NAV_LANDING, flightPlanNavGetState());
    EXPECT_EQ(0, g_disarmCalls);

    g_landed = true;
    tick();
    EXPECT_EQ(FP_NAV_COMPLETE, flightPlanNavGetState());
    EXPECT_EQ(1, g_disarmCalls);
    EXPECT_FALSE(g_landingActive);
}

TEST_F(FlightPlanNavWingTest, AMissionLandsOnItsLandWaypointsGroundOnTheCourseItFliesInOn)
{
    addWaypoint(0.0f, 400.0f, WAYPOINT_TYPE_FLYOVER);
    addWaypoint(300.0f, 400.0f, WAYPOINT_TYPE_LAND, 0, WAYPOINT_PATTERN_NONE, WAYPOINT_YAW_DEFAULT, 12.0f);
    flightPlanNavEngage();
    flyTo(0.0f, 401.0f);
    flyTo(250.0f, 400.0f);
    ASSERT_EQ(FP_NAV_LANDING, flightPlanNavGetState());
    EXPECT_NEAR(300.0f, g_landingSite.touchdownEnuM.v[ENU_E], 0.2f);
    EXPECT_NEAR(400.0f, g_landingSite.touchdownEnuM.v[ENU_N], 0.2f);
    EXPECT_NEAR(12.0f, g_landingSite.touchdownEnuM.v[ENU_U], 0.1f);  // the waypoint's altitude is the ground's
    EXPECT_NEAR(90.0f, g_landingSite.headingDeg, 0.5f);

    // flown to first, it has no course in: the landing chooses one
    SetUp();
    addWaypoint(300.0f, 400.0f, WAYPOINT_TYPE_LAND, 0, WAYPOINT_PATTERN_NONE, WAYPOINT_YAW_DEFAULT, 12.0f);
    flightPlanNavEngage();
    flyTo(250.0f, 400.0f);
    ASSERT_EQ(FP_NAV_LANDING, flightPlanNavGetState());
    EXPECT_LT(g_landingSite.headingDeg, 0.0f);
}

TEST_F(FlightPlanNavWingTest, ALandWaypointsDurationIsLoiteredBeforeLanding)
{
    addWaypoint(0.0f, 400.0f, WAYPOINT_TYPE_LAND, 300, WAYPOINT_PATTERN_NONE, WAYPOINT_YAW_DEFAULT, 0.0f);
    flightPlanNavEngage();
    flyTo(0.0f, 350.0f);
    EXPECT_EQ(FP_NAV_HOLDING, flightPlanNavGetState());
    for (int i = 0; i < 299; i++) {
        tick();
    }
    EXPECT_EQ(FP_NAV_HOLDING, flightPlanNavGetState());
    EXPECT_EQ(0, g_landingStarts);
    tick();
    EXPECT_EQ(FP_NAV_LANDING, flightPlanNavGetState());
    EXPECT_EQ(1, g_landingStarts);
}

TEST_F(FlightPlanNavWingTest, ALandingGivesUpAPositionLostForLongerThanALegWould)
{
    addWaypoint(0.0f, 400.0f, WAYPOINT_TYPE_LAND, 0, WAYPOINT_PATTERN_NONE, WAYPOINT_YAW_DEFAULT, 0.0f);
    flightPlanNavEngage();
    flyTo(0.0f, 350.0f);
    ASSERT_EQ(FP_NAV_LANDING, flightPlanNavGetState());
    g_validXY = false;
    for (int i = 0; i < 295; i++) {
        tick();
    }
    EXPECT_EQ(FP_NAV_LANDING, flightPlanNavGetState());
    for (int i = 0; i < 10; i++) {
        tick();
    }
    EXPECT_EQ(FP_NAV_ABORTED, flightPlanNavGetState());
    EXPECT_EQ(FP_ABORT_ESTIMATOR, flightPlanNavGetAbortReason());
    EXPECT_FALSE(g_landingActive);
    EXPECT_EQ(0, g_disarmCalls);
}

TEST_F(FlightPlanNavWingTest, TheEndOfThePlanLoitersAboutTheLastWaypoint)
{
    addWaypoint(0.0f, 400.0f, WAYPOINT_TYPE_FLYOVER);
    flightPlanNavEngage();
    flyTo(5.0f, 401.0f);
    EXPECT_EQ(FP_NAV_COMPLETE, flightPlanNavGetState());
    EXPECT_TRUE(g_nav.active);
    EXPECT_EQ(NAV_TRACK_LOITER, g_nav.track);
    EXPECT_NEAR(0.0f, targetEastM(), 0.1f);
    EXPECT_NEAR(400.0f, targetNorthM(), 0.1f);
    EXPECT_NEAR(ALT_M, g_nav.targetEfM.v[ENU_U], 0.1f);
}

TEST_F(FlightPlanNavWingTest, AnEmptyPlanLoitersWhereItIs)
{
    flightPlanNavEngage();
    EXPECT_EQ(FP_NAV_COMPLETE, flightPlanNavGetState());
    EXPECT_FALSE(g_nav.active);
}

// Sanity checks

TEST_F(FlightPlanNavWingTest, ALegThatMakesNoProgressForAMinuteIsStalled)
{
    addWaypoint(0.0f, 2000.0f, WAYPOINT_TYPE_FLYOVER);
    flightPlanNavEngage();
    flyTo(0.0f, 100.0f);
    for (int i = 0; i < 590; i++) {
        tick();
    }
    EXPECT_EQ(FP_NAV_TARGETING, flightPlanNavGetState());
    for (int i = 0; i < 20; i++) {
        tick();
    }
    EXPECT_EQ(FP_NAV_ABORTED, flightPlanNavGetState());
    EXPECT_EQ(FP_ABORT_STALLED, flightPlanNavGetAbortReason());
}

TEST_F(FlightPlanNavWingTest, TheFlyawayMarginAllowsForTurningBack)
{
    // 2 Rmin + L1 + 2 s at 15 m/s: 156 m
    addWaypoint(0.0f, 400.0f, WAYPOINT_TYPE_FLYOVER);
    flightPlanNavEngage();
    flyTo(0.0f, 0.0f);
    flyTo(0.0f, -150.0f);
    EXPECT_EQ(FP_NAV_TARGETING, flightPlanNavGetState());
    flyTo(0.0f, -160.0f);
    EXPECT_EQ(FP_ABORT_FLYAWAY, flightPlanNavGetAbortReason());
}

TEST_F(FlightPlanNavWingTest, TheFlyawayMarginHasAFloorAndACeiling)
{
    setVelocity(0.0f, 0.0f);
    g_minTurnRadiusM = 0.0f;
    g_l1M = 10.0f;
    addWaypoint(0.0f, 100.0f, WAYPOINT_TYPE_FLYOVER);
    flightPlanNavEngage();
    flyTo(0.0f, 0.0f);
    flyTo(0.0f, -55.0f);
    EXPECT_EQ(FP_NAV_TARGETING, flightPlanNavGetState());
    flyTo(0.0f, -65.0f);
    EXPECT_EQ(FP_ABORT_FLYAWAY, flightPlanNavGetAbortReason());

    SetUp();
    g_minTurnRadiusM = 500.0f;
    addWaypoint(0.0f, 100.0f, WAYPOINT_TYPE_FLYOVER);
    flightPlanNavEngage();
    flyTo(0.0f, 0.0f);
    flyTo(0.0f, -295.0f);
    EXPECT_EQ(FP_NAV_TARGETING, flightPlanNavGetState());
    flyTo(0.0f, -305.0f);
    EXPECT_EQ(FP_ABORT_FLYAWAY, flightPlanNavGetAbortReason());
}

TEST_F(FlightPlanNavWingTest, APositionLossIsRiddenOutForThirtySeconds)
{
    addWaypoint(0.0f, 400.0f, WAYPOINT_TYPE_HOLD, 100);
    addWaypoint(400.0f, 400.0f, WAYPOINT_TYPE_FLYOVER);
    flightPlanNavEngage();
    flyTo(0.0f, 350.0f);
    ASSERT_EQ(FP_NAV_HOLDING, flightPlanNavGetState());
    for (int i = 0; i < 50; i++) {
        tick();
    }

    g_validXY = false;
    for (int i = 0; i < 290; i++) {
        tick();
    }
    EXPECT_EQ(FP_NAV_HOLDING, flightPlanNavGetState());     // its clock stopped too
    EXPECT_EQ(FP_ABORT_NONE, flightPlanNavGetAbortReason());
    EXPECT_TRUE(g_nav.active);

    g_validXY = true;
    for (int i = 0; i < 45; i++) {
        tick();
    }
    EXPECT_EQ(FP_NAV_HOLDING, flightPlanNavGetState());
    for (int i = 0; i < 10; i++) {
        tick();
    }
    EXPECT_EQ(1, flightPlanNavGetCurrentIndex());
}

TEST_F(FlightPlanNavWingTest, APositionLostForLongerAbortsTheMission)
{
    addWaypoint(0.0f, 400.0f, WAYPOINT_TYPE_FLYOVER);
    flightPlanNavEngage();
    flyTo(0.0f, 100.0f);
    g_validXY = false;
    for (int i = 0; i < 295; i++) {
        tick();
    }
    EXPECT_EQ(FP_NAV_TARGETING, flightPlanNavGetState());
    for (int i = 0; i < 10; i++) {
        tick();
    }
    EXPECT_EQ(FP_NAV_ABORTED, flightPlanNavGetState());
    EXPECT_EQ(FP_ABORT_ESTIMATOR, flightPlanNavGetAbortReason());
}

TEST_F(FlightPlanNavWingTest, AnAbortLoitersWhereTheAircraftIsAndClearsTheLimits)
{
    flyAt(0.0f, 0.0f, 10.0f);
    addWaypoint(0.0f, 0.0f, WAYPOINT_TYPE_TAKEOFF, 0, WAYPOINT_PATTERN_NONE, WAYPOINT_YAW_DEFAULT, 60.0f);
    flightPlanNavEngage();
    ASSERT_TRUE(g_limitsSet);
    g_validXY = false;
    for (int i = 0; i < 305; i++) {
        tick();
    }
    ASSERT_EQ(FP_NAV_ABORTED, flightPlanNavGetState());
    EXPECT_FALSE(g_nav.active);
    EXPECT_FALSE(g_limitsSet);
}

// Geofence

TEST_F(FlightPlanNavWingTest, AGeofenceReturnFliesHomeAndLandsThere)
{
    autopilotConfigMutable()->maxDistanceFromHomeM = 500;
    autopilotConfigMutable()->geofenceAction = AP_GEOFENCE_RTH;
    stateFlags = GPS_FIX_HOME;
    addWaypoint(0.0f, 2000.0f, WAYPOINT_TYPE_FLYOVER);
    flightPlanNavEngage();
    flyTo(0.0f, 490.0f);
    EXPECT_FALSE(flightPlanNavIsInjectedPlanActive());

    GPS_distanceToHome = 510;
    flyTo(0.0f, 510.0f);
    ASSERT_TRUE(flightPlanNavIsInjectedPlanActive());
    EXPECT_EQ(NAV_TRACK_LINE, g_nav.track);
    EXPECT_NEAR(0.0f, targetNorthM(), 0.1f);
    EXPECT_NEAR(510.0f, g_nav.trackStartEfM.y, 0.1f);
    EXPECT_NEAR(50.0f, g_nav.targetEfM.v[ENU_U], 0.1f);     // gps_rescue_return_alt above home

    flyTo(0.0f, -1.0f);
    EXPECT_EQ(NAV_TRACK_LOITER, g_nav.track);
    EXPECT_NEAR(0.0f, targetNorthM(), 0.1f);
    flyTo(0.0f, 20.0f);
    ASSERT_EQ(FP_NAV_LANDING, flightPlanNavGetState());
    EXPECT_NEAR(0.0f, g_landingSite.touchdownEnuM.v[ENU_N], 0.2f);
    EXPECT_FLOAT_EQ(0.0f, g_landingSite.touchdownEnuM.v[ENU_U]);    // on the arming point's ground
    EXPECT_LT(g_landingSite.headingDeg, 0.0f);                       // on the wing's own heading
}

TEST_F(FlightPlanNavWingTest, AGeofenceLandingLandsWhereTheFenceWasCrossed)
{
    autopilotConfigMutable()->maxDistanceFromHomeM = 500;
    autopilotConfigMutable()->geofenceAction = AP_GEOFENCE_LAND;
    stateFlags = GPS_FIX_HOME;
    addWaypoint(0.0f, 2000.0f, WAYPOINT_TYPE_FLYOVER);
    flightPlanNavEngage();

    GPS_distanceToHome = 510;
    gpsSol.llh.lat = latUnits(510.0f);
    gpsSol.llh.altCm = 10000 + lrintf(ALT_M * 100.0f);
    flyTo(0.0f, 510.0f);
    ASSERT_TRUE(flightPlanNavIsInjectedPlanActive());
    EXPECT_EQ(NAV_TRACK_LOITER, g_nav.track);
    EXPECT_NEAR(510.0f, targetNorthM(), 0.2f);
    EXPECT_NEAR(ALT_M, g_nav.targetEfM.v[ENU_U], 0.1f);
    tick();
    ASSERT_EQ(FP_NAV_LANDING, flightPlanNavGetState());
    EXPECT_NEAR(510.0f, g_landingSite.touchdownEnuM.v[ENU_N], 0.2f);
    EXPECT_FLOAT_EQ(0.0f, g_landingSite.touchdownEnuM.v[ENU_U]);
    EXPECT_LT(g_landingSite.headingDeg, 0.0f);
    EXPECT_EQ(0, g_disarmCalls);
}

// The fence crossed, 510 m from home.
static void landWhereTheFenceIsCrossed(void)
{
    autopilotConfigMutable()->maxDistanceFromHomeM = 500;
    autopilotConfigMutable()->geofenceAction = AP_GEOFENCE_LAND;
    stateFlags = GPS_FIX_HOME;
    flightPlanNavEngage();
    GPS_distanceToHome = 510;
    gpsSol.llh.lat = latUnits(510.0f);
    gpsSol.llh.altCm = 10000 + lrintf(ALT_M * 100.0f);
    g_estimate.position.v[ENU_N] = 510.0f * 100.0f;
    g_micros += 100000;
    flightPlanNavUpdate(g_micros);
    g_micros += 100000;
    flightPlanNavUpdate(g_micros);
}

TEST_F(FlightPlanNavWingTest, AwayFromHomeTheGroundIsUnknownAndNoApproachIsFlown)
{
    addWaypoint(0.0f, 2000.0f, WAYPOINT_TYPE_FLYOVER);
    landWhereTheFenceIsCrossed();
    ASSERT_EQ(FP_NAV_LANDING, flightPlanNavGetState());
    EXPECT_FALSE(g_landingSite.groundKnown);

    // a rangefinder senses the ground once low, but cannot place the pattern above it
    SetUp();
    g_rangefinderHealthy = true;
    addWaypoint(0.0f, 2000.0f, WAYPOINT_TYPE_FLYOVER);
    landWhereTheFenceIsCrossed();
    ASSERT_EQ(FP_NAV_LANDING, flightPlanNavGetState());
    EXPECT_FALSE(g_landingSite.groundKnown);
}

// Rescue

// The aircraft as the estimator and the GPS both see it.
static void craftAt(float eastM, float northM, float upM)
{
    g_estimate.position.v[ENU_E] = eastM * 100.0f;
    g_estimate.position.v[ENU_N] = northM * 100.0f;
    g_estimate.position.v[ENU_U] = upM * 100.0f;
    g_commandedAltCm = upM * 100.0f;
    gpsSol.llh.lat = latUnits(northM);
    gpsSol.llh.lon = latUnits(eastM);
    gpsSol.llh.altCm = 10000 + lrintf(upM * 100.0f);
    GPS_distanceToHome = lrintf(hypotf(eastM, northM));
}

static void startRescue(void)
{
    stateFlags = GPS_FIX | GPS_FIX_HOME;
    ASSERT_TRUE(flightPlanNavStageRescuePlan());
    flightPlanNavEngage();
    ASSERT_TRUE(flightPlanNavIsRescuePlanActive());
}

static void flyRescueTo(float eastM, float northM, float upM)
{
    craftAt(eastM, northM, upM);
    g_micros += 100000;
    flightPlanNavUpdate(g_micros);
}

TEST_F(FlightPlanNavWingTest, ARescueClimbsInALoiterWhereItIsThenFliesHome)
{
    gpsRescueConfigMutable()->ascendRate = 500;
    craftAt(0.0f, 400.0f, 20.0f);
    setVelocity(0.0f, 15.0f);
    startRescue();

    // a right hand loiter flying north from where it is, on its circle from the start
    EXPECT_EQ(NAV_TRACK_LOITER, g_nav.track);
    EXPECT_NEAR(60.0f, targetEastM(), 0.2f);
    EXPECT_NEAR(400.0f, targetNorthM(), 0.2f);
    EXPECT_NEAR(50.0f, g_nav.targetEfM.v[ENU_U], 0.1f);
    EXPECT_FLOAT_EQ(60.0f, g_nav.loiterRadiusM);
    EXPECT_EQ(1, g_nav.loiterDirection);
    EXPECT_NEAR(3.0f, g_nav.vertRateMps, 0.01f);           // no faster than the wing climbs

    flyRescueTo(0.0f, 400.0f, 45.0f);
    EXPECT_EQ(0, flightPlanNavGetCurrentIndex());
    flyRescueTo(0.0f, 400.0f, 49.0f);
    EXPECT_EQ(0, flightPlanNavGetCurrentIndex());           // at height, but heading away from home
    setVelocity(0.0f, -15.0f);
    flyRescueTo(120.0f, 400.0f, 49.0f);
    ASSERT_EQ(1, flightPlanNavGetCurrentIndex());

    // home on the line from the loiter's centre, which leaving the loiter that way it flies over
    EXPECT_EQ(NAV_TRACK_LINE, g_nav.track);
    EXPECT_NEAR(60.0f, g_nav.trackStartEfM.x, 0.2f);
    EXPECT_NEAR(400.0f, g_nav.trackStartEfM.y, 0.2f);
    EXPECT_NEAR(0.0f, targetEastM(), 0.2f);
    EXPECT_NEAR(0.0f, targetNorthM(), 0.2f);
    EXPECT_NEAR(50.0f, g_nav.targetEfM.v[ENU_U], 0.1f);

    // the loiter about home takes over as it is turned onto, at the height it arrives at
    flyRescueTo(0.0f, 101.0f, 50.0f);
    EXPECT_EQ(1, flightPlanNavGetCurrentIndex());
    flyRescueTo(0.0f, 99.0f, 50.0f);
    ASSERT_EQ(2, flightPlanNavGetCurrentIndex());
    EXPECT_EQ(NAV_TRACK_LOITER, g_nav.track);
    EXPECT_NEAR(0.0f, targetEastM(), 0.2f);
    EXPECT_NEAR(0.0f, targetNorthM(), 0.2f);
    EXPECT_NEAR(50.0f, g_nav.targetEfM.v[ENU_U], 0.1f);
    flyRescueTo(0.0f, 70.0f, 50.0f);
    ASSERT_EQ(FP_NAV_LANDING, flightPlanNavGetState());
    EXPECT_NEAR(0.0f, g_landingSite.touchdownEnuM.v[ENU_E], 0.2f);
    EXPECT_NEAR(0.0f, g_landingSite.touchdownEnuM.v[ENU_N], 0.2f);
    EXPECT_FLOAT_EQ(0.0f, g_landingSite.touchdownEnuM.v[ENU_U]);
    EXPECT_LT(g_landingSite.headingDeg, 0.0f);
    EXPECT_NEAR(2.0f, g_landingSite.sinkRateMps, 0.01f);             // gps_rescue_descend_rate
    EXPECT_EQ(0, g_disarmCalls);
}

TEST_F(FlightPlanNavWingTest, ARescueClimbsTheLoitersWayAndAboutItselfWhenSlow)
{
    autopilotWingConfigMutable()->loiterDirection = WING_LOITER_LEFT;
    craftAt(0.0f, 400.0f, 20.0f);
    setVelocity(15.0f, 0.0f);       // flying east, a left hand loiter's centre is to the north
    startRescue();
    EXPECT_NEAR(0.0f, targetEastM(), 0.2f);
    EXPECT_NEAR(460.0f, targetNorthM(), 0.2f);
    EXPECT_EQ(-1, g_nav.loiterDirection);

    flightPlanNavDisengage();
    setVelocity(0.0f, 2.0f);
    startRescue();
    EXPECT_NEAR(0.0f, targetEastM(), 0.2f);
    EXPECT_NEAR(400.0f, targetNorthM(), 0.2f);
}

TEST_F(FlightPlanNavWingTest, ARescueWithLittleToClimbSetsOffForHomeAtOnce)
{
    craftAt(0.0f, 400.0f, 46.0f);
    startRescue();
    EXPECT_EQ(NAV_TRACK_LINE, g_nav.track);
    EXPECT_NEAR(400.0f, g_nav.trackStartEfM.y, 0.2f);
    EXPECT_NEAR(0.0f, targetNorthM(), 0.2f);
    EXPECT_NEAR(50.0f, g_nav.targetEfM.v[ENU_U], 0.1f);

    flightPlanNavDisengage();
    craftAt(0.0f, 400.0f, 44.0f);
    startRescue();
    EXPECT_EQ(NAV_TRACK_LOITER, g_nav.track);
}

TEST_F(FlightPlanNavWingTest, NearHomeARescueLandsAtHome)
{
    craftAt(30.0f, 80.0f, 25.0f);
    startRescue();
    EXPECT_EQ(NAV_TRACK_LOITER, g_nav.track);
    EXPECT_NEAR(0.0f, targetEastM(), 0.2f);
    EXPECT_NEAR(0.0f, targetNorthM(), 0.2f);
    EXPECT_NEAR(25.0f, g_nav.targetEfM.v[ENU_U], 0.1f);
    tick();
    EXPECT_EQ(FP_NAV_LANDING, flightPlanNavGetState());
    EXPECT_EQ(0, flightPlanNavGetCurrentIndex());
    EXPECT_FLOAT_EQ(0.0f, g_landingSite.touchdownEnuM.v[ENU_U]);
}

TEST_F(FlightPlanNavWingTest, ARescueLandsOnTheArmingPointsGround)
{
    GPS_home_llh.altCm = 10300;     // home's GPS altitude has drifted 3 m from the estimator's zero
    craftAt(30.0f, 80.0f, 25.0f);
    startRescue();
    tick();
    ASSERT_EQ(FP_NAV_LANDING, flightPlanNavGetState());
    EXPECT_FLOAT_EQ(0.0f, g_landingSite.touchdownEnuM.v[ENU_U]);

    // thrown from the hand, the ground is below where it was armed
    SetUp();
    g_homeGroundM = -1.5f;
    craftAt(30.0f, 80.0f, 25.0f);
    startRescue();
    tick();
    ASSERT_EQ(FP_NAV_LANDING, flightPlanNavGetState());
    EXPECT_FLOAT_EQ(-1.5f, g_landingSite.touchdownEnuM.v[ENU_U]);
}

TEST_F(FlightPlanNavWingTest, ARescueNeedsNoHeading)
{
    stateFlags = GPS_FIX | GPS_FIX_HOME;
    craftAt(10.0f, 10.0f, 20.0f);
    EXPECT_TRUE(flightPlanNavStageRescuePlan());
    craftAt(0.0f, 400.0f, 20.0f);
    EXPECT_TRUE(flightPlanNavStageRescuePlan());

    stateFlags = GPS_FIX;
    EXPECT_FALSE(flightPlanNavStageRescuePlan());           // but it does need home
}

static float rescueReturnAltitudeM(uint8_t mode, float upM)
{
    gpsRescueConfigMutable()->altitudeMode = mode;
    flightPlanNavDisengage();
    craftAt(0.0f, 400.0f, upM);
    stateFlags = GPS_FIX | GPS_FIX_HOME;
    flightPlanNavStageRescuePlan();
    flightPlanNavEngage();
    return g_nav.targetEfM.v[ENU_U];
}

TEST_F(FlightPlanNavWingTest, TheRescueReturnsAtTheAltitudeItsModeAsksFor)
{
    g_maxAltitudeCm = 4500.0f;
    EXPECT_NEAR(50.0f, rescueReturnAltitudeM(GPS_RESCUE_ALT_MODE_FIXED, 20.0f), 0.1f);
    EXPECT_NEAR(30.0f, rescueReturnAltitudeM(GPS_RESCUE_ALT_MODE_CURRENT, 20.0f), 0.1f);
    EXPECT_NEAR(55.0f, rescueReturnAltitudeM(GPS_RESCUE_ALT_MODE_MAX, 20.0f), 0.1f);

    // and never below where it is
    EXPECT_NEAR(80.0f, rescueReturnAltitudeM(GPS_RESCUE_ALT_MODE_FIXED, 80.0f), 0.1f);
    EXPECT_NEAR(80.0f, rescueReturnAltitudeM(GPS_RESCUE_ALT_MODE_MAX, 80.0f), 0.1f);
}

TEST_F(FlightPlanNavWingTest, TheRescueReturnsNoLowerThanTheLandingApproachIsFlown)
{
    autopilotWingConfigMutable()->landApproachAlt = 60;
    EXPECT_NEAR(60.0f, rescueReturnAltitudeM(GPS_RESCUE_ALT_MODE_FIXED, 20.0f), 0.1f);
    EXPECT_NEAR(80.0f, rescueReturnAltitudeM(GPS_RESCUE_ALT_MODE_FIXED, 80.0f), 0.1f);
}

TEST_F(FlightPlanNavWingTest, ARescueReplacesTheMissionInFlight)
{
    addWaypoint(0.0f, 2000.0f, WAYPOINT_TYPE_FLYOVER);
    flightPlanNavEngage();
    craftAt(0.0f, 400.0f, 20.0f);
    tick();
    stateFlags = GPS_FIX | GPS_FIX_HOME;
    ASSERT_TRUE(flightPlanNavStageRescuePlan());
    EXPECT_TRUE(flightPlanNavIsRescuePlanActive());
    EXPECT_EQ(NAV_TRACK_LOITER, g_nav.track);
    EXPECT_NEAR(400.0f, targetNorthM(), 0.2f);
    EXPECT_NEAR(50.0f, g_nav.targetEfM.v[ENU_U], 0.1f);

    // with no climb to make its line home starts where it is, not at the last waypoint passed
    SetUp();
    addWaypoint(0.0f, 300.0f, WAYPOINT_TYPE_FLYOVER);
    addWaypoint(300.0f, 300.0f, WAYPOINT_TYPE_FLYOVER);
    flightPlanNavEngage();
    flyTo(0.0f, 301.0f);
    ASSERT_EQ(1, flightPlanNavGetCurrentIndex());
    craftAt(100.0f, 300.0f, 48.0f);
    tick();
    stateFlags = GPS_FIX | GPS_FIX_HOME;
    ASSERT_TRUE(flightPlanNavStageRescuePlan());
    EXPECT_EQ(NAV_TRACK_LINE, g_nav.track);
    EXPECT_NEAR(100.0f, g_nav.trackStartEfM.x, 0.2f);
    EXPECT_NEAR(300.0f, g_nav.trackStartEfM.y, 0.2f);
}

TEST_F(FlightPlanNavWingTest, ARescueRidesOutAPositionLossThenGivesUp)
{
    craftAt(0.0f, 400.0f, 46.0f);
    startRescue();
    flyRescueTo(0.0f, 380.0f, 48.0f);
    g_validXY = false;
    for (int i = 0; i < 295; i++) {
        tick();
    }
    EXPECT_EQ(FP_NAV_TARGETING, flightPlanNavGetState());
    for (int i = 0; i < 10; i++) {
        tick();
    }
    EXPECT_EQ(FP_NAV_ABORTED, flightPlanNavGetState());
    EXPECT_EQ(FP_ABORT_ESTIMATOR, flightPlanNavGetAbortReason());
}

TEST_F(FlightPlanNavWingTest, AWaypointLoiteredAboutNextIsReachedOnTheLoiter)
{
    addWaypoint(0.0f, 400.0f, WAYPOINT_TYPE_FLYOVER);
    addWaypoint(0.0f, 400.0f, WAYPOINT_TYPE_HOLD, 300);
    flightPlanNavEngage();
    flyTo(0.0f, 299.0f);
    EXPECT_EQ(0, flightPlanNavGetCurrentIndex());
    flyTo(0.0f, 301.0f);
    EXPECT_EQ(1, flightPlanNavGetCurrentIndex());

    // a loiter somewhere else waits for the waypoint itself
    SetUp();
    addWaypoint(0.0f, 400.0f, WAYPOINT_TYPE_FLYOVER);
    addWaypoint(0.0f, 600.0f, WAYPOINT_TYPE_HOLD, 300);
    flightPlanNavEngage();
    flyTo(0.0f, 380.0f);
    EXPECT_EQ(0, flightPlanNavGetCurrentIndex());
}

TEST_F(FlightPlanNavWingTest, TheSwitchDescentLeavesTouchdownAlone)
{
    // a sink that stops, as a multirotor's does on the ground and a wing's may in the air
    autopilotConfigMutable()->landingVelocityThreshold = 50;
    autopilotConfigMutable()->landingDetectionTime = 10;
    g_estimate.velocity.v[ENU_U] = -200.0f;
    for (int i = 0; i < 1000; i++) {
        g_micros += 10000;
        if (i == 200) {
            g_estimate.velocity.v[ENU_U] = 0.0f;
        }
        flightPlanNavRescueDescent(true, g_micros);
    }
    EXPECT_TRUE(flightPlanNavIsRescueDescentActive());
    EXPECT_TRUE(g_emergencyDescent);
    EXPECT_FLOAT_EQ(200.0f, g_emergencyDescentRateCmS);
    EXPECT_EQ(0, g_disarmCalls);

    flightPlanNavRescueDescent(false, g_micros);
    EXPECT_FALSE(g_emergencyDescent);
}

TEST_F(FlightPlanNavWingTest, NearHomeAndOnAMissionsLandWaypointTheGroundIsKnown)
{
    craftAt(30.0f, 80.0f, 25.0f);
    startRescue();
    tick();
    ASSERT_EQ(FP_NAV_LANDING, flightPlanNavGetState());
    EXPECT_TRUE(g_landingSite.groundKnown);

    SetUp();
    addWaypoint(0.0f, 600.0f, WAYPOINT_TYPE_LAND, 0, WAYPOINT_PATTERN_NONE, WAYPOINT_YAW_DEFAULT, 0.0f);
    flightPlanNavEngage();
    flyTo(0.0f, 590.0f);
    flyTo(0.0f, 600.0f, 0.0f);
    ASSERT_EQ(FP_NAV_LANDING, flightPlanNavGetState());
    EXPECT_TRUE(g_landingSite.groundKnown);
}
