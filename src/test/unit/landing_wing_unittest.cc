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
#include <string.h>

extern "C" {
    #include "platform.h"
    #include "build/debug.h"

    #include "common/maths.h"
    #include "common/vector.h"

    #include "fc/rc.h"
    #include "fc/runtime_config.h"

    #include "flight/autopilot.h"
    #include "flight/failsafe.h"
    #include "flight/flight_plan_nav_wing.h"
    #include "flight/imu.h"
    #include "flight/landing_wing.h"
    #include "flight/launch_wing.h"
    #include "flight/position.h"
    #include "flight/position_estimator.h"
    #include "flight/position_nav.h"

    #include "pg/autopilot.h"
    #include "pg/autopilot_wing.h"
    #include "pg/pg.h"

    #include "rx/rx.h"

    #include "sensors/acceleration.h"
    #include "sensors/gyro.h"
    #include "sensors/rangefinder.h"

    extern uint8_t __config_start;
    extern uint8_t __config_end;

    int16_t debug[DEBUG16_VALUE_COUNT];
    uint8_t debugMode;
    uint8_t armingFlags;
    acc_t acc;
    gyro_t gyro;
    attitudeEulerAngles_t attitude;
    matrix33_t rMat;

    bool landingWingContact(landingWingTouchdown_t *touchdown, timeUs_t nowUs, float heightM, bool heightKnown, float sinkDemandCmS);
    void landingWingPattern(const landingWingSite_t *site, float headingDeg, int8_t side, landingWingPattern_t *out);
    float landingWingGlideAltM(float toTouchdownM, float finalHeightM, float slopeRad);
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

namespace {

enum { CMD_NONE = 0, CMD_LOITER, CMD_LINE };

struct Command {
    int kind;
    vector2_t a;        // loiter centre, or line start
    vector2_t b;        // line end
    float altM;
    float vertRateMps;
    float startAltM;
};

Command g_cmd;
float g_navAltM;
bool g_limitsSet;
autopilotWingLimits_t g_limits;
autopilotWingLimitsOwner_e g_limitsOwner;

positionEstimate3d_t g_est;
float g_altitudeCm;
timeUs_t g_nowUs;
bool g_rangefinderHealthy;
int32_t g_rangeCm;
float g_rollStick;
float g_pitchStick;
bool g_rxValid;
bool g_failsafe;
bool g_hasLaunchCourse;
float g_launchCourseDeg;
float g_pitchDeg;
float g_idlePitchDeg;

const float MIN_TURN_RADIUS_M = 43.0f;
const float LOITER_RADIUS_M = 60.0f;
const float L1_M = 40.0f;
const float MIN_THROTTLE = 0.25f;

} // namespace

extern "C" {

float getAltitudeCmControl(void) { return g_altitudeCm; }
const positionEstimate3d_t *positionEstimatorGetEstimate(void) { return &g_est; }

bool positionNavHasActiveTarget(void) { return g_cmd.kind != CMD_NONE; }
float positionNavGetTargetAltitudeCm(void) { return g_navAltM * 100.0f; }
void positionNavLowerTargetAltitude(float upM) { g_cmd.altM = fminf(g_cmd.altM, upM); }

void flightPlanWingLoiterAt(const vector2_t *centreEnuM, float altM, float vertRateMps, float startAltM)
{
    g_cmd = (Command){ CMD_LOITER, *centreEnuM, *centreEnuM, altM, vertRateMps, startAltM };
}

void flightPlanWingFlyLine(const vector2_t *startEnuM, const vector2_t *endEnuM, float altM, float vertRateMps, float startAltM)
{
    g_cmd = (Command){ CMD_LINE, *startEnuM, *endEnuM, altM, vertRateMps, startAltM };
}

float flightPlanWingCommandedAltitudeM(void) { return positionNavHasActiveTarget() ? g_navAltM : g_altitudeCm * 0.01f; }

float autopilotWingLoiterRadiusM(void) { return LOITER_RADIUS_M; }
float autopilotWingAttitudePitchDeg(void) { return g_pitchDeg; }
float autopilotWingIdlePitchDeg(float) { return g_idlePitchDeg; }
float autopilotWingMinTurnRadiusM(void) { return MIN_TURN_RADIUS_M; }
float autopilotWingTurnDistanceM(float turnDeg) { return L1_M * fminf(fabsf(turnDeg) / 90.0f, 1.0f); }
float g_settleM;
float autopilotWingLineSettleDistanceM(void) { return g_settleM; }

void autopilotWingDefaultLimits(autopilotWingLimits_t *limits)
{
    memset(limits, 0, sizeof(*limits));
    limits->bankLimitDeg = 35.0f;
    limits->pitchMinDeg = -12.0f;
    limits->pitchMaxDeg = 15.0f;
    limits->throttleMin = MIN_THROTTLE;
    limits->throttleMax = 0.9f;
}

void autopilotWingSetLimits(autopilotWingLimitsOwner_e owner, const autopilotWingLimits_t *limits)
{
    g_limits = *limits;
    g_limitsSet = true;
    g_limitsOwner = owner;
}

void autopilotWingClearLimits(autopilotWingLimitsOwner_e owner)
{
    if (owner == g_limitsOwner) {
        g_limitsSet = false;
    }
}

bool g_thrown;
bool launchWingThrown(void) { return g_thrown; }

bool launchWingGetCourseDeg(float *courseDeg)
{
    *courseDeg = g_launchCourseDeg;
    return g_hasLaunchCourse;
}

float getRcDeflectionAbs(int axis) { return fabsf(axis == FD_ROLL ? g_rollStick : g_pitchStick); }
bool rxAreFlightChannelsValid(void) { return g_rxValid; }
bool failsafeIsActive(void) { return g_failsafe; }

bool rangefinderIsHealthy(void) { return g_rangefinderHealthy; }
int32_t rangefinderGetLatestAltitude(void) { return g_rangeCm; }

} // extern "C"

static const timeUs_t STEP_US = 20000;

// Level, slowing along the nose at g.
static void brake(float g)
{
    acc.dev.acc_1G = 512;
    acc.dev.acc_1G_rec = 1.0f / acc.dev.acc_1G;
    acc.accADC.v[X] = -g * acc.dev.acc_1G;
}

// The aircraft, at upM in the estimator frame, flying courseDeg at speedMps over the ground.
static void craft(float eastM, float northM, float upM, float courseDeg, float speedMps, float climbMps = 0.0f)
{
    const float courseRad = DEGREES_TO_RADIANS(courseDeg);
    g_est.position.v[ENU_E] = eastM * 100.0f;
    g_est.position.v[ENU_N] = northM * 100.0f;
    g_est.position.v[ENU_U] = upM * 100.0f;
    g_est.velocity.v[ENU_E] = speedMps * sinf(courseRad) * 100.0f;
    g_est.velocity.v[ENU_N] = speedMps * cosf(courseRad) * 100.0f;
    g_est.velocity.v[ENU_U] = climbMps * 100.0f;
    g_altitudeCm = upM * 100.0f;
    g_navAltM = upM;
}

static bool step(int steps = 1)
{
    bool landed = false;
    for (int i = 0; i < steps; i++) {
        g_nowUs += STEP_US;
        landed = landingWingUpdate(g_nowUs);
    }
    return landed;
}

static float landingHeadingDeg(void)
{
    return landingWingGetFinalCourseDeg();
}

static landingWingSite_t site(float headingDeg = 0.0f)
{
    landingWingSite_t touchdown;
    memset(&touchdown, 0, sizeof(touchdown));
    touchdown.groundKnown = true;
    touchdown.headingDeg = headingDeg;
    touchdown.sinkRateMps = 2.0f;
    return touchdown;
}

static float finalHeightM(void)
{
    const landingWingSite_t touchdown = site();
    landingWingPattern_t pattern;
    landingWingPattern(&touchdown, 0.0f, -1, &pattern);
    return pattern.finalHeightM;
}

static void startLanding(const landingWingSite_t &touchdown)
{
    landingWingStop();
    landingWingStart(&touchdown, g_nowUs);
}

class LandingWingTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        pgResetAll();
        // the geometry is worked at a 6 deg slope, and a flare sinking as fast as the glide floats
        // no further than the glide slope comes down
        autopilotWingConfigMutable()->landGlideAngle = 6;
        autopilotWingConfigMutable()->landFlareSink = 16;
        memset(&g_cmd, 0, sizeof(g_cmd));
        g_limitsSet = false;
        memset(&g_limits, 0, sizeof(g_limits));
        memset(&g_est, 0, sizeof(g_est));
        g_est.isValidXY = true;
        g_nowUs = 1000000;
        g_rangefinderHealthy = false;
        g_rangeCm = -1;
        g_rollStick = 0.0f;
        g_pitchStick = 0.0f;
        g_rxValid = true;
        g_failsafe = false;
        g_hasLaunchCourse = false;
        g_launchCourseDeg = 0.0f;
        g_thrown = false;
        g_settleM = 0.0f;
        g_pitchDeg = 0.0f;
        g_idlePitchDeg = 90.0f;
        acc.accMagnitude = 1.0f;
        brake(0.0f);
        memset(&gyro, 0, sizeof(gyro));
        memset(&attitude, 0, sizeof(attitude));
        armingFlags = 0;
        debugMode = DEBUG_AUTOPILOT_LANDING;
        memset(debug, 0, sizeof(debug));
        craft(0.0f, -300.0f, 50.0f, 0.0f, 15.0f);
        landingWingNoteDepartureCourse(g_nowUs);   // disarmed: forgets any departure
        landingWingStop();
    }

    // Round the right hand loiter about the touchdown at upM, 5 deg a step, until the landing
    // leaves it or maxDeg is flown; speedFor gives the groundspeed at a course.
    static void circle(float upM, float maxDeg, float (*speedFor)(float courseDeg) = nullptr)
    {
        for (float angleDeg = 0.0f; angleDeg < maxDeg && landingWingGetPhase() == LANDING_WING_LOITER_DOWN; angleDeg += 5.0f) {
            const float angleRad = DEGREES_TO_RADIANS(angleDeg);
            const float courseDeg = fmodf(angleDeg + 90.0f, 360.0f);
            craft(LOITER_RADIUS_M * sinf(angleRad), LOITER_RADIUS_M * cosf(angleRad), upM, courseDeg,
                  speedFor ? speedFor(courseDeg) : 15.0f);
            step();
        }
    }

    // Down the loiter and round the pattern to the start of final, landing north on a left hand
    // pattern: downwind is flown south 107.5 m west of the touchdown, base east, final north.
    static void flyToFinal(void)
    {
        startLanding(site());
        circle(30.0f, 1080.0f);
        ASSERT_EQ(LANDING_WING_ALIGN, landingWingGetPhase());
        craft(-107.5f, 55.0f, 30.0f, 180.0f, 15.0f);
        step();
        ASSERT_EQ(LANDING_WING_DOWNWIND, landingWingGetPhase());
        craft(-107.5f, -145.0f, 30.0f, 180.0f, 15.0f);
        step();
        ASSERT_EQ(LANDING_WING_BASE, landingWingGetPhase());
        craft(-5.0f, -150.0f, 16.0f, 90.0f, 15.0f);
        step();
        ASSERT_EQ(LANDING_WING_FINAL, landingWingGetPhase());
    }

    // Down final at 15 m/s to toTouchdownM short of the touchdown at heightM, rightM to the side of
    // it, on the climb that takes it there, and climbing at climbMps once there; short of there if it
    // goes around.
    static void onFinal(float toTouchdownM, float heightM, float rightM = 0.0f, float climbMps = -1.6f)
    {
        const float fromEastM = g_est.position.v[ENU_E] * 0.01f;
        const float fromNorthM = g_est.position.v[ENU_N] * 0.01f;
        const float fromUpM = g_altitudeCm * 0.01f;
        const float stepS = STEP_US * 1e-6f;
        const int steps = MAX(1, (int)ceilf(fabsf(-toTouchdownM - fromNorthM) / (15.0f * stepS)));
        for (int i = 1; i < steps && landingWingGetPhase() != LANDING_WING_GO_AROUND; i++) {
            const float f = (float)i / steps;
            craft(fromEastM + f * (rightM - fromEastM), fromNorthM + f * (-toTouchdownM - fromNorthM),
                  fromUpM + f * (heightM - fromUpM), 0.0f, 15.0f, (heightM - fromUpM) / (steps * stepS));
            step();
        }
        if (landingWingGetPhase() != LANDING_WING_GO_AROUND) {
            craft(rightM, -toTouchdownM, heightM, 0.0f, 15.0f, climbMps);
            step();
        }
    }

    static void flyToFlare(void)
    {
        flyToFinal();
        onFinal(40.0f, 4.0f);
        onFinal(28.0f, 2.9f);
        ASSERT_EQ(LANDING_WING_FLARE, landingWingGetPhase());
    }
};

// Geometry

TEST_F(LandingWingTest, ThePatternLiesOnTheSideItTurnsTo)
{
    const landingWingSite_t touchdown = site();
    landingWingPattern_t pattern;

    // north, left hand: downwind to the west, flown south from abeam beyond the touchdown
    landingWingPattern(&touchdown, 0.0f, -1, &pattern);
    EXPECT_NEAR(0.0f, pattern.finalEnuM.x, 0.01f);
    EXPECT_NEAR(-150.0f, pattern.finalEnuM.y, 0.01f);
    EXPECT_NEAR(-107.5f, pattern.baseEnuM.x, 0.01f);        // 2.5 turn radii out
    EXPECT_NEAR(-150.0f, pattern.baseEnuM.y, 0.01f);
    EXPECT_NEAR(-107.5f, pattern.entryEnuM.x, 0.01f);
    EXPECT_NEAR(60.0f, pattern.entryEnuM.y, 0.01f);         // a loiter radius beyond
    EXPECT_NEAR(150.0f * tanf(DEGREES_TO_RADIANS(6.0f)), pattern.finalHeightM, 0.01f);

    // north, right hand: to the east
    landingWingPattern(&touchdown, 0.0f, 1, &pattern);
    EXPECT_NEAR(107.5f, pattern.baseEnuM.x, 0.01f);
    EXPECT_NEAR(107.5f, pattern.entryEnuM.x, 0.01f);
    EXPECT_NEAR(60.0f, pattern.entryEnuM.y, 0.01f);

    // east, left hand: to the north
    landingWingPattern(&touchdown, 90.0f, -1, &pattern);
    EXPECT_NEAR(-150.0f, pattern.finalEnuM.x, 0.01f);
    EXPECT_NEAR(0.0f, pattern.finalEnuM.y, 0.01f);
    EXPECT_NEAR(-150.0f, pattern.baseEnuM.x, 0.01f);
    EXPECT_NEAR(107.5f, pattern.baseEnuM.y, 0.01f);
    EXPECT_NEAR(60.0f, pattern.entryEnuM.x, 0.01f);
    EXPECT_NEAR(107.5f, pattern.entryEnuM.y, 0.01f);

    // east, right hand: to the south
    landingWingPattern(&touchdown, 90.0f, 1, &pattern);
    EXPECT_NEAR(-107.5f, pattern.baseEnuM.y, 0.01f);
    EXPECT_NEAR(-107.5f, pattern.entryEnuM.y, 0.01f);
}

TEST_F(LandingWingTest, FinalIsTurnedOntoFarEnoughOutToSettleOnItBeforeTheSlope)
{
    g_settleM = 80.0f;
    const landingWingSite_t touchdown = site();
    landingWingPattern_t pattern;
    landingWingPattern(&touchdown, 0.0f, -1, &pattern);
    EXPECT_NEAR(0.0f, pattern.finalEnuM.x, 0.01f);
    EXPECT_NEAR(-230.0f, pattern.finalEnuM.y, 0.01f);
    EXPECT_NEAR(-230.0f, pattern.baseEnuM.y, 0.01f);
    EXPECT_NEAR(150.0f * tanf(DEGREES_TO_RADIANS(6.0f)), pattern.finalHeightM, 0.01f);

    // flown level until the slope
    EXPECT_FLOAT_EQ(pattern.finalHeightM, landingWingGlideAltM(200.0f, pattern.finalHeightM, DEGREES_TO_RADIANS(6.0f)));
}

TEST_F(LandingWingTest, ALongFinalWidensThePatternAndTopsOutAtTheApproachAltitude)
{
    autopilotWingConfigMutable()->landFinalLength = 400;
    const landingWingSite_t touchdown = site();
    landingWingPattern_t pattern;
    landingWingPattern(&touchdown, 0.0f, 1, &pattern);
    EXPECT_NEAR(200.0f, pattern.baseEnuM.x, 0.01f);
    EXPECT_NEAR(400.0f / 3.0f, pattern.entryEnuM.y, 0.01f);
    EXPECT_FLOAT_EQ(30.0f, pattern.finalHeightM);
}

TEST_F(LandingWingTest, TheGlideSlopeComesDownToTheTouchdown)
{
    const float slopeRad = DEGREES_TO_RADIANS(6.0f);
    EXPECT_NEAR(15.77f, landingWingGlideAltM(500.0f, 15.77f, slopeRad), 0.01f);
    EXPECT_NEAR(100.0f * tanf(slopeRad), landingWingGlideAltM(100.0f, 15.77f, slopeRad), 0.01f);
    EXPECT_FLOAT_EQ(0.0f, landingWingGlideAltM(0.0f, 15.77f, slopeRad));
    EXPECT_FLOAT_EQ(0.0f, landingWingGlideAltM(-20.0f, 15.77f, slopeRad));
}

TEST_F(LandingWingTest, TheGlideSlopeAimsShortByHowFarTheFlareFloats)
{
    // from 3 m the flare eases the 1.58 m/s glide at 15 m/s down to 0.5 m/s: 28.5 m of glide slope
    // stretched by ln(1.58 / 0.5)
    autopilotWingConfigMutable()->landFlareSink = 5;
    const float slope = tanf(DEGREES_TO_RADIANS(6.0f));
    const float floatM = 3.0f / slope * logf(1500.0f * slope / 50.0f);
    const landingWingSite_t touchdown = site();
    landingWingPattern_t pattern;
    landingWingPattern(&touchdown, 0.0f, -1, &pattern);
    EXPECT_NEAR((150.0f - floatM) * slope, pattern.finalHeightM, 0.05f);

    flyToFinal();
    onFinal(100.0f, (100.0f - floatM) * slope);
    EXPECT_NEAR((100.0f - floatM) * slope, g_cmd.altM, 0.05f);
    onFinal(floatM + 30.0f, 30.0f * slope);
    EXPECT_NEAR(30.0f * slope, g_cmd.altM, 0.05f);

    // slower over the ground ahead of the slope, into a headwind, the flare carries it less far, and
    // speeding up down the slope does not move the aim
    SetUp();
    autopilotWingConfigMutable()->landFlareSink = 5;
    flyToFinal();
    const float flareS = 3.0f / (15.0f * slope) * (logf(1500.0f * slope / 50.0f) + 1.0f);
    const float floatSlowM = 10.0f * flareS - 3.0f / slope;
    craft(0.0f, -150.0f, finalHeightM(), 0.0f, 10.0f);
    step();
    craft(0.0f, -100.0f, (100.0f - floatSlowM) * slope, 0.0f, 15.0f);
    step();
    EXPECT_NEAR((100.0f - floatSlowM) * slope, g_cmd.altM, 0.05f);
}

// The approach

TEST_F(LandingWingTest, ItComesDownTheLoiterThenFliesThePatternToTheGround)
{
    startLanding(site());
    ASSERT_EQ(LANDING_WING_LOITER_DOWN, landingWingGetPhase());
    EXPECT_EQ(CMD_LOITER, g_cmd.kind);
    EXPECT_NEAR(0.0f, g_cmd.a.x, 0.01f);
    EXPECT_NEAR(0.0f, g_cmd.a.y, 0.01f);
    EXPECT_FLOAT_EQ(30.0f, g_cmd.altM);
    EXPECT_FLOAT_EQ(2.0f, g_cmd.vertRateMps);

    // a lap high does not do
    circle(45.0f, 720.0f);
    EXPECT_EQ(LANDING_WING_LOITER_DOWN, landingWingGetPhase());
    // at the approach altitude it leaves heading for the downwind leg
    circle(30.0f, 1080.0f);
    ASSERT_EQ(LANDING_WING_ALIGN, landingWingGetPhase());
    EXPECT_EQ(CMD_LINE, g_cmd.kind);
    EXPECT_NEAR(-107.5f, g_cmd.b.x, 0.01f);
    EXPECT_NEAR(60.0f, g_cmd.b.y, 0.01f);

    craft(-107.5f, 55.0f, 30.0f, 180.0f, 15.0f);
    step();
    ASSERT_EQ(LANDING_WING_DOWNWIND, landingWingGetPhase());
    EXPECT_NEAR(-107.5f, g_cmd.b.x, 0.01f);
    EXPECT_NEAR(-150.0f, g_cmd.b.y, 0.01f);
    EXPECT_FLOAT_EQ(30.0f, g_cmd.altM);

    // base turns in a turn distance short of its corner, and comes down to the top of the slope
    craft(-107.5f, -105.0f, 30.0f, 180.0f, 15.0f);
    step();
    EXPECT_EQ(LANDING_WING_DOWNWIND, landingWingGetPhase());
    craft(-107.5f, -111.0f, 30.0f, 180.0f, 15.0f);
    step();
    ASSERT_EQ(LANDING_WING_BASE, landingWingGetPhase());
    EXPECT_NEAR(0.0f, g_cmd.b.x, 0.01f);
    EXPECT_NEAR(-150.0f, g_cmd.b.y, 0.01f);
    EXPECT_NEAR(15.77f, g_cmd.altM, 0.01f);
    EXPECT_NEAR(14.23f / (107.5f / 15.0f), g_cmd.vertRateMps, 0.01f);

    craft(-39.0f, -150.0f, 16.0f, 90.0f, 15.0f);
    step();
    ASSERT_EQ(LANDING_WING_FINAL, landingWingGetPhase());
    EXPECT_NEAR(0.0f, g_cmd.a.x, 0.01f);
    EXPECT_NEAR(-150.0f, g_cmd.a.y, 0.01f);
    EXPECT_NEAR(0.0f, g_cmd.b.x, 0.01f);
    EXPECT_NEAR(0.0f, g_cmd.b.y, 0.01f);
    EXPECT_NEAR(1.5f * 15.0f * tanf(DEGREES_TO_RADIANS(6.0f)), g_cmd.vertRateMps, 0.01f);

    // down the glide slope
    craft(0.0f, -150.0f, 16.0f, 0.0f, 15.0f);
    step();
    onFinal(100.0f, 10.5f);
    EXPECT_NEAR(100.0f * tanf(DEGREES_TO_RADIANS(6.0f)), g_cmd.altM, 0.01f);
    onFinal(50.0f, 5.3f);
    EXPECT_NEAR(50.0f * tanf(DEGREES_TO_RADIANS(6.0f)), g_cmd.altM, 0.01f);
    EXPECT_EQ(LANDING_WING_FINAL, landingWingGetPhase());

    onFinal(28.0f, 2.9f);
    ASSERT_EQ(LANDING_WING_FLARE, landingWingGetPhase());

    // the touchdown, and it sits still
    craft(0.0f, 10.0f, 0.1f, 0.0f, 2.0f);
    EXPECT_FALSE(step(99));
    EXPECT_TRUE(step(2));
    EXPECT_EQ(LANDING_WING_TOUCHDOWN, landingWingGetPhase());
}

TEST_F(LandingWingTest, ItLeavesTheLoiterOnlyAfterALapAtTheApproachAltitude)
{
    startLanding(site());
    circle(30.0f, 300.0f);
    EXPECT_EQ(LANDING_WING_LOITER_DOWN, landingWingGetPhase());
    circle(34.0f, 720.0f);
    EXPECT_EQ(LANDING_WING_LOITER_DOWN, landingWingGetPhase());
    circle(32.0f, 720.0f);
    EXPECT_EQ(LANDING_WING_ALIGN, landingWingGetPhase());
}

TEST_F(LandingWingTest, TheFlareTakesItsHeightFromTheRangefinder)
{
    flyToFinal();
    onFinal(60.0f, 6.0f);
    g_rangefinderHealthy = true;
    g_rangeCm = 280;
    step();
    EXPECT_EQ(LANDING_WING_FLARE, landingWingGetPhase());

    // without a reading it is the altitude above the touchdown
    SetUp();
    flyToFinal();
    g_rangefinderHealthy = true;
    onFinal(60.0f, 6.0f);
    EXPECT_EQ(LANDING_WING_FINAL, landingWingGetPhase());
    onFinal(25.0f, 2.9f);
    EXPECT_EQ(LANDING_WING_FLARE, landingWingGetPhase());
}

TEST_F(LandingWingTest, TheBankOnFinalNarrowsToTheFlaresAtTheFlare)
{
    flyToFinal();
    step();
    // still turning in from base, the turn may bank as steeply as any
    EXPECT_FLOAT_EQ(35.0f, g_limits.bankLimitDeg);
    EXPECT_FLOAT_EQ(0.5f, g_limits.throttleMax);              // no more than the cruise throttle

    onFinal(120.0f, 12.0f);
    EXPECT_FLOAT_EQ(15.0f, g_limits.bankLimitDeg);
    onFinal(45.0f, 4.5f);
    EXPECT_NEAR(7.5f, g_limits.bankLimitDeg, 0.01f);
    onFinal(33.0f, 3.5f);
    EXPECT_NEAR(3.0f, g_limits.bankLimitDeg, 0.01f);
}

TEST_F(LandingWingTest, DownTheSlopeTheThrottleIsForThePathAndMayClose)
{
    flyToFinal();
    step();
    // level ahead of the slope it flies on the cruise throttle
    EXPECT_FALSE(g_limits.throttleForPath);
    EXPECT_FLOAT_EQ(MIN_THROTTLE, g_limits.throttleMin);

    onFinal(100.0f, 10.5f);
    EXPECT_TRUE(g_limits.throttleForPath);
    EXPECT_FLOAT_EQ(0.0f, g_limits.throttleMin);
    EXPECT_FLOAT_EQ(0.5f, g_limits.throttleMax);
    EXPECT_FALSE(g_limits.motorStop);
}

// The flare's sink at a height, easing off from the 1.58 m/s glide at 15 m/s.
static float flareSinkAtCmS(float heightM)
{
    return heightM / 3.0f * 1500.0f * tanf(DEGREES_TO_RADIANS(6.0f));
}

TEST_F(LandingWingTest, TheFlareClosesTheThrottleForGoodAndRaisesTheNoseAsTheSinkNeedsWithoutClimbing)
{
    autopilotWingConfigMutable()->landFlareSink = 3;
    g_pitchDeg = -7.0f;         // the attitude it glides down the slope at
    flyToFlare();
    ASSERT_TRUE(g_limitsSet);
    // holding final's line against a crosswind, with no more bank than it can touch down with
    EXPECT_FALSE(g_limits.wingsLevel);
    EXPECT_FLOAT_EQ(3.0f, g_limits.bankLimitDeg);
    EXPECT_TRUE(g_limits.climbRateOverride);
    EXPECT_FLOAT_EQ(0.0f, g_limits.throttleMin);
    EXPECT_FLOAT_EQ(0.0f, g_limits.throttleMax);
    EXPECT_TRUE(g_limits.motorStop);
    EXPECT_NEAR(-7.0f, g_limits.pitchMinDeg, 0.1f);
    EXPECT_FLOAT_EQ(5.0f, g_limits.pitchMaxDeg);

    // sinking faster than the flare's sink, which eases off with the height, the nose comes up at 4 deg/s
    craft(0.0f, -20.0f, 2.5f, 0.0f, 15.0f, -1.6f);
    step(50);
    EXPECT_NEAR(-flareSinkAtCmS(2.5f), g_limits.climbRateCmS, 0.5f);
    EXPECT_NEAR(-3.0f, g_limits.pitchMinDeg, 0.2f);

    // ballooning, it comes back down to the glide's attitude and no further, and never asks to climb
    craft(0.0f, -10.0f, 2.4f, 0.0f, 13.0f, 0.5f);
    step(100);
    EXPECT_FLOAT_EQ(-7.0f, g_limits.pitchMinDeg);
    EXPECT_NEAR(-flareSinkAtCmS(2.4f), g_limits.climbRateCmS, 0.5f);

    // back above the flare height the throttle stays closed
    craft(0.0f, -5.0f, 3.5f, 0.0f, 13.0f, 0.2f);
    step();
    EXPECT_EQ(LANDING_WING_FLARE, landingWingGetPhase());
    EXPECT_TRUE(g_limits.motorStop);
    EXPECT_FLOAT_EQ(0.0f, g_limits.throttleMax);

    // low down it holds the flare's sink, raising the nose to ap_wing_land_flare_pitch and no more
    craft(0.0f, 0.0f, 0.3f, 0.0f, 11.0f, -0.6f);
    step(170);
    EXPECT_FLOAT_EQ(-30.0f, g_limits.climbRateCmS);
    EXPECT_FLOAT_EQ(5.0f, g_limits.pitchMinDeg);
    EXPECT_EQ(LANDING_WING_FLARE, landingWingGetPhase());

    // without a position there is no line to hold
    g_est.isValidXY = false;
    step();
    EXPECT_TRUE(g_limits.wingsLevel);
}

TEST_F(LandingWingTest, AFlareComingInUnderPowerStartsNoHigherThanItGlidesAt)
{
    g_pitchDeg = 2.0f;
    g_idlePitchDeg = -5.0f;
    flyToFlare();
    EXPECT_FLOAT_EQ(-5.0f, g_limits.pitchMinDeg);
}

TEST_F(LandingWingTest, AFlareThatNeverReachesTheGroundGlidesOnDownAtTheAttitudeItCameInAt)
{
    autopilotWingConfigMutable()->landFlareSink = 3;
    g_pitchDeg = -7.0f;
    flyToFlare();
    craft(0.0f, -20.0f, 0.5f, 0.0f, 12.0f, -1.0f);
    // the flare eases 1.58 m/s off to 0.3 m/s from 3 m, then holds it: 5.1 s down, and 3 s more
    const float flareS = 3.0f / (1500.0f * tanf(DEGREES_TO_RADIANS(6.0f)) * 0.01f) * (logf(1500.0f * tanf(DEGREES_TO_RADIANS(6.0f)) / 30.0f) + 1.0f);
    const int steps = lrintf((flareS + 3.0f) / (STEP_US * 1e-6f));
    step(steps - 5);
    EXPECT_FLOAT_EQ(5.0f, g_limits.pitchMinDeg);
    EXPECT_FLOAT_EQ(-30.0f, g_limits.climbRateCmS);

    step(10);
    EXPECT_FLOAT_EQ(-12.0f, g_limits.pitchMinDeg);
    EXPECT_LT(g_limits.pitchMaxDeg, 5.0f);
    step(200);
    EXPECT_FLOAT_EQ(-7.0f, g_limits.pitchMaxDeg);
    // no less sink than it came in with, which it glided at
    EXPECT_LT(g_limits.climbRateCmS, -100.0f);
    EXPECT_TRUE(g_limits.motorStop);
    EXPECT_EQ(LANDING_WING_FLARE, landingWingGetPhase());
}

// Home

TEST_F(LandingWingTest, AfterAThrowTheGroundAtHomeIsBelowTheHandItWasArmedIn)
{
    EXPECT_FLOAT_EQ(0.0f, landingWingHomeGroundM());
    g_thrown = true;
    EXPECT_FLOAT_EQ(-1.5f, landingWingHomeGroundM());
    autopilotWingConfigMutable()->landLaunchHeight = 0;
    EXPECT_FLOAT_EQ(0.0f, landingWingHomeGroundM());

    // a rangefinder reading knows better
    autopilotWingConfigMutable()->landLaunchHeight = 150;
    craft(0.0f, 0.0f, 2.0f, 0.0f, 15.0f);
    EXPECT_FLOAT_EQ(3.5f, landingWingHeightM(landingWingHomeGroundM()));
    g_rangefinderHealthy = true;
    g_rangeCm = 220;
    EXPECT_FLOAT_EQ(2.2f, landingWingHeightM(landingWingHomeGroundM()));
}

// Touchdown

TEST_F(LandingWingTest, TouchdownWaitsUntilItIsStillOnTheGround)
{
    landingWingTouchdown_t touchdown;
    landingWingTouchdownReset(&touchdown);
    craft(0.0f, 0.0f, 0.2f, 0.0f, 1.0f);

    // sinking at the rate commanded
    g_est.velocity.v[ENU_U] = -100.0f;
    for (int i = 0; i < 300; i++) {
        g_nowUs += STEP_US;
        EXPECT_FALSE(landingWingTouchdownUpdate(&touchdown, g_nowUs, 0.2f, 100.0f));
    }
    // slow over the ground but still coming down
    g_est.velocity.v[ENU_U] = -60.0f;
    for (int i = 0; i < 300; i++) {
        g_nowUs += STEP_US;
        EXPECT_FALSE(landingWingTouchdownUpdate(&touchdown, g_nowUs, 0.2f, 100.0f));
    }
    // still: for ap_landing_detection_time
    g_est.velocity.v[ENU_U] = -10.0f;
    for (int i = 0; i < 99; i++) {
        g_nowUs += STEP_US;
        EXPECT_FALSE(landingWingTouchdownUpdate(&touchdown, g_nowUs, 0.2f, 100.0f));
    }
    g_nowUs += 2 * STEP_US;
    EXPECT_TRUE(landingWingTouchdownUpdate(&touchdown, g_nowUs, 0.2f, 100.0f));
}

TEST_F(LandingWingTest, TouchdownNeedsEverythingStill)
{
    landingWingTouchdown_t touchdown;
    struct {
        float speedMps;
        float rollDps;
        float accG;
        float heightM;
        float sinkDemandCmS;
        float bankDeg;
    } const cases[] = {
        { 4.0f, 0.0f, 1.0f, 0.2f, 100.0f, 0.0f },     // rolling along the ground
        { 1.0f, 20.0f, 1.0f, 0.2f, 100.0f, 0.0f },    // rocking
        { 1.0f, 0.0f, 1.3f, 0.2f, 100.0f, 0.0f },     // bumping
        { 1.0f, 0.0f, 1.0f, 5.5f, 100.0f, 10.0f },    // well above the ground, banked
        { 1.0f, 0.0f, 1.0f, 0.2f, 0.0f, 0.0f },       // nothing commands it down
    };
    for (const auto &c : cases) {
        landingWingTouchdownReset(&touchdown);
        craft(0.0f, 0.0f, c.heightM, 0.0f, c.speedMps);
        gyro.gyroADCf[FD_ROLL] = c.rollDps;
        acc.accMagnitude = c.accG;
        attitude.values.roll = lrintf(c.bankDeg * 10.0f);
        bool landed = false;
        for (int i = 0; i < 500; i++) {
            g_nowUs += STEP_US;
            landed = landed || landingWingTouchdownUpdate(&touchdown, g_nowUs, c.heightM, c.sinkDemandCmS);
        }
        EXPECT_FALSE(landed) << "speed " << c.speedMps << " roll " << c.rollDps << " acc " << c.accG
                             << " height " << c.heightM << " sink " << c.sinkDemandCmS << " bank " << c.bankDeg;
    }
}

TEST_F(LandingWingTest, AnIdlingMotorShakingTheGyroDoesNotStopATouchdown)
{
    landingWingTouchdown_t touchdown;
    landingWingTouchdownReset(&touchdown);
    craft(0.0f, 0.0f, 0.2f, 0.0f, 1.0f);
    bool landed = false;
    for (int i = 0; i < 150 && !landed; i++) {
        gyro.gyroADCf[FD_PITCH] = (i % 2) ? 40.0f : -40.0f;
        gyro.gyroADCf[FD_YAW] = (i % 3) ? -25.0f : 50.0f;
        g_nowUs += STEP_US;
        landed = landingWingTouchdownUpdate(&touchdown, g_nowUs, 0.2f, 100.0f);
    }
    EXPECT_TRUE(landed);
}

TEST_F(LandingWingTest, ATouchdownStartedAfterHalfTheTimerRangeIsStillDetected)
{
    g_nowUs = 0x80000000u + 1000000u;
    landingWingTouchdown_t touchdown;
    landingWingTouchdownReset(&touchdown);
    craft(0.0f, 0.0f, 0.2f, 0.0f, 1.0f);
    gyro.gyroADCf[FD_ROLL] = 2.0f;
    bool landed = false;
    for (int i = 0; i < 150 && !landed; i++) {
        g_nowUs += STEP_US;
        landed = landingWingTouchdownUpdate(&touchdown, g_nowUs, 0.2f, 100.0f);
    }
    EXPECT_TRUE(landed);
}

TEST_F(LandingWingTest, OnGroundOfUnknownHeightItHasLandedOnceStillWithItsWingsFlat)
{
    landingWingTouchdown_t touchdown;
    for (const float bankDeg : { 0.0f, 5.0f, -5.0f, 180.0f, -176.0f }) {
        landingWingTouchdownReset(&touchdown);
        craft(0.0f, 0.0f, 25.0f, 0.0f, 1.0f);
        attitude.values.roll = lrintf(bankDeg * 10.0f);
        for (int i = 0; i < 149; i++) {
            g_nowUs += STEP_US;
            EXPECT_FALSE(landingWingTouchdownUpdate(&touchdown, g_nowUs, 25.0f, 100.0f)) << bankDeg;
        }
        g_nowUs += 2 * STEP_US;
        EXPECT_TRUE(landingWingTouchdownUpdate(&touchdown, g_nowUs, 25.0f, 100.0f)) << bankDeg;
    }
}

// Coming down where it is

static landingWingSite_t unknownGround(void)
{
    landingWingSite_t touchdown = site(-1.0f);
    touchdown.groundKnown = false;
    touchdown.touchdownEnuM.v[ENU_E] = 400.0f;
    touchdown.touchdownEnuM.v[ENU_N] = 300.0f;
    return touchdown;
}

TEST_F(LandingWingTest, WithoutTheGroundsHeightItComesStraightDownInTheLoiter)
{
    craft(400.0f, 300.0f, 60.0f, 0.0f, 15.0f, -2.0f);
    startLanding(unknownGround());
    EXPECT_EQ(LANDING_WING_DESCEND, landingWingGetPhase());
    EXPECT_EQ(CMD_LOITER, g_cmd.kind);
    EXPECT_FLOAT_EQ(400.0f, g_cmd.a.x);
    EXPECT_FLOAT_EQ(300.0f, g_cmd.a.y);
    EXPECT_FLOAT_EQ(60.0f, g_cmd.startAltM);
    EXPECT_LT(g_cmd.altM, -500.0f);         // no floor short of the ground it meets
    EXPECT_FLOAT_EQ(2.0f, g_cmd.vertRateMps);

    // no pattern, however long the loiter
    EXPECT_FALSE(step(3000));
    EXPECT_EQ(LANDING_WING_DESCEND, landingWingGetPhase());
    EXPECT_EQ(CMD_LOITER, g_cmd.kind);
}

TEST_F(LandingWingTest, ComingStraightDownItSinksAtLeastAMetreASecond)
{
    landingWingSite_t touchdown = unknownGround();
    touchdown.sinkRateMps = 0.3f;
    craft(400.0f, 300.0f, 60.0f, 0.0f, 15.0f, -0.3f);
    startLanding(touchdown);
    EXPECT_FLOAT_EQ(1.0f, g_cmd.vertRateMps);
}

// Coming straight down over home's ground, which the descent takes for the ground under it.
static void comeDownNearHome(float upM, float climbMps)
{
    craft(40.0f, 30.0f, upM, 0.0f, 15.0f, climbMps);
}

TEST_F(LandingWingTest, ComingStraightDownNearHomeItFlaresOverHomesGroundAndLands)
{
    autopilotWingConfigMutable()->landFlareSink = 3;
    craft(400.0f, 300.0f, 60.0f, 0.0f, 15.0f, -2.0f);
    startLanding(unknownGround());
    comeDownNearHome(60.0f, -2.0f);
    step(10);
    EXPECT_TRUE(g_limits.throttleForPath);
    EXPECT_FLOAT_EQ(0.0f, g_limits.throttleMin);
    EXPECT_FALSE(g_limits.motorStop);
    EXPECT_FALSE(g_limits.wingsLevel);

    // low over home's ground the wings level
    comeDownNearHome(5.0f, -2.0f);
    step();
    EXPECT_TRUE(g_limits.wingsLevel);
    EXPECT_FALSE(g_limits.motorStop);
    EXPECT_FALSE(g_limits.climbRateOverride);

    // and lower it flares, once it has been low for long enough to be no spike in the height: the
    // throttle closed and the wings level, with no line to hold
    comeDownNearHome(2.9f, -2.0f);
    step(10);
    EXPECT_FALSE(g_limits.motorStop);
    step();
    EXPECT_TRUE(g_limits.wingsLevel);
    EXPECT_TRUE(g_limits.motorStop);
    EXPECT_FLOAT_EQ(0.0f, g_limits.throttleMax);
    EXPECT_TRUE(g_limits.climbRateOverride);
    EXPECT_GT(g_limits.climbRateCmS, -200.0f);

    // the sink stops on the ground
    comeDownNearHome(0.5f, 0.0f);
    g_est.velocity.v[ENU_N] = 100.0f;
    EXPECT_FALSE(step());
    EXPECT_FALSE(step(98));
    EXPECT_TRUE(step(3));
    EXPECT_EQ(LANDING_WING_TOUCHDOWN, landingWingGetPhase());
}

TEST_F(LandingWingTest, ASpikeInTheHeightDoesNotStartADescentsFlare)
{
    craft(400.0f, 300.0f, 60.0f, 0.0f, 15.0f, -2.0f);
    startLanding(unknownGround());
    comeDownNearHome(6.0f, -2.0f);
    step();
    comeDownNearHome(1.0f, -2.0f);
    step(5);
    comeDownNearHome(6.0f, -2.0f);
    step(20);
    EXPECT_FALSE(g_limits.motorStop);
    EXPECT_TRUE(g_limits.throttleForPath);
}

TEST_F(LandingWingTest, AwayFromHomeItFlaresUnderPowerOntoWhereverTheGroundIs)
{
    autopilotWingConfigMutable()->landFlareSink = 3;
    craft(400.0f, 300.0f, 60.0f, 0.0f, 15.0f, -2.0f);
    startLanding(unknownGround());
    // circling down gently, as the ground may come at any height, and wider near home's ground
    step(10);
    EXPECT_FALSE(g_limits.wingsLevel);
    EXPECT_FLOAT_EQ(15.0f, g_limits.bankLimitDeg);
    EXPECT_FALSE(g_limits.climbRateOverride);

    craft(400.0f, 300.0f, 5.0f, 0.0f, 15.0f, -2.0f);
    step();
    EXPECT_FALSE(g_limits.wingsLevel);
    EXPECT_FLOAT_EQ(10.0f, g_limits.bankLimitDeg);
    EXPECT_FALSE(g_limits.climbRateOverride);

    // over home's ground it flares under power, and below it holds the flare's sink for as long as it takes
    craft(400.0f, 300.0f, 2.0f, 0.0f, 12.0f, -0.5f);
    step(20);
    // the nose left to the sink and the throttle, rather than held up as the speed goes
    craft(400.0f, 300.0f, -30.0f, 0.0f, 12.0f, -0.8f);
    for (int i = 0; i < 30; i++) {
        step(50);
        ASSERT_FLOAT_EQ(-12.0f, g_limits.pitchMinDeg) << i;
        ASSERT_FALSE(g_limits.wingsLevel) << i;
        ASSERT_FLOAT_EQ(10.0f, g_limits.bankLimitDeg) << i;
        ASSERT_TRUE(g_limits.throttleForPath) << i;
        ASSERT_FALSE(g_limits.motorStop) << i;
        ASSERT_GT(g_limits.throttleMax, 0.0f) << i;
        ASSERT_TRUE(g_limits.climbRateOverride) << i;
        ASSERT_FLOAT_EQ(-30.0f, g_limits.climbRateCmS) << i;
    }

    // the ground braking it closes the throttle
    craft(400.0f, 300.0f, -30.0f, 0.0f, 5.0f, -0.1f);
    brake(0.4f);
    step(10);
    EXPECT_FALSE(g_limits.motorStop);
    step();
    EXPECT_TRUE(g_limits.motorStop);
    EXPECT_FLOAT_EQ(0.0f, g_limits.throttleMax);
}

TEST_F(LandingWingTest, AWrongGuessAtTheGroundInAPoweredFlareGlidesOnDownUntilTheThrottleComesBack)
{
    autopilotWingConfigMutable()->landFlareSink = 3;
    craft(400.0f, 300.0f, 60.0f, 0.0f, 15.0f, -2.0f);
    startLanding(unknownGround());
    craft(400.0f, 300.0f, -30.0f, 0.0f, 12.0f, -0.3f);
    step(100);
    ASSERT_FALSE(g_limits.motorStop);

    // slowing as hard as the ground would in the air closes the throttle; the flare gives up holding the nose up at once,
    // and glides on down no slower than a descent comes down
    brake(0.3f);
    step(11);
    ASSERT_TRUE(g_limits.motorStop);
    brake(0.0f);
    step();
    EXPECT_FLOAT_EQ(-LANDING_WING_DESCENT_MIN_SINK_CMS, g_limits.climbRateCmS);
    EXPECT_FLOAT_EQ(g_limits.pitchMaxDeg, g_pitchDeg);
}

TEST_F(LandingWingTest, ADescentsFlareThatNeverMeetsTheGroundGivesTheThrottleBack)
{
    autopilotWingConfigMutable()->landFlareSink = 3;
    craft(400.0f, 300.0f, 60.0f, 0.0f, 15.0f, -2.0f);
    startLanding(unknownGround());
    comeDownNearHome(2.5f, -1.0f);
    step(20);
    ASSERT_TRUE(g_limits.motorStop);

    // the ground lower than home's: once the flare gives up it flares on down under power
    comeDownNearHome(-5.0f, -1.0f);
    const float glideSinkCmS = 1500.0f * tanf(DEGREES_TO_RADIANS(6.0f));
    const float flareS = 3.0f / (glideSinkCmS * 0.01f) * (logf(glideSinkCmS / 30.0f) + 1.0f);
    step(lrintf((flareS + 3.0f) / (STEP_US * 1e-6f)));
    EXPECT_FALSE(g_limits.motorStop);
    EXPECT_TRUE(g_limits.throttleForPath);
    EXPECT_FALSE(g_limits.wingsLevel);
    EXPECT_FLOAT_EQ(10.0f, g_limits.bankLimitDeg);
    EXPECT_FLOAT_EQ(-30.0f, g_limits.climbRateCmS);
    step(100);
    EXPECT_FALSE(g_limits.motorStop);
}

static landingWingTouchdown_t freshTouchdown(void)
{
    landingWingTouchdown_t touchdown;
    landingWingTouchdownReset(&touchdown);
    return touchdown;
}

TEST_F(LandingWingTest, ContactIsAStoppedSinkOrAnImpactLowDown)
{
    landingWingTouchdown_t touchdown = freshTouchdown();
    craft(0.0f, 0.0f, 1.0f, 0.0f, 10.0f, -1.0f);
    EXPECT_FALSE(landingWingContact(&touchdown, g_nowUs, 1.0f, true, 150.0f));
    g_est.velocity.v[ENU_U] = -20.0f;
    EXPECT_TRUE(landingWingContact(&touchdown, g_nowUs, 1.0f, true, 150.0f));
    EXPECT_FALSE(landingWingContact(&touchdown, g_nowUs, 1.0f, true, 0.0f));      // nothing commands it down
    EXPECT_FALSE(landingWingContact(&touchdown, g_nowUs, 9.5f, true, 150.0f));    // above where the wings level
    g_est.velocity.v[ENU_U] = 60.0f;
    EXPECT_FALSE(landingWingContact(&touchdown, g_nowUs, 1.0f, true, 150.0f));    // bouncing back up

    g_est.velocity.v[ENU_U] = -150.0f;
    acc.accMagnitude = 2.5f;
    EXPECT_TRUE(landingWingContact(&touchdown, g_nowUs, 1.0f, true, 150.0f));
    EXPECT_FALSE(landingWingContact(&touchdown, g_nowUs, 9.5f, true, 150.0f));
}

TEST_F(LandingWingTest, OverGroundOfUnknownHeightContactIsAnImpactOrTheGroundBrakingIt)
{
    // an impact, at any height
    landingWingTouchdown_t touchdown = freshTouchdown();
    craft(0.0f, 0.0f, 20.0f, 0.0f, 10.0f, -1.5f);
    acc.accMagnitude = 2.5f;
    EXPECT_TRUE(landingWingContact(&touchdown, g_nowUs, 20.0f, false, 150.0f));
    acc.accMagnitude = 1.0f;

    // the sink stopping is not enough, as it may in the air
    touchdown = freshTouchdown();
    g_est.velocity.v[ENU_U] = 0.0f;
    EXPECT_FALSE(landingWingContact(&touchdown, g_nowUs, 20.0f, false, 150.0f));
    EXPECT_FALSE(landingWingContact(&touchdown, g_nowUs + 1000000, 20.0f, false, 150.0f));

    // slowing as flight might is not either; slowing harder for a while is the ground braking it
    brake(0.1f);
    EXPECT_FALSE(landingWingContact(&touchdown, g_nowUs + 1000000, 20.0f, false, 150.0f));
    brake(0.3f);
    EXPECT_FALSE(landingWingContact(&touchdown, g_nowUs + 1100000, 20.0f, false, 150.0f));
    EXPECT_FALSE(landingWingContact(&touchdown, g_nowUs + 1250000, 20.0f, false, 150.0f));
    EXPECT_TRUE(landingWingContact(&touchdown, g_nowUs + 1300000, 20.0f, false, 150.0f));
}

TEST_F(LandingWingTest, AHardContactShortensTheWait)
{
    landingWingTouchdown_t touchdown;
    landingWingTouchdownReset(&touchdown);
    craft(0.0f, 0.0f, 0.2f, 0.0f, 1.0f);
    acc.accMagnitude = 2.5f;
    g_nowUs += STEP_US;
    EXPECT_FALSE(landingWingTouchdownUpdate(&touchdown, g_nowUs, 0.2f, 100.0f));
    acc.accMagnitude = 1.0f;
    for (int i = 0; i < 49; i++) {
        g_nowUs += STEP_US;
        EXPECT_FALSE(landingWingTouchdownUpdate(&touchdown, g_nowUs, 0.2f, 100.0f));
    }
    g_nowUs += 2 * STEP_US;
    EXPECT_TRUE(landingWingTouchdownUpdate(&touchdown, g_nowUs, 0.2f, 100.0f));
}

TEST_F(LandingWingTest, ALongFlareThatHasStoppedCountsAsDown)
{
    flyToFlare();
    craft(0.0f, 10.0f, 1.0f, 0.0f, 4.0f, -0.6f);    // the sink never stops, as on a soft slope
    EXPECT_FALSE(step(1490));
    EXPECT_TRUE(step(20));
}

// Going around

static void expectGoingAround(landingWingGoAround_e cause)
{
    ASSERT_EQ(LANDING_WING_GO_AROUND, landingWingGetPhase());
    EXPECT_EQ(cause, landingWingGetGoAround());
    // climbing out wings level at full throttle, along the landing heading
    EXPECT_EQ(CMD_LINE, g_cmd.kind);
    EXPECT_NEAR(g_cmd.a.x, g_cmd.b.x, 0.01f);
    EXPECT_GT(g_cmd.b.y, g_cmd.a.y);
    EXPECT_FLOAT_EQ(30.0f, g_cmd.altM);
}

TEST_F(LandingWingTest, ThePilotSendsItAroundWithTheSticks)
{
    flyToFinal();
    onFinal(60.0f, 6.0f);
    g_pitchStick = 0.35f;
    step();
    expectGoingAround(LANDING_WING_GO_AROUND_STICK);
    step();
    EXPECT_TRUE(g_limits.wingsLevel);
    EXPECT_TRUE(g_limits.climbRateOverride);
    EXPECT_FLOAT_EQ(300.0f, g_limits.climbRateCmS);
    EXPECT_FLOAT_EQ(0.9f, g_limits.throttleMin);
    EXPECT_FLOAT_EQ(0.9f, g_limits.throttleMax);

    // not without a link, nor in a failsafe
    SetUp();
    flyToFinal();
    g_pitchStick = 0.35f;
    g_rxValid = false;
    onFinal(110.0f, 11.5f);
    EXPECT_EQ(LANDING_WING_FINAL, landingWingGetPhase());
    g_rxValid = true;
    g_failsafe = true;
    onFinal(108.0f, 11.3f);
    EXPECT_EQ(LANDING_WING_FINAL, landingWingGetPhase());
}

TEST_F(LandingWingTest, ThePullUpLastsThreeSecondsAndUntilClearOfTheGround)
{
    flyToFinal();
    onFinal(60.0f, 6.0f);
    g_rollStick = -0.5f;
    step();
    expectGoingAround(LANDING_WING_GO_AROUND_STICK);
    g_rollStick = 0.0f;
    craft(0.0f, -40.0f, 9.0f, 0.0f, 15.0f, 3.0f);
    step(145);
    EXPECT_TRUE(g_limitsSet);
    EXPECT_TRUE(g_limits.wingsLevel);
    step(10);
    EXPECT_FALSE(g_limitsSet);

    // still low after three seconds it keeps climbing out
    SetUp();
    flyToFinal();
    onFinal(60.0f, 6.0f);
    g_pitchStick = 0.5f;
    step();
    g_pitchStick = 0.0f;
    step(200);
    EXPECT_TRUE(g_limitsSet);
    craft(0.0f, -40.0f, 8.5f, 0.0f, 15.0f, 3.0f);
    step();
    EXPECT_FALSE(g_limitsSet);
}

TEST_F(LandingWingTest, AGoAroundClimbsFromWhereverItIs)
{
    flyToFinal();
    craft(0.0f, -95.0f, 21.0f, 0.0f, 15.0f);
    g_navAltM = 10.0f;      // the approach had it coming down to the slope
    step();
    expectGoingAround(LANDING_WING_GO_AROUND_SLOPE);
    EXPECT_FLOAT_EQ(21.0f, g_cmd.startAltM);
}

TEST_F(LandingWingTest, OvershootingTheTouchdownHighGoesAround)
{
    flyToFinal();
    onFinal(-25.0f, 4.0f);
    EXPECT_EQ(LANDING_WING_FINAL, landingWingGetPhase());
    onFinal(-31.0f, 4.0f);
    expectGoingAround(LANDING_WING_GO_AROUND_OVERSHOOT);
}

TEST_F(LandingWingTest, StrayingOffTheGlideSlopeGoesAround)
{
    flyToFinal();
    onFinal(100.0f, 20.0f);
    EXPECT_EQ(LANDING_WING_FINAL, landingWingGetPhase());
    onFinal(95.0f, 21.0f);
    expectGoingAround(LANDING_WING_GO_AROUND_SLOPE);

    // too low as much as too high, but not in the last 30 m
    SetUp();
    flyToFinal();
    onFinal(150.0f, 4.0f);
    expectGoingAround(LANDING_WING_GO_AROUND_SLOPE);
}

// Down final with the altitude on the slope, while the climb rate has the aircraft sinking
// divingMps faster than that, as when the approach chases down a step the altitude is taking in.
static void chaseTheSlopeDown(float divingMps, int steps)
{
    const float slope = tanf(DEGREES_TO_RADIANS(6.0f));
    for (int i = 0; i < steps && landingWingGetPhase() == LANDING_WING_FINAL; i++) {
        const float toTouchdownM = -g_est.position.v[ENU_N] * 0.01f - 15.0f * STEP_US * 1e-6f;
        craft(0.0f, -toTouchdownM, fminf(toTouchdownM * slope, finalHeightM()), 0.0f, 15.0f, -15.0f * slope - divingMps);
        step();
    }
}

TEST_F(LandingWingTest, AStepInTheAltitudeTheApproachFollowsDownGoesAroundOnceMoreThanTheTolerance)
{
    flyToFinal();
    craft(0.0f, -150.0f, finalHeightM(), 0.0f, 15.0f);
    step();
    chaseTheSlopeDown(3.0f, 160);
    EXPECT_EQ(LANDING_WING_FINAL, landingWingGetPhase());
    chaseTheSlopeDown(3.0f, 30);
    expectGoingAround(LANDING_WING_GO_AROUND_SLOPE);

    // less than the tolerance, it carries on
    SetUp();
    flyToFinal();
    craft(0.0f, -150.0f, finalHeightM(), 0.0f, 15.0f);
    step();
    chaseTheSlopeDown(3.0f, 140);
    chaseTheSlopeDown(0.0f, 200);
    EXPECT_EQ(LANDING_WING_FINAL, landingWingGetPhase());
}

TEST_F(LandingWingTest, OnALongSlowFinalTheAltitudeDriftingFromItsClimbRateIsNoStep)
{
    // 3 m/s over the ground down the slope, the climb rate reading 0.3 m/s more sink than the altitude shows
    flyToFinal();
    craft(0.0f, -150.0f, finalHeightM(), 0.0f, 3.0f);
    step();
    const float slope = tanf(DEGREES_TO_RADIANS(6.0f));
    for (int i = 0; i < 2000 && landingWingGetPhase() == LANDING_WING_FINAL; i++) {
        const float toTouchdownM = -g_est.position.v[ENU_N] * 0.01f - 3.0f * STEP_US * 1e-6f;
        if (toTouchdownM < 30.0f) {
            break;
        }
        craft(0.0f, -toTouchdownM, g_cmd.altM, 0.0f, 3.0f, -3.0f * slope - 0.3f);
        step();
    }
    EXPECT_EQ(LANDING_WING_FINAL, landingWingGetPhase());
}

TEST_F(LandingWingTest, OffToTheSideLowGoesAround)
{
    flyToFinal();
    onFinal(150.0f, 15.7f, 20.0f);
    EXPECT_EQ(LANDING_WING_FINAL, landingWingGetPhase());
    onFinal(140.0f, 14.7f, -16.0f);
    expectGoingAround(LANDING_WING_GO_AROUND_CROSS_TRACK);
}

TEST_F(LandingWingTest, AheadOfTheSlopeItIsStillSettlingOntoFinal)
{
    g_settleM = 80.0f;
    startLanding(site());
    circle(30.0f, 1080.0f);
    ASSERT_EQ(LANDING_WING_ALIGN, landingWingGetPhase());
    craft(-107.5f, 55.0f, 30.0f, 180.0f, 15.0f);
    step();
    craft(-107.5f, -225.0f, 30.0f, 180.0f, 15.0f);
    step();
    ASSERT_EQ(LANDING_WING_BASE, landingWingGetPhase());
    craft(-5.0f, -230.0f, 16.0f, 90.0f, 15.0f);
    step();
    ASSERT_EQ(LANDING_WING_FINAL, landingWingGetPhase());

    // overshooting the turn, low and level ahead of the slope, banking as steeply as it needs
    onFinal(200.0f, 14.0f, 20.0f);
    EXPECT_EQ(LANDING_WING_FINAL, landingWingGetPhase());
    EXPECT_FLOAT_EQ(35.0f, g_limits.bankLimitDeg);
    // still off to the side on it
    onFinal(140.0f, 14.7f, 20.0f);
    expectGoingAround(LANDING_WING_GO_AROUND_CROSS_TRACK);
}

TEST_F(LandingWingTest, LosingThePositionInThePatternGoesAroundAndWaitsForIt)
{
    startLanding(site());
    circle(30.0f, 1080.0f);
    craft(-107.5f, 55.0f, 30.0f, 180.0f, 15.0f);
    step();
    ASSERT_EQ(LANDING_WING_DOWNWIND, landingWingGetPhase());

    craft(-107.5f, 0.0f, 30.0f, 180.0f, 15.0f);
    g_est.isValidXY = false;
    step();
    expectGoingAround(LANDING_WING_GO_AROUND_POSITION);
    step();
    EXPECT_FALSE(g_limitsSet);      // well clear of the ground there is nothing to pull up from
    step(500);
    EXPECT_EQ(LANDING_WING_GO_AROUND, landingWingGetPhase());

    g_est.isValidXY = true;
    step();
    EXPECT_EQ(LANDING_WING_LOITER_DOWN, landingWingGetPhase());
    EXPECT_EQ(1, landingWingGetAttempts());
}

TEST_F(LandingWingTest, LosingThePositionLowOnFinalCarriesOnStraightDown)
{
    autopilotWingConfigMutable()->landFlareSink = 5;
    flyToFinal();
    onFinal(50.0f, 5.5f);
    g_est.isValidXY = false;
    step();
    EXPECT_EQ(LANDING_WING_FINAL, landingWingGetPhase());
    EXPECT_TRUE(g_limits.wingsLevel);
    EXPECT_TRUE(g_limits.climbRateOverride);
    EXPECT_NEAR(-1500.0f * tanf(DEGREES_TO_RADIANS(6.0f)), g_limits.climbRateCmS, 0.5f);
}

TEST_F(LandingWingTest, AGoAroundClimbsBackToTheApproachAltitudeAndTriesAgain)
{
    flyToFinal();
    onFinal(100.0f, 20.0f);
    onFinal(95.0f, 21.0f);
    expectGoingAround(LANDING_WING_GO_AROUND_SLOPE);
    craft(0.0f, 0.0f, 24.0f, 0.0f, 15.0f, 3.0f);
    step(150);
    EXPECT_EQ(LANDING_WING_GO_AROUND, landingWingGetPhase());
    craft(0.0f, 50.0f, 26.0f, 0.0f, 15.0f, 3.0f);
    step();
    ASSERT_EQ(LANDING_WING_LOITER_DOWN, landingWingGetPhase());
    EXPECT_EQ(1, landingWingGetAttempts());
    EXPECT_EQ(CMD_LOITER, g_cmd.kind);
    EXPECT_FLOAT_EQ(30.0f, g_cmd.altM);
}

TEST_F(LandingWingTest, AfterItsAttemptsItIsCommittedSaveForThePilot)
{
    autopilotWingConfigMutable()->landAttempts = 1;
    flyToFinal();
    onFinal(95.0f, 21.0f);
    expectGoingAround(LANDING_WING_GO_AROUND_SLOPE);
    craft(0.0f, 50.0f, 30.0f, 0.0f, 15.0f);
    step(151);
    ASSERT_EQ(1, landingWingGetAttempts());

    circle(30.0f, 1080.0f);
    craft(-107.5f, 55.0f, 30.0f, 180.0f, 15.0f);
    step();
    craft(-107.5f, -145.0f, 30.0f, 180.0f, 15.0f);
    step();
    craft(-5.0f, -150.0f, 16.0f, 90.0f, 15.0f);
    step();
    ASSERT_EQ(LANDING_WING_FINAL, landingWingGetPhase());
    onFinal(95.0f, 21.0f);
    onFinal(-35.0f, 5.0f);
    onFinal(140.0f, 10.0f, 20.0f);
    EXPECT_EQ(LANDING_WING_FINAL, landingWingGetPhase());

    g_pitchStick = -0.4f;
    onFinal(90.0f, 9.5f);
    EXPECT_EQ(LANDING_WING_GO_AROUND, landingWingGetPhase());
}

// The landing heading

static float headingFlown(const landingWingSite_t &touchdown)
{
    startLanding(touchdown);
    // the course round the loiter as it comes to the approach altitude is the last resort
    for (float angleDeg = 0.0f; angleDeg <= 400.0f && landingWingGetPhase() == LANDING_WING_LOITER_DOWN; angleDeg += 5.0f) {
        const float angleRad = DEGREES_TO_RADIANS(angleDeg);
        craft(LOITER_RADIUS_M * sinf(angleRad), LOITER_RADIUS_M * cosf(angleRad), 30.0f, fmodf(angleDeg + 90.0f, 360.0f), 15.0f);
        step();
    }
    return landingHeadingDeg();
}

TEST_F(LandingWingTest, TheHeadingIsTheSettingThenTheCourseInThenTheLaunchThenTheDepartureThenTheLoiter)
{
    autopilotWingConfigMutable()->landHeading = 200;
    g_hasLaunchCourse = true;
    g_launchCourseDeg = 300.0f;
    EXPECT_NEAR(200.0f, headingFlown(site(90.0f)), 1.0f);

    autopilotWingConfigMutable()->landHeading = 0;
    EXPECT_NEAR(0.0f, headingFlown(site(90.0f)), 1.0f);

    autopilotWingConfigMutable()->landHeading = -1;
    EXPECT_NEAR(90.0f, headingFlown(site(90.0f)), 1.0f);
    EXPECT_NEAR(300.0f, headingFlown(site(-1.0f)), 1.0f);

    g_hasLaunchCourse = false;
    ENABLE_ARMING_FLAG(ARMED);
    craft(0.0f, 0.0f, 10.0f, 45.0f, 12.0f);
    for (int i = 0; i < 160; i++) {
        g_nowUs += STEP_US;
        landingWingNoteDepartureCourse(g_nowUs);
    }
    EXPECT_NEAR(45.0f, headingFlown(site(-1.0f)), 1.0f);

    DISABLE_ARMING_FLAG(ARMED);
    landingWingNoteDepartureCourse(g_nowUs);
    EXPECT_NEAR(90.0f, headingFlown(site(-1.0f)), 1.0f);      // the course a lap round the loiter
}

TEST_F(LandingWingTest, TheDepartureCourseIsTheFirstStraightFlightLowDown)
{
    ENABLE_ARMING_FLAG(ARMED);
    // turning
    for (int i = 0; i < 300; i++) {
        g_nowUs += STEP_US;
        craft(0.0f, 0.0f, 10.0f, i * 0.5f, 12.0f);
        landingWingNoteDepartureCourse(g_nowUs);
    }
    // too high
    craft(0.0f, 0.0f, 35.0f, 270.0f, 12.0f);
    for (int i = 0; i < 300; i++) {
        g_nowUs += STEP_US;
        landingWingNoteDepartureCourse(g_nowUs);
    }
    // too slow
    craft(0.0f, 0.0f, 10.0f, 270.0f, 4.0f);
    for (int i = 0; i < 300; i++) {
        g_nowUs += STEP_US;
        landingWingNoteDepartureCourse(g_nowUs);
    }
    EXPECT_NEAR(90.0f, headingFlown(site(-1.0f)), 1.0f);

    craft(0.0f, 0.0f, 10.0f, 270.0f, 12.0f);
    for (int i = 0; i < 160; i++) {
        g_nowUs += STEP_US;
        landingWingNoteDepartureCourse(g_nowUs);
    }
    // and kept once noted
    craft(0.0f, 0.0f, 10.0f, 90.0f, 12.0f);
    for (int i = 0; i < 300; i++) {
        g_nowUs += STEP_US;
        landingWingNoteDepartureCourse(g_nowUs);
    }
    EXPECT_NEAR(270.0f, headingFlown(site(-1.0f)), 1.0f);
}

// 15 m/s of airspeed in 5 m/s of wind blowing towards the north.
static float northerlyDrift(float courseDeg)
{
    return 15.0f + 5.0f * cosf(DEGREES_TO_RADIANS(courseDeg));
}

static float headingInWind(const landingWingSite_t &touchdown)
{
    startLanding(touchdown);
    for (float angleDeg = 0.0f; angleDeg <= 400.0f && landingWingGetPhase() == LANDING_WING_LOITER_DOWN; angleDeg += 5.0f) {
        const float angleRad = DEGREES_TO_RADIANS(angleDeg);
        const float courseDeg = fmodf(angleDeg + 90.0f, 360.0f);
        craft(LOITER_RADIUS_M * sinf(angleRad), LOITER_RADIUS_M * cosf(angleRad), 30.0f, courseDeg, northerlyDrift(courseDeg));
        step();
    }
    return landingHeadingDeg();
}

TEST_F(LandingWingTest, TooMuchTailwindTurnsTheLandingRound)
{
    EXPECT_NEAR(180.0f, headingInWind(site(0.0f)), 1.0f);
    EXPECT_NEAR(180.0f, headingInWind(site(180.0f)), 1.0f);
    EXPECT_NEAR(90.0f, headingInWind(site(90.0f)), 1.0f);     // a crosswind is no tailwind

    autopilotWingConfigMutable()->landMaxTailwind = 60;
    EXPECT_NEAR(0.0f, headingInWind(site(0.0f)), 1.0f);
    autopilotWingConfigMutable()->landMaxTailwind = 0;
    EXPECT_NEAR(0.0f, headingInWind(site(0.0f)), 1.0f);
}

TEST_F(LandingWingTest, StoppingClearsTheLimits)
{
    flyToFlare();
    ASSERT_TRUE(g_limitsSet);
    landingWingStop();
    EXPECT_FALSE(g_limitsSet);
    EXPECT_EQ(LANDING_WING_IDLE, landingWingGetPhase());
    EXPECT_FALSE(step());
}
