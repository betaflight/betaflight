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
#include <limits.h>
#include <string.h>

extern "C" {

    #include "platform.h"
    #include "build/debug.h"
    #include "pg/pg_ids.h"

    #include "common/filter.h"
    #include "common/vector.h"

    #include "fc/core.h"
    #include "fc/rc_controls.h"
    #include "fc/runtime_config.h"

    #include "flight/alt_hold.h"
    #include "flight/autopilot_multirotor.h"
    #include "flight/failsafe.h"
    #include "flight/imu.h"
    #include "flight/pid.h"
    #include "flight/position.h"
    #include "flight/position_estimator.h"
    #include "flight/position_nav.h"

    #include "io/gps.h"

    #include "rx/rx.h"
    #include "scheduler/scheduler.h"

    #include "pg/alt_hold.h"
    #include "pg/autopilot.h"

    #include "sensors/acceleration.h"
    #include "sensors/gyro.h"

    PG_REGISTER(accelerometerConfig_t, accelerometerConfig, PG_ACCELEROMETER_CONFIG, 0);
    PG_REGISTER(altHoldConfig_t, altHoldConfig, PG_ALTHOLD_CONFIG, 0);
    PG_REGISTER(autopilotConfig_t, autopilotConfig, PG_AUTOPILOT, 0);
    PG_REGISTER(gyroConfig_t, gyroConfig, PG_GYRO_CONFIG, 0);
    PG_REGISTER(positionConfig_t, positionConfig, PG_POSITION, 0);
    PG_REGISTER(rcControlsConfig_t, rcControlsConfig, PG_RC_CONTROLS_CONFIG, 0);

    extern bool testFailsafeActive;
    timeUs_t currentTimeUs = 0;
    bool isAltHoldActive();
    extern float testAltitudeCm;
    extern float testAltitudeDerivativeCmS;
    extern float testCosTiltAngle;
    extern throttleStatus_e testThrottleStatus;
    bool testFailsafeActive = false;
    bool failsafeIsActive(void) { return testFailsafeActive; }
    extern bool testNavActive;
    extern bool testNavReached;
    extern float testNavTargetVelZCmS;
    extern positionNavCommand_t testNavCmd;
}

#include "unittest_macros.h"
#include <algorithm>

#include "gtest/gtest.h"

uint32_t millisRW;
uint32_t millis() {
    return millisRW;
}

class AltholdControlUnittest : public ::testing::Test {
protected:
    void SetUp() override {
        memset(&attitude, 0, sizeof(attitude));
        memset(&gpsSol, 0, sizeof(gpsSol));
        memset(rcCommand, 0, sizeof(rcCommand));
        flightModeFlags = 0;
        millisRW = 0;
        testAltitudeCm = 0.0f;
        testAltitudeDerivativeCmS = 0.0f;
        testCosTiltAngle = 1.0f;
        testThrottleStatus = THROTTLE_LOW;
        testFailsafeActive = false;
        testNavActive = false;
        testNavReached = false;
        testNavTargetVelZCmS = 0.0f;
        memset(&testNavCmd, 0, sizeof(testNavCmd));
        autopilotSetLandingSettle(false);

        autopilotConfig_t *apCfg = autopilotConfigMutable();
        apCfg->hoverThrottle = 1500;
        apCfg->throttleMin = 1000;
        apCfg->throttleMax = 2000;
        apCfg->altitudeP = 50;
        apCfg->altitudeI = 50;
        apCfg->altitudeD = 50;
        apCfg->altitudeF = 0;
        apCfg->landingAltitudeM = 5;

        altHoldConfig_t *ahCfg = altHoldConfigMutable();
        ahCfg->deadband = 10;
        ahCfg->climbRate = 50;

        rxConfigMutable()->mincheck = 1050;

        autopilotInit();
        altHoldInit();
        resetAltitudeControl();
    }
};

TEST(AltholdUnittest, altHoldTransitionsTest)
{
    updateAltHold(currentTimeUs);
    EXPECT_EQ(isAltHoldActive(), false);

    flightModeFlags |= ALT_HOLD_MODE;
    millisRW = 42;
    updateAltHold(currentTimeUs);
    EXPECT_EQ(isAltHoldActive(), true);

    flightModeFlags ^= ALT_HOLD_MODE;
    millisRW = 56;
    updateAltHold(currentTimeUs);
    EXPECT_EQ(isAltHoldActive(), false);

    flightModeFlags |= ALT_HOLD_MODE;
    millisRW = 64;
    updateAltHold(currentTimeUs);
    EXPECT_EQ(isAltHoldActive(), true);
}

TEST(AltholdUnittest, altHoldTransitionsTestUnfinishedExitEnter)
{
    altHoldInit();
    EXPECT_EQ(isAltHoldActive(), false);

    flightModeFlags |= ALT_HOLD_MODE;
    millisRW = 42;
    updateAltHold(currentTimeUs);
    EXPECT_EQ(isAltHoldActive(), true);
}

TEST(AltholdUnittest, altHoldCapturesHoverThrottleWhenConfigZero)
{
    altHoldInit();
    autopilotConfigMutable()->hoverThrottle = 0;
    rcCommand[THROTTLE] = 1450.0f;

    flightModeFlags |= ALT_HOLD_MODE;
    updateAltHold(currentTimeUs);
    EXPECT_EQ(autopilotGetEffectiveHoverThrottlePwm(), 1450);

    flightModeFlags &= ~ALT_HOLD_MODE;
    updateAltHold(currentTimeUs);
    EXPECT_EQ(autopilotGetEffectiveHoverThrottlePwm(), AP_HOVER_THROTTLE_DEFAULT);
}

TEST_F(AltholdControlUnittest, AltitudeControlRaisesThrottleWhenBelowTarget)
{
    testAltitudeCm = 0.0f;
    testAltitudeDerivativeCmS = 0.0f;
    altitudeControl(100.0f, 0.01f, 0.0f, 500.0f);
    const float belowTargetThrottle = getAutopilotThrottle();

    resetAltitudeControl();
    testAltitudeCm = 200.0f;
    testAltitudeDerivativeCmS = 0.0f;
    altitudeControl(100.0f, 0.01f, 0.0f, 500.0f);
    const float aboveTargetThrottle = getAutopilotThrottle();

    EXPECT_GT(belowTargetThrottle, aboveTargetThrottle);
}

TEST_F(AltholdControlUnittest, AltitudeControlRespectsVelocityLimit)
{
    testAltitudeCm = 0.0f;
    testAltitudeDerivativeCmS = 0.0f;

    altitudeControl(200.0f, 0.01f, 0.0f, 100.0f);
    const float limitedThrottle = getAutopilotThrottle();

    resetAltitudeControl();
    altitudeControl(800.0f, 0.01f, 0.0f, 1000.0f);
    const float highLimitThrottle = getAutopilotThrottle();

    EXPECT_GT(highLimitThrottle, limitedThrottle);
}

// Regression: the flight-plan landing target sits FP_LANDING_TARGET_DEPTH_M (200m) below the
// craft so descent continues until touchdown is detected. That error must not drive the
// position P/I terms into output saturation - the descent rate is owned by the velocity terms.
// Before the fix this pinned the output at throttleMin and the craft free-fell.
TEST_F(AltholdControlUnittest, AltitudeControlLandingTargetDoesNotSaturateThrottle)
{
    testAltitudeCm = 1110.0f;             // 11.1m up, as in the reported crash
    testAltitudeDerivativeCmS = 0.0f;
    const float landingTargetCm = testAltitudeCm - 20000.0f;  // 200m below the craft
    const float descendRateCmS = 120.0f;

    // hold the target for a few seconds of task iterations so any iTerm windup would show up
    for (int i = 0; i < 500; i++) {
        altitudeControl(landingTargetCm, 0.01f, -descendRateCmS, descendRateCmS);
    }

    const float throttle = getAutopilotThrottle();
    const float throttleMin = autopilotConfig()->throttleMin;
    const float throttleRange = autopilotConfig()->throttleMax - throttleMin;
    const float throttlePwm = throttleMin + throttle * throttleRange;

    // must not be pinned at the floor - that is free-fall
    EXPECT_GT(throttlePwm, throttleMin + 1.0f);
    // and must stay below hover, i.e. it is still commanding a descent
    EXPECT_LT(throttlePwm, (float)autopilotConfig()->hoverThrottle);
}

// End-to-end: a rescue/mission LAND leg publishes an endpoint FP_LANDING_TARGET_DEPTH_M (200m)
// below the craft, with positionNav rate-limiting its velocity output to the configured descent
// rate. Alt-hold must follow a carrot at that rate, not the raw endpoint. Before the fix the
// endpoint went straight into the altitude PID and pinned throttle at throttleMin - a free-fall.
TEST_F(AltholdControlUnittest, NavLandingTargetTracksCommandedDescentRate)
{
    const float descendRateCmS = 120.0f;
    altHoldConfigMutable()->climbRate = 12;      // CLI units: 12 -> 120cm/s
    altHoldInit();
    debugMode = DEBUG_AUTOPILOT_ALTITUDE;        // debug[2] == the targetAltitudeCm actually used

    testAltitudeCm = 1110.0f;                    // 11.1m, as in the reported crash
    flightModeFlags = ALT_HOLD_MODE;
    updateAltHold(0);                            // engage, anchoring the target at 11.1m

    testNavActive = true;
    testNavReached = false;
    testNavCmd.includeAltitude = true;
    testNavCmd.targetPosEfM.z = (testAltitudeCm - 20000.0f) * 0.01f;  // 200m below, in metres
    testNavTargetVelZCmS = -descendRateCmS;

    const float throttleMin = autopilotConfig()->throttleMin;
    const float dt = 0.01f;
    for (int i = 0; i < 200; i++) {              // 2s, craft tracking the commanded descent
        testAltitudeDerivativeCmS = -descendRateCmS;
        testAltitudeCm += -descendRateCmS * dt;
        updateAltHold(0);

        const float throttlePwm = throttleMin + getAutopilotThrottle()
                                * (autopilotConfig()->throttleMax - throttleMin);
        ASSERT_GT(throttlePwm, throttleMin + 1.0f) << "throttle pinned at floor on iteration " << i;
    }

    // the commanded target must have marched down with the craft, never jumped to the endpoint
    EXPECT_NEAR((float)debug[2], testAltitudeCm, descendRateCmS * 1.5f);
    EXPECT_GT(debug[2], -1000);                  // nowhere near -18890
}

// Closed-loop landing simulation against the real altitudeControl(), with a crude but honest
// physics model: thrust proportional to throttle above idle, normalised so hoverThrottle holds
// altitude, plus ground contact and ground effect. This exists to test the bounce mechanism
// quantitatively rather than by assertion - the craft can only leave the ground here if the
// controller commands enough thrust for ground effect to lift it.
namespace {
struct LandingSim {
    float altCm = 0.0f;
    float vzCmS = 0.0f;
    static constexpr float kGroundEffectHeightCm = 30.0f;
    static constexpr float kGroundEffectGain = 0.25f;   // +25% thrust on the deck

    void step(float throttlePwm, float hoverPwm, float minPwm, float dt) {
        const float span = MAX(hoverPwm - minPwm, 1.0f);
        float thrustRatio = (throttlePwm - minPwm) / span;   // 1.0 == weight
        if (altCm < kGroundEffectHeightCm) {
            thrustRatio *= 1.0f + kGroundEffectGain * (1.0f - altCm / kGroundEffectHeightCm);
        }
        const float accelCmS2 = 981.0f * (thrustRatio - 1.0f);
        vzCmS += accelCmS2 * dt;
        altCm += vzCmS * dt;
        if (altCm <= 0.0f) {                 // inelastic ground contact
            altCm = 0.0f;
            if (vzCmS < 0.0f) { vzCmS = 0.0f; }
        }
    }
};
}

TEST_F(AltholdControlUnittest, LandingSimSettlesOnGroundInsteadOfBouncing)
{
    const float descendRateCmS = 120.0f;
    const float simHoverPwm = 1300.0f;          // what the airframe actually needs
    autopilotConfigMutable()->hoverThrottle = 1250;   // configured slightly low, so iTerm trims up
    autopilotConfigMutable()->throttleMin = 1100;
    autopilotConfigMutable()->throttleMax = 1900;
    altHoldConfigMutable()->climbRate = 12;     // 120 cm/s
    autopilotInit();
    altHoldInit();
    autopilotSetLandingSettle(false);

    LandingSim sim;
    sim.altCm = 800.0f;
    testAltitudeCm = sim.altCm;
    flightModeFlags = ALT_HOLD_MODE;
    testThrottleStatus = THROTTLE_LOW;         // no pilot stick input
    const float dt = 0.01f;
    const float minPwm = (float)autopilotConfig()->throttleMin;

    auto runStep = [&]() {
        testAltitudeCm = sim.altCm;
        testAltitudeDerivativeCmS = sim.vzCmS;
        updateAltHold(0);
        const float throttlePwm = minPwm + getAutopilotThrottle()
                                * (autopilotConfig()->throttleMax - minPwm);
        sim.step(throttlePwm, simHoverPwm, 1000.0f, dt);
        return throttlePwm;
    };

    // Hover first, so iTerm trims out the hover-throttle error the way it would in flight.
    // This is what makes the bounce possible: iTerm carries that positive trim into the landing.
    for (int i = 0; i < 800; i++) { runStep(); }

    // Now the LAND leg: nav publishes a bounded target below the craft and a rate-limited demand.
    testNavActive = true;
    testNavReached = false;
    testNavCmd.includeAltitude = true;
    testNavTargetVelZCmS = -descendRateCmS;

    int liftoffsAfterContact = 0;
    bool everTouched = false;
    float stallS = 0.0f;
    for (int i = 0; i < 2000; i++) {
        testNavCmd.targetPosEfM.z = (sim.altCm - 500.0f) * 0.01f;  // FP_LANDING_TARGET_DEPTH_M
        stallS = (sim.vzCmS > -0.25f * descendRateCmS) ? stallS + dt : 0.0f;
        autopilotSetLandingSettle(stallS > 0.8f);
        runStep();
        if (sim.altCm <= 0.01f) { everTouched = true; }
        else if (everTouched && sim.altCm > 5.0f) { liftoffsAfterContact++; }
    }

    EXPECT_TRUE(everTouched) << "never reached the ground";
    EXPECT_LT(liftoffsAfterContact, 20) << "craft kept leaving the ground after touchdown";
    EXPECT_LT(sim.altCm, 5.0f) << "did not settle on the ground";
    EXPECT_NEAR(sim.vzCmS, 0.0f, 25.0f) << "still moving vertically at the end";
}

// Regression: while the landing settle ceiling holds the output at throttleMin the craft is on
// the ground, but the loop still commands a descent, so the altitude error stays negative. If
// saturation is judged against throttleMax rather than against the ceiling actually in force,
// the output never reads as saturated low and iTerm integrates down against a floor it is
// already sitting on. Clearing the settle then releases that iTerm as a thrust dip at ground
// level, which is the same free-fall this PR exists to prevent.
TEST_F(AltholdControlUnittest, LandingSettleDoesNotWindIntegralDownAgainstTheCeiling)
{
    const float simHoverPwm = 1300.0f;
    autopilotConfigMutable()->hoverThrottle = 1250;
    autopilotConfigMutable()->throttleMin = 1100;
    autopilotConfigMutable()->throttleMax = 1900;
    altHoldConfigMutable()->climbRate = 12;
    autopilotInit();
    altHoldInit();
    autopilotSetLandingSettle(false);
    debugMode = DEBUG_AUTOPILOT_ALTITUDE;   // debug[5] is altitudeI, debug[0] the output PWM

    LandingSim sim;
    sim.altCm = 400.0f;
    testAltitudeCm = sim.altCm;
    flightModeFlags = ALT_HOLD_MODE;
    testThrottleStatus = THROTTLE_LOW;
    const float dt = 0.01f;
    const float minPwm = (float)autopilotConfig()->throttleMin;

    auto runStep = [&]() {
        testAltitudeCm = sim.altCm;
        testAltitudeDerivativeCmS = sim.vzCmS;
        updateAltHold(0);
        const float throttlePwm = minPwm + getAutopilotThrottle()
                                * (autopilotConfig()->throttleMax - minPwm);
        sim.step(throttlePwm, simHoverPwm, 1000.0f, dt);
    };

    // Hover, so iTerm trims up to cover the gap between configured and real hover throttle.
    for (int i = 0; i < 400; i++) { runStep(); }
    const int iTermBeforeSettle = debug[5];
    ASSERT_GT(iTermBeforeSettle, 0) << "iTerm never trimmed up, test cannot prove anything";

    // Land, and hold the craft pinned on the ground with the settle ceiling engaged. The nav
    // target stays below the craft throughout, so the error stays negative the whole time.
    testNavActive = true;
    testNavReached = false;
    testNavCmd.includeAltitude = true;
    testNavTargetVelZCmS = -120.0f;
    autopilotSetLandingSettle(true);
    for (int i = 0; i < 1500; i++) {
        sim.altCm = 0.0f;                          // on the ground and staying there
        sim.vzCmS = 0.0f;
        testNavCmd.targetPosEfM.z = -5.0f;         // FP_LANDING_TARGET_DEPTH_M below ground
        runStep();
    }

    ASSERT_LE(debug[0], (int)minPwm + 1) << "ceiling never bled to throttleMin";
    EXPECT_GT(debug[5], iTermBeforeSettle - 20)
        << "iTerm fell from " << iTermBeforeSettle << " to " << debug[5]
        << " while the output was pinned against the settle ceiling; it should be held, not "
        << "wound down into a thrust dip that is released when the settle clears";
}

// Regression: the settle ceiling is altitude-loop state, so resetAltitudeControl() must clear it.
// flight_plan_nav clears it on its own lifecycle boundaries, but alt hold and GPS rescue reset
// the loop independently. Without this, re-engaging after an abandoned landing inherits a stale
// ceiling and the craft is pinned at throttleMin while trying to hold or regain altitude.
TEST_F(AltholdControlUnittest, ResetAltitudeControlClearsLandingSettle)
{
    autopilotConfigMutable()->hoverThrottle = 1300;
    autopilotConfigMutable()->throttleMin = 1100;
    autopilotConfigMutable()->throttleMax = 1900;
    autopilotInit();
    altHoldInit();
    debugMode = DEBUG_AUTOPILOT_ALTITUDE;   // debug[0] is the output PWM
    const int minPwm = autopilotConfig()->throttleMin;

    // Drive the ceiling all the way down, as a landing that reaches the ground would.
    autopilotSetLandingSettle(true);
    testAltitudeCm = 0.0f;
    testAltitudeDerivativeCmS = 0.0f;
    for (int i = 0; i < 500; i++) {
        altitudeControl(-500.0f, 0.01f, -120.0f, 0.0f);
    }
    ASSERT_LE(debug[0], minPwm + 1) << "ceiling never bled down, test cannot prove anything";

    // Abandon the landing without going through flight_plan_nav's disengage, then re-engage and
    // ask for a climb. A stale ceiling would hold the output at throttleMin regardless of demand.
    resetAltitudeControl();
    for (int i = 0; i < 20; i++) {
        altitudeControl(500.0f, 0.01f, 100.0f, 0.0f);
    }

    EXPECT_GT(debug[0], minPwm + 100)
        << "output stuck at " << debug[0] << " with a 5m climb demanded; the landing throttle "
        << "ceiling survived resetAltitudeControl()";
}

// Regression: a GPS rescue runs under failsafe, and altHoldUpdateTargetAltitude()'s failsafe
// branch drives altHold.targetAltitudeCm down at up to 10x the climb rate. That variable is
// also the nav carrot's accumulator, so if both run the descent term cancels the mission's
// climb and the craft sinks where it stands instead of climbing to return altitude.
TEST_F(AltholdControlUnittest, NavClimbNotCancelledByFailsafeDescentTerm)
{
    altHoldConfigMutable()->climbRate = 10;      // 100 cm/s, as gps_rescue_ascend_rate would give
    altHoldConfigMutable()->deadband = 10;
    autopilotInit();
    altHoldInit();

    testAltitudeCm = 1000.0f;                    // 10m up, rescue just engaged
    flightModeFlags = ALT_HOLD_MODE;
    testThrottleStatus = THROTTLE_LOW;
    updateAltHold(0);

    testFailsafeActive = true;                   // rescue is running under failsafe
    testNavActive = true;
    testNavReached = false;
    testNavCmd.includeAltitude = true;
    testNavCmd.targetPosEfM.z = 30.0f;           // climb to 30m return altitude
    testNavTargetVelZCmS = 100.0f;               // positionNav publishes a climb

    debugMode = DEBUG_AUTOPILOT_ALTITUDE;
    for (int i = 0; i < 100; i++) {              // 1s of task iterations
        updateAltHold(0);
    }
    testFailsafeActive = false;

    // debug[2] is the target actually handed to altitudeControl
    EXPECT_GT(debug[2], (int)testAltitudeCm)
        << "commanded altitude target must be above the craft during a rescue climb";
}

// Regression: with a target far below the craft, a stale positive iTerm must be able to unwind.
// A blanket anti-windup freeze stranded it, held throttle above hover, and the craft climbed
// away from a target 29m below - seen in the SITL altitude-only fallback descent.
// Regression from the SITL altitude-only fallback descent: the craft climbed away from a target
// 29m below it. A blanket anti-windup freeze stranded a stale positive iTerm (+97) which, with
// the position error clamped so P had only -12 of authority, held throttle above hover forever.
// iTerm must always be free to unwind, only inhibited in the direction that grows it.
TEST_F(AltholdControlUnittest, StaleITermUnwindsWhenTargetIsFarBelow)
{
    // stock gains - the fixture's are strong enough to mask this
    autopilotConfigMutable()->altitudeP = 30;
    autopilotConfigMutable()->altitudeI = 30;
    autopilotConfigMutable()->altitudeD = 30;
    autopilotConfigMutable()->altitudeF = 30;
    autopilotConfigMutable()->hoverThrottle = 1275;
    autopilotConfigMutable()->throttleMin = 1100;
    autopilotConfigMutable()->throttleMax = 1900;
    autopilotInit();
    resetAltitudeControl();

    testAltitudeCm = 6000.0f;                 // 60m up
    testAltitudeDerivativeCmS = 0.0f;

    // Build the stale trim the way flight does: a small persistent error inside the rate limit.
    // A large error is itself rate limited and would never integrate.
    for (int i = 0; i < 3000; i++) {
        altitudeControl(testAltitudeCm + 79.0f, 0.01f, 80.0f, 80.0f);
    }

    // Target jumps far below, as the fallback descent commands.
    for (int i = 0; i < 6000; i++) {
        altitudeControl(testAltitudeCm - 2900.0f, 0.01f, -80.0f, 80.0f);
    }

    const float throttleMin = autopilotConfig()->throttleMin;
    const float throttlePwm = throttleMin + getAutopilotThrottle()
                            * (autopilotConfig()->throttleMax - throttleMin);
    EXPECT_LT(throttlePwm, (float)autopilotConfig()->hoverThrottle)
        << "must command descent, not hold above hover on a stale iTerm";
}

// Replay of the reported crash configuration (SEQUREH7V2, 2026.12.0-alpha): a GPS rescue LAND
// leg entered at 11.1m. On the original firmware the 200m-deep landing endpoint drove the
// altitude P term to -3000, pinned throttle at ap_throttle_min and the craft fell ~10m in 1.75s
// at up to -9.1 m/s and was destroyed. Same settings here must produce a controlled descent.
TEST_F(AltholdControlUnittest, ReportedCrashConfigDescendsUnderControl)
{
    const float descendRateCmS = 120.0f;      // gps_rescue_descend_rate = 120
    autopilotConfigMutable()->hoverThrottle = 1250;   // ap_hover_throttle = 1250
    autopilotConfigMutable()->throttleMin = 1100;
    autopilotConfigMutable()->throttleMax = 1900;
    autopilotConfigMutable()->altitudeP = 30;         // all ap_altitude_* stock
    autopilotConfigMutable()->altitudeI = 30;
    autopilotConfigMutable()->altitudeD = 30;
    autopilotConfigMutable()->altitudeF = 30;
    altHoldConfigMutable()->climbRate = 12;           // 120 cm/s vertical budget
    autopilotInit();
    altHoldInit();
    autopilotSetLandingSettle(false);
    debugMode = DEBUG_AUTOPILOT_ALTITUDE;

    LandingSim sim;
    sim.altCm = 1110.0f;                      // 11.1m, the logged altitude at LAND dispatch
    testAltitudeCm = sim.altCm;
    flightModeFlags = ALT_HOLD_MODE;
    testThrottleStatus = THROTTLE_LOW;
    updateAltHold(0);

    testNavActive = true;
    testNavReached = false;
    testNavCmd.includeAltitude = true;
    testNavTargetVelZCmS = -descendRateCmS;

    const float dt = 0.01f;
    const float minPwm = (float)autopilotConfig()->throttleMin;
    float worstVzCmS = 0.0f;
    int flooredSteps = 0;

    for (int i = 0; i < 1200 && sim.altCm > 0.0f; i++) {
        // nav publishes the endpoint 200m below, exactly as startLanding did originally
        testNavCmd.targetPosEfM.z = (sim.altCm - 20000.0f) * 0.01f;
        testAltitudeCm = sim.altCm;
        testAltitudeDerivativeCmS = sim.vzCmS;
        updateAltHold(0);

        const float throttlePwm = minPwm + getAutopilotThrottle()
                                * (autopilotConfig()->throttleMax - minPwm);
        if (throttlePwm <= minPwm + 1.0f) { flooredSteps++; }
        sim.step(throttlePwm, 1250.0f, 1000.0f, dt);
        worstVzCmS = std::min(worstVzCmS, sim.vzCmS);
    }

    EXPECT_LT(flooredSteps, 20) << "throttle must not sit on the floor - that is the free-fall";
    EXPECT_GT(worstVzCmS, -400.0f) << "peak sink " << worstVzCmS
        << " cm/s; the crash reached -910 cm/s";
    EXPECT_GT(debug[2], -1000) << "commanded target must not be the 200m endpoint";
}

TEST_F(AltholdControlUnittest, AltitudeControlCompensatesForTilt)
{
    testAltitudeCm = 100.0f;
    testAltitudeDerivativeCmS = 0.0f;

    testCosTiltAngle = 1.0f;
    altitudeControl(100.0f, 0.01f, 0.0f, 500.0f);
    const float levelThrottle = getAutopilotThrottle();

    resetAltitudeControl();
    testCosTiltAngle = 0.5f;
    altitudeControl(100.0f, 0.01f, 0.0f, 500.0f);
    const float tiltedThrottle = getAutopilotThrottle();

    EXPECT_GT(tiltedThrottle, levelThrottle);
}

// STUBS

extern "C" {
    uint8_t armingFlags = 0;
    int16_t debug[DEBUG16_VALUE_COUNT];
    uint8_t debugMode;
    uint16_t flightModeFlags = 0;
    uint8_t stateFlags = 0;

    acc_t acc;
    attitudeEulerAngles_t attitude;
    gpsSolutionData_t gpsSol;
    gyro_t gyro;

    float testAltitudeCm = 0.0f;
    float testAltitudeDerivativeCmS = 0.0f;
    float testCosTiltAngle = 1.0f;
    throttleStatus_e testThrottleStatus = THROTTLE_LOW;

    float getAltitudeCm(void) { return testAltitudeCm; }
    float getAltitudeDerivative(void) { return testAltitudeDerivativeCmS; }
    float getAltitudeCmControl(void) { return testAltitudeCm; }
    float getAltitudeDerivativeControl(void) { return testAltitudeDerivativeCmS; }
    float getCosTiltAngle(void) { return testCosTiltAngle; }
    float getGpsDataIntervalSeconds(void) { return 0.01f; }

    float rcCommand[4];

    bool gpsHasNewData(uint16_t* gpsStamp) {
        UNUSED(*gpsStamp);
        return true;
    }
    float getSetpointRate(int axis)
    {
        UNUSED(axis);
        return 0.0f;
    }

    float getMaxRcRate(int axis)
    {
        UNUSED(axis);
        return 720.0f; // nonzero: autopilotInit divides maxVelocity by this
    }

    void GPS_distance2d(const gpsLocation_t* /*from*/, const gpsLocation_t* /*to*/, vector2_t* /*dest*/) { }

    static positionEstimate3d_t stubEstimate = {};

    const positionEstimate3d_t *positionEstimatorGetEstimate(void) { return &stubEstimate; }
    void positionEstimatorEnableXY(bool /*enable*/) { }
    bool positionEstimatorIsValidXY(void) { return false; }

    void positionNavInit(void) { }
    void positionNavReset(void) { }
    void positionNavUpdate(float /*dt*/, const positionEstimate3d_t * /*est*/) { }
    bool testNavActive = false;
    bool testNavReached = false;
    float testNavTargetVelZCmS = 0.0f;
    positionNavCommand_t testNavCmd;
    bool positionNavHasActiveTarget(void) { return testNavActive; }
    bool positionNavTargetReached(void) { return testNavReached; }
    vector3_t positionNavGetTargetVelocityCmS(void) { return (vector3_t){{0, 0, testNavTargetVelZCmS}}; }
    const positionNavCommand_t *positionNavGetActiveCommand(void) { return &testNavCmd; }

    void parseRcChannels(const char *input, rxConfig_t *rxConfig) {
        UNUSED(input);
        UNUSED(rxConfig);
    }

    timeDelta_t getTaskDeltaTimeUs(taskId_e taskId)
    {
        UNUSED(taskId);
        return TASK_PERIOD_HZ(100); // default poshold rate in tests
    }

    throttleStatus_e calculateThrottleStatus() { return testThrottleStatus; }
}
