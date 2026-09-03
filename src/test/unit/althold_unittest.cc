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
    extern float testAltitudeAccelerationCmS;
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

// getAutopilotThrottle() normalises over [MAX(mincheck, PWM_RANGE_MIN), PWM_RANGE_MAX], not over
// [throttleMin, throttleMax]. Inverting with the wrong range shifts the result by tens of PWM at
// the floor, which is exactly where the floored-throttle assertions look.
static float autopilotThrottlePwm(void)
{
    const float lowPwm = MAX((float)rxConfig()->mincheck, (float)PWM_RANGE_MIN);
    return lowPwm + getAutopilotThrottle() * ((float)PWM_RANGE_MAX - lowPwm);
}

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
        testAltitudeAccelerationCmS = 0.0f;
        testCosTiltAngle = 1.0f;
        testThrottleStatus = THROTTLE_LOW;
        testFailsafeActive = false;
        testNavActive = false;
        testNavReached = false;
        testNavTargetVelZCmS = 0.0f;
        memset(&testNavCmd, 0, sizeof(testNavCmd));

        autopilotConfig_t *apCfg = autopilotConfigMutable();
        apCfg->hoverThrottle = 1500;
        apCfg->throttleMin = 1000;
        apCfg->throttleMax = 2000;
        apCfg->altitudeP = 50;
        apCfg->altitudeI = 50;
        apCfg->altitudeD = 50;
        apCfg->altitudeA = 50;
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
    testAltitudeAccelerationCmS = 0.0f;
    altitudeControl(100.0f, 0.01f, 0.0f, 500.0f);
    const float belowTargetThrottle = getAutopilotThrottle();

    resetAltitudeControl();
    testAltitudeCm = 200.0f;
    testAltitudeDerivativeCmS = 0.0f;
    testAltitudeAccelerationCmS = 0.0f;
    altitudeControl(100.0f, 0.01f, 0.0f, 500.0f);
    const float aboveTargetThrottle = getAutopilotThrottle();

    EXPECT_GT(belowTargetThrottle, aboveTargetThrottle);
}

// The A term opposes measured vertical acceleration: altitudeA = -acceleration * altitudeKa,
// so climbing acceleration must take throttle off. Getting that sign wrong would turn it into
// positive feedback on vertical acceleration, and no other test would notice - every other
// case in this file holds testAltitudeAccelerationCmS at zero.
//
// Altitude and vertical velocity are held exactly at target so P, I, D and F all contribute
// nothing and the A term is the only thing that differs between the two scenarios.
TEST_F(AltholdControlUnittest, AltitudeControlLowersThrottleWhenAcceleratingUpward)
{
    testAltitudeCm = 100.0f;
    testAltitudeDerivativeCmS = 0.0f;
    testAltitudeAccelerationCmS = 0.0f;
    altitudeControl(100.0f, 0.01f, 0.0f, 500.0f);
    const float noAccelerationThrottle = getAutopilotThrottle();

    resetAltitudeControl();
    testAltitudeCm = 100.0f;
    testAltitudeDerivativeCmS = 0.0f;
    testAltitudeAccelerationCmS = 200.0f; // accelerating upward
    altitudeControl(100.0f, 0.01f, 0.0f, 500.0f);
    const float upwardAccelerationThrottle = getAutopilotThrottle();

    // altitudeKa is 50 * 0.006 = 0.3, so this is roughly 60 PWM of opposition.
    EXPECT_LT(upwardAccelerationThrottle, noAccelerationThrottle);
}

TEST_F(AltholdControlUnittest, AltitudeControlRespectsVelocityLimit)
{
    testAltitudeCm = 0.0f;
    testAltitudeDerivativeCmS = 0.0f;
    testAltitudeAccelerationCmS = 0.0f;
    altitudeControl(200.0f, 0.01f, 0.0f, 100.0f);
    const float limitedThrottle = getAutopilotThrottle();

    resetAltitudeControl();
    altitudeControl(800.0f, 0.01f, 0.0f, 1000.0f);
    const float highLimitThrottle = getAutopilotThrottle();

    EXPECT_GT(highLimitThrottle, limitedThrottle);
}

// A LAND leg publishes an endpoint below the craft that is never reached, with positionNav
// rate-limiting the velocity it publishes to the configured descent rate. Alt hold must follow a
// carrot at that rate, not the raw endpoint, which went straight into the altitude PID and pinned
// throttle at throttleMin - the free-fall. The endpoint here is the 200 m one the firmware used
// to publish: the carrot must bound an arbitrarily distant endpoint, not just the current 5 m.
TEST_F(AltholdControlUnittest, NavLandingTargetTracksCommandedDescentRate)
{
    const float descendRateCmS = 120.0f;
    altHoldConfigMutable()->climbRate = 12;      // CLI units: 12 -> 120cm/s
    altHoldInit();
    debugMode = DEBUG_AUTOPILOT_ALTITUDE;        // debug[1] == the targetAltitudeCm actually used

    testAltitudeCm = 1110.0f;                    // 11.1m, as in the reported crash
    flightModeFlags = ALT_HOLD_MODE;
    updateAltHold(0);                            // engage, anchoring the target at 11.1m

    testNavActive = true;
    testNavReached = false;
    testNavCmd.includeAltitude = true;
    testNavCmd.targetPosEfM.z = (testAltitudeCm - 20000.0f) * 0.01f;  // metres
    testNavTargetVelZCmS = -descendRateCmS;

    const float throttleMin = autopilotConfig()->throttleMin;
    const float dt = 0.01f;
    for (int i = 0; i < 200; i++) {              // 2s, craft tracking the commanded descent
        testAltitudeDerivativeCmS = -descendRateCmS;
        testAltitudeCm += -descendRateCmS * dt;
        updateAltHold(0);

        const float throttlePwm = autopilotThrottlePwm();
        ASSERT_GT(throttlePwm, throttleMin + 1.0f) << "throttle pinned at floor on iteration " << i;
    }

    // the commanded target must have marched down with the craft, never jumped to the endpoint
    EXPECT_NEAR((float)debug[1], testAltitudeCm, descendRateCmS * 1.5f);
    EXPECT_GT(debug[1], -1000);                  // nowhere near -18890
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

    float accelCmS2 = 0.0f;              // last step's vertical acceleration, for the A term

    void step(float throttlePwm, float hoverPwm, float minPwm, float dt) {
        const float span = MAX(hoverPwm - minPwm, 1.0f);
        float thrustRatio = (throttlePwm - minPwm) / span;   // 1.0 == weight
        if (altCm < kGroundEffectHeightCm) {
            thrustRatio *= 1.0f + kGroundEffectGain * (1.0f - altCm / kGroundEffectHeightCm);
        }
        accelCmS2 = 981.0f * (thrustRatio - 1.0f);
        vzCmS += accelCmS2 * dt;
        altCm += vzCmS * dt;
        if (altCm <= 0.0f) {                 // inelastic ground contact
            altCm = 0.0f;
            if (vzCmS < 0.0f) { vzCmS = 0.0f; }
        }
    }
};
}

// ap_landing_descent_rate reaches 200 cm/s while alt_hold_climb_rate can be set below it. Sized
// from the smaller climb rate, the window silently reduces the leg's configured descent. Holding
// the craft still lets the carrot run out to the window, whose size is the limit in force.
TEST_F(AltholdControlUnittest, LandingDescentRateIsNotCappedByTheClimbRate)
{
    const float descendRateCmS = 200.0f;         // ap_landing_descent_rate = 200
    altHoldConfigMutable()->climbRate = 10;      // 100 cm/s: below the descent rate
    altHoldInit();
    debugMode = DEBUG_AUTOPILOT_ALTITUDE;

    testAltitudeCm = 1000.0f;
    flightModeFlags = ALT_HOLD_MODE;
    updateAltHold(0);

    testNavActive = true;
    testNavReached = false;
    testNavCmd.includeAltitude = true;
    testNavCmd.vertSpeedLimitMps = descendRateCmS * 0.01f;
    testNavCmd.targetPosEfM.z = (testAltitudeCm - 5000.0f) * 0.01f;   // endpoint well clear
    testNavTargetVelZCmS = -descendRateCmS;

    for (int i = 0; i < 200; i++) {              // craft held still: the carrot runs to the window
        testAltitudeDerivativeCmS = 0.0f;
        updateAltHold(0);
    }

    const float trailCm = testAltitudeCm - (float)debug[1];
    EXPECT_NEAR(trailCm, descendRateCmS, 10.0f) << "carrot held " << trailCm
        << " cm below the craft; one second of the LAND rate is " << descendRateCmS;
}

// The same stall, reached the other way: positionNavSetTargetEf() resets vertSpeedLimitMps to 0,
// and altitude-carrying waypoint legs never set a replacement. With alt_hold_climb_rate also 0
// there is no rate anywhere, and a window sized from it freezes the carrot on the craft.
TEST_F(AltholdControlUnittest, NavLegWithNoVerticalLimitStillMovesAtZeroClimbRate)
{
    altHoldConfigMutable()->climbRate = 0;
    altHoldInit();
    debugMode = DEBUG_AUTOPILOT_ALTITUDE;

    testAltitudeCm = 1000.0f;
    flightModeFlags = ALT_HOLD_MODE;
    updateAltHold(0);

    testNavActive = true;
    testNavReached = false;
    testNavCmd.includeAltitude = true;
    testNavCmd.vertSpeedLimitMps = 0.0f;          // as positionNavSetTargetEf leaves it
    testNavCmd.cruiseSpeedMps = 5.0f;
    testNavCmd.targetPosEfM.z = (testAltitudeCm + 3000.0f) * 0.01f;
    testNavTargetVelZCmS = 200.0f;

    const float startAltitudeCm = testAltitudeCm;
    for (int i = 0; i < 50; i++) {                // craft held still: only the target may move
        testAltitudeDerivativeCmS = 0.0f;
        updateAltHold(0);
    }

    EXPECT_GT((float)debug[1], startAltitudeCm + 20.0f)
        << "commanded target " << debug[1] << " cm never climbed from " << startAltitudeCm;
}

// alt_hold_climb_rate is allowed to be 0. The one-second window is derived from it, so a nav
// altitude leg must take its rate from the leg instead - otherwise the window collapses onto the
// craft, the target can never move, and the descent never starts.
TEST_F(AltholdControlUnittest, ZeroClimbRateDoesNotStallANavDescent)
{
    const float descendRateCmS = 100.0f;
    altHoldConfigMutable()->climbRate = 0;
    altHoldInit();
    debugMode = DEBUG_AUTOPILOT_ALTITUDE;

    testAltitudeCm = 1000.0f;
    flightModeFlags = ALT_HOLD_MODE;
    updateAltHold(0);

    testNavActive = true;
    testNavReached = false;
    testNavCmd.includeAltitude = true;
    testNavCmd.vertSpeedLimitMps = descendRateCmS * 0.01f;
    testNavCmd.targetPosEfM.z = (testAltitudeCm - 500.0f) * 0.01f;
    testNavTargetVelZCmS = -descendRateCmS;

    const float startAltitudeCm = testAltitudeCm;
    for (int i = 0; i < 50; i++) {               // craft held still: only the target may move
        testAltitudeDerivativeCmS = 0.0f;
        updateAltHold(0);
    }

    EXPECT_LT((float)debug[1], startAltitudeCm - 20.0f)
        << "commanded target " << debug[1] << " cm never descended from " << startAltitudeCm;
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

    // debug[1] is the target actually handed to altitudeControl
    EXPECT_GT(debug[1], (int)testAltitudeCm)
        << "commanded altitude target must be above the craft during a rescue climb";
}

// Replay of the reported crash configuration (SEQUREH7V2, 2026.12.0-alpha): a GPS rescue LAND
// leg entered at 11.1 m. The landing endpoint 200 m down drove the altitude P term to -3000,
// pinned throttle at ap_throttle_min, and the craft fell ~10 m in 1.75 s at up to -9.1 m/s and was
// destroyed. The same settings, against the same endpoint, must produce a controlled descent.
TEST_F(AltholdControlUnittest, ReportedCrashConfigDescendsUnderControl)
{
    const float descendRateCmS = 120.0f;      // gps_rescue_descend_rate = 120
    autopilotConfigMutable()->hoverThrottle = 1250;   // ap_hover_throttle = 1250
    autopilotConfigMutable()->throttleMin = 1100;
    autopilotConfigMutable()->throttleMax = 1900;
    autopilotConfigMutable()->altitudeP = 30;         // all ap_altitude_* stock
    autopilotConfigMutable()->altitudeI = 30;
    autopilotConfigMutable()->altitudeD = 30;
    autopilotConfigMutable()->altitudeA = 30;
    autopilotConfigMutable()->altitudeF = 30;
    altHoldConfigMutable()->climbRate = 12;           // 120 cm/s vertical budget
    autopilotInit();
    altHoldInit();
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
        // the endpoint startLanding published at the time of the crash
        testNavCmd.targetPosEfM.z = (sim.altCm - 20000.0f) * 0.01f;
        testAltitudeCm = sim.altCm;
        testAltitudeDerivativeCmS = sim.vzCmS;
        testAltitudeAccelerationCmS = sim.accelCmS2;   // altitudeA is live; leaving it 0 mutes it
        updateAltHold(0);

        const float throttlePwm = autopilotThrottlePwm();
        if (throttlePwm <= minPwm + 1.0f) { flooredSteps++; }
        sim.step(throttlePwm, 1250.0f, 1000.0f, dt);
        worstVzCmS = std::min(worstVzCmS, sim.vzCmS);
    }

    EXPECT_LT(flooredSteps, 20) << "throttle must not sit on the floor - that is the free-fall";
    EXPECT_GT(worstVzCmS, -400.0f) << "peak sink " << worstVzCmS
        << " cm/s; the crash reached -910 cm/s";
    EXPECT_GT(debug[1], -1000) << "commanded target must not be the raw endpoint";
}

TEST_F(AltholdControlUnittest, AltitudeControlCompensatesForTilt)
{
    testAltitudeCm = 100.0f;
    testAltitudeDerivativeCmS = 0.0f;
    testAltitudeAccelerationCmS = 0.0f;
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
    float testAltitudeAccelerationCmS = 0.0f;
    float testCosTiltAngle = 1.0f;
    throttleStatus_e testThrottleStatus = THROTTLE_LOW;

    float getAltitudeCm(void) { return testAltitudeCm; }
    float getAltitudeDerivative(void) { return testAltitudeDerivativeCmS; }
    float getAltitudeCmControl(void) { return testAltitudeCm; }
    float getAltitudeAccelerationControl(void) { return testAltitudeAccelerationCmS; }
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
