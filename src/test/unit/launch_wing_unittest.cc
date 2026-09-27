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
#include <cmath>

extern "C" {
    #include "platform.h"

    #include "build/debug.h"
    #include "common/axis.h"
    #include "common/maths.h"
    #include "common/time.h"
    #include "fc/rc.h"
    #include "fc/rc_controls.h"
    #include "fc/rc_modes.h"
    #include "fc/runtime_config.h"
    #include "flight/autopilot.h"
    #include "flight/imu.h"
    #include "flight/launch_wing.h"
    #include "flight/mixer.h"
    #include "flight/position.h"
    #include "io/gps.h"
    #include "rx/rx.h"
    #include "sensors/acceleration.h"
    #include "sensors/gyro.h"
    #include "sensors/sensors.h"
    #include "pg/launch_wing.h"

    extern uint8_t __config_start;
    extern uint8_t __config_end;

    acc_t acc;
    gyro_t gyro;
    attitudeEulerAngles_t attitude;
    float rcCommand[4];
    float autopilotAngle[RP_AXIS_COUNT];
    gpsSolutionData_t gpsSol;
    int16_t debug[DEBUG16_VALUE_COUNT];
    uint8_t debugMode;

    // test-controlled environment
    float testCosTilt = 1.0f;
    float testRcDeflection[3] = { 0.0f, 0.0f, 0.0f };
    throttleStatus_e testThrottleStatus = THROTTLE_LOW;
    bool testLaunchBoxActive = false;
    bool testIsFixedWing = true;
    bool testAccPresent = true;
    bool testAltitudeAvailable = true;
    float testAltitudeCm = 0.0f;
    float testAltitudeDerivative = 0.0f;
    bool testFailsafeActive = false;

    float getCosTiltAngle(void) { return testCosTilt; }
    float getRcDeflectionAbs(int axis) { return testRcDeflection[axis]; }
    throttleStatus_e calculateThrottleStatus(void) { return testThrottleStatus; }
    bool isFixedWing(void) { return testIsFixedWing; }
    bool sensors(uint32_t mask) { return (mask == SENSOR_ACC) ? testAccPresent : true; }
    bool isAltitudeAvailable(void) { return testAltitudeAvailable; }
    float getAltitudeCm(void) { return testAltitudeCm; }
    float getAltitudeDerivative(void) { return testAltitudeDerivative; }
    bool failsafeIsActive(void) { return testFailsafeActive; }
    bool IS_RC_MODE_ACTIVE(boxId_e boxId) { return boxId == BOXLAUNCH && testLaunchBoxActive; }

    int testTpaSpeedResets = 0;
    void pidResetTpaSpeed(void) { testTpaSpeedResets++; }
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

extern "C" {
    uint16_t flightModeFlags;
    uint8_t stateFlags;
    extern int testTpaSpeedResets;
}

static const timeUs_t US_PER_MS = 1000;

// Put the aircraft in the thrower's hand: level, still, nothing detectable.
static void setQuiescent(void)
{
    acc.dev.acc_1G_rec = 1.0f;
    acc.accADC.x = 0.0f;
    acc.accADC.y = 0.0f;
    acc.accADC.z = 1.0f;
    acc.accMagnitude = 1.0f;
    gyro.gyroADCf[FD_ROLL] = 0.0f;
    gyro.gyroADCf[FD_PITCH] = 0.0f;
    gyro.gyroADCf[FD_YAW] = 0.0f;
    testCosTilt = 1.0f;
    testRcDeflection[FD_ROLL] = 0.0f;
    testRcDeflection[FD_PITCH] = 0.0f;
    rcCommand[THROTTLE] = PWM_RANGE_MIN;
    gpsSol.numSat = 0;
    gpsSol.groundSpeed = 0;
    stateFlags = 0;
    testAltitudeCm = 0.0f;
    testAltitudeDerivative = 0.0f;
    testAltitudeAvailable = true;
    testFailsafeActive = false;
}

static void resetForTest(void)
{
    setQuiescent();
    testThrottleStatus = THROTTLE_LOW;
    testLaunchBoxActive = true;
    testIsFixedWing = true;
    testAccPresent = true;
    flightModeFlags = LAUNCH_MODE;
    pgResetAll();
    launchWingInit();
    launchWingArm();
}

// Drive the FSM from `fromUs` for `durationMs` at 1 ms steps, returning the end time.
static timeUs_t run(timeUs_t fromUs, uint32_t durationMs)
{
    timeUs_t t = fromUs;
    for (uint32_t i = 0; i < durationMs; i++) {
        t += US_PER_MS;
        launchWingUpdate(t);
    }
    return t;
}

static void setBungeeThrow(void)
{
    acc.accADC.x = 2.0f;    // 2.0 g forward, above the 1.9 g default
}

static void setSwingThrow(void)
{
    acc.accADC.x = 0.5f;    // forward, but below the bungee threshold
    acc.accADC.y = 1.5f;    // 1.5 g lateral
    gyro.gyroADCf[FD_YAW] = 200.0f;
    // 1.5 * 980.665 / radians(200) = 421 cm/s, above the 300 cm/s default
}

static void setForwardThrow(void)
{
    acc.accADC.x = 0.5f;
    stateFlags = GPS_FIX;
    gpsSol.numSat = 8;
    gpsSol.groundSpeed = 400;
}

// Advance to WAIT_DETECTION: raise throttle, then let the idle ramp complete.
static timeUs_t reachWaitDetection(timeUs_t t)
{
    testThrottleStatus = THROTTLE_HIGH;
    t = run(t, 1);
    EXPECT_EQ(LAUNCH_WING_MOTOR_IDLE, launchWingGetState());
    t = run(t, 1600);
    EXPECT_EQ(LAUNCH_WING_WAIT_DETECTION, launchWingGetState());
    return t;
}

TEST(LaunchWingTest, LatchRequiresBoxFixedWingAndStationary)
{
    resetForTest();
    EXPECT_TRUE(launchWingLatched());

    // box off at the arm instant
    testLaunchBoxActive = false;
    launchWingArm();
    EXPECT_FALSE(launchWingLatched());

    // not a wing
    testLaunchBoxActive = true;
    testIsFixedWing = false;
    launchWingArm();
    EXPECT_FALSE(launchWingLatched());

    // moving over the ground
    testIsFixedWing = true;
    stateFlags = GPS_FIX;
    gpsSol.numSat = 8;
    gpsSol.groundSpeed = 500;
    launchWingArm();
    EXPECT_FALSE(launchWingLatched());

    // being carried: accelerating, so not at 1 g
    setQuiescent();
    acc.accMagnitude = 1.4f;
    launchWingArm();
    EXPECT_FALSE(launchWingLatched());

    // and a clean stationary arm latches again
    setQuiescent();
    launchWingArm();
    EXPECT_TRUE(launchWingLatched());
}

TEST(LaunchWingTest, ArmingClearsTheModelledAirspeed)
{
    // a bench run before a launch must not leave TPA attenuating to a speed the
    // aircraft does not have
    resetForTest();
    const int before = testTpaSpeedResets;
    launchWingArm();
    EXPECT_EQ(before + 1, testTpaSpeedResets);
}

TEST(LaunchWingTest, DisarmClearsTheLatch)
{
    resetForTest();
    EXPECT_TRUE(launchWingLatched());
    launchWingDisarm();
    EXPECT_FALSE(launchWingLatched());
    EXPECT_EQ(LAUNCH_WING_IDLE, launchWingGetState());
}

TEST(LaunchWingTest, WaitsForThrottleAndHoldsMotorAtZero)
{
    resetForTest();
    timeUs_t t = run(0, 10);
    EXPECT_EQ(LAUNCH_WING_WAIT_THROTTLE, launchWingGetState());
    EXPECT_FLOAT_EQ(0.0f, launchWingGetThrottle());

    testThrottleStatus = THROTTLE_HIGH;
    t = run(t, 1);
    EXPECT_EQ(LAUNCH_WING_MOTOR_IDLE, launchWingGetState());

    // dropping the throttle before detection is recoverable, not an abort
    testThrottleStatus = THROTTLE_LOW;
    run(t, 1);
    EXPECT_EQ(LAUNCH_WING_WAIT_THROTTLE, launchWingGetState());
}

TEST(LaunchWingTest, IdleRampReachesIdleThrottleAndClimbAngle)
{
    resetForTest();
    timeUs_t t = run(0, 1);
    testThrottleStatus = THROTTLE_HIGH;
    t = run(t, 1);

    const float idle = launchWingConfig()->idleThrottlePercent * 0.01f;

    t = run(t, 750);    // halfway through the 1500 ms ramp
    EXPECT_NEAR(idle * 0.5f, launchWingGetThrottle(), idle * 0.1f);
    EXPECT_NEAR(launchWingConfig()->climbAngleDeg * -0.5f, autopilotAngle[AI_PITCH], 2.0f);

    t = run(t, 800);
    EXPECT_EQ(LAUNCH_WING_WAIT_DETECTION, launchWingGetState());
    EXPECT_FLOAT_EQ(idle, launchWingGetThrottle());
    EXPECT_FLOAT_EQ(-(float)launchWingConfig()->climbAngleDeg, autopilotAngle[AI_PITCH]);
    EXPECT_FLOAT_EQ(0.0f, autopilotAngle[AI_ROLL]);
}

TEST(LaunchWingTest, CarryingTheAircraftNeverSpinsTheMotorUp)
{
    resetForTest();
    timeUs_t t = run(0, 1);
    t = reachWaitDetection(t);

    const float idle = launchWingConfig()->idleThrottlePercent * 0.01f;

    // 30 s of being carried around and tilted, with occasional single-frame bumps
    for (int i = 0; i < 30000; i++) {
        t += US_PER_MS;
        if (i % 500 == 0) {
            acc.accADC.x = 2.5f;        // a knock, but only for this frame
        } else {
            acc.accADC.x = 0.0f;
            testCosTilt = (i % 100 < 50) ? 0.8f : 1.0f;
        }
        launchWingUpdate(t);
        ASSERT_EQ(LAUNCH_WING_WAIT_DETECTION, launchWingGetState());
        ASSERT_LE(launchWingGetThrottle(), idle + 0.02f);
    }
}

TEST(LaunchWingTest, DetectionRequiresAContinuousHold)
{
    resetForTest();
    timeUs_t t = run(0, 1);
    t = reachWaitDetection(t);

    // 39 ms is one short of the 40 ms default
    setBungeeThrow();
    t = run(t, 39);
    EXPECT_EQ(LAUNCH_WING_WAIT_DETECTION, launchWingGetState());

    // a single frame of no-detect re-seeds the clock
    setQuiescent();
    t = run(t, 1);
    setBungeeThrow();
    t = run(t, 39);
    EXPECT_EQ(LAUNCH_WING_WAIT_DETECTION, launchWingGetState());

    // one more frame completes an uninterrupted 40 ms
    t = run(t, 1);
    EXPECT_EQ(LAUNCH_WING_MOTOR_DELAY, launchWingGetState());
}

TEST(LaunchWingTest, BungeeDetectorFiresOnForwardAccelWhenLevel)
{
    resetForTest();
    timeUs_t t = run(0, 1);
    t = reachWaitDetection(t);

    // just under the 1.9 g threshold does nothing
    acc.accADC.x = 1.8f;
    t = run(t, 100);
    EXPECT_EQ(LAUNCH_WING_WAIT_DETECTION, launchWingGetState());

    // over the threshold but nose-high beyond launch_max_angle does nothing
    acc.accADC.x = 2.0f;
    testCosTilt = cosf(DEGREES_TO_RADIANS(60.0f));
    t = run(t, 100);
    EXPECT_EQ(LAUNCH_WING_WAIT_DETECTION, launchWingGetState());

    // level and over the threshold fires
    testCosTilt = 1.0f;
    t = run(t, 50);
    EXPECT_EQ(LAUNCH_WING_MOTOR_DELAY, launchWingGetState());
}

TEST(LaunchWingTest, SwingDetectorUsesLateralAccelOverYawRate)
{
    resetForTest();
    timeUs_t t = run(0, 1);
    t = reachWaitDetection(t);

    // lateral accel with too little rotation is not a swing
    acc.accADC.x = 0.5f;
    acc.accADC.y = 1.5f;
    gyro.gyroADCf[FD_YAW] = 50.0f;
    t = run(t, 100);
    EXPECT_EQ(LAUNCH_WING_WAIT_DETECTION, launchWingGetState());

    // rotation without enough lateral accel is not a swing either:
    // 0.2 g at 200 dps is 56 cm/s, well under the 300 cm/s default
    acc.accADC.y = 0.2f;
    gyro.gyroADCf[FD_YAW] = 200.0f;
    t = run(t, 100);
    EXPECT_EQ(LAUNCH_WING_WAIT_DETECTION, launchWingGetState());

    // a real swing fires, and the sign of the rotation must not matter
    acc.accADC.y = 1.5f;
    gyro.gyroADCf[FD_YAW] = -200.0f;
    t = run(t, 50);
    EXPECT_EQ(LAUNCH_WING_MOTOR_DELAY, launchWingGetState());
}

TEST(LaunchWingTest, SwingDetectorRejectsABackswing)
{
    resetForTest();
    timeUs_t t = run(0, 1);
    t = reachWaitDetection(t);

    // same rotation and lateral accel, but decelerating: no forward specific force
    setSwingThrow();
    acc.accADC.x = -0.5f;
    t = run(t, 200);
    EXPECT_EQ(LAUNCH_WING_WAIT_DETECTION, launchWingGetState());
}

TEST(LaunchWingTest, ForwardDetectorFiresOnGroundSpeed)
{
    resetForTest();
    timeUs_t t = run(0, 1);
    t = reachWaitDetection(t);

    // ground speed without a fix does nothing
    gpsSol.groundSpeed = 400;
    acc.accADC.x = 0.5f;
    t = run(t, 100);
    EXPECT_EQ(LAUNCH_WING_WAIT_DETECTION, launchWingGetState());

    setForwardThrow();
    t = run(t, 50);
    EXPECT_EQ(LAUNCH_WING_MOTOR_DELAY, launchWingGetState());
}

TEST(LaunchWingTest, MotorDelayThenSpinupThenClimbOut)
{
    resetForTest();
    timeUs_t t = run(0, 1);
    t = reachWaitDetection(t);
    setBungeeThrow();
    t = run(t, 41);
    ASSERT_EQ(LAUNCH_WING_MOTOR_DELAY, launchWingGetState());

    const float idle = launchWingConfig()->idleThrottlePercent * 0.01f;
    const float launch = launchWingConfig()->throttlePercent * 0.01f;

    // throttle stays at idle for the whole motor delay
    t = run(t, launchWingConfig()->motorDelayMs - 10);
    EXPECT_EQ(LAUNCH_WING_MOTOR_DELAY, launchWingGetState());
    EXPECT_FLOAT_EQ(idle, launchWingGetThrottle());

    t = run(t, 20);
    EXPECT_EQ(LAUNCH_WING_SPINUP, launchWingGetState());

    t = run(t, launchWingConfig()->spinupTimeMs + 5);
    EXPECT_EQ(LAUNCH_WING_IN_PROGRESS, launchWingGetState());
    EXPECT_FLOAT_EQ(launch, launchWingGetThrottle());
    EXPECT_FLOAT_EQ(-(float)launchWingConfig()->climbAngleDeg, autopilotAngle[AI_PITCH]);
}

TEST(LaunchWingTest, ItermIsHeldUntilSpinup)
{
    resetForTest();
    timeUs_t t = run(0, 1);
    EXPECT_TRUE(launchWingHoldsIterm());
    t = reachWaitDetection(t);
    EXPECT_TRUE(launchWingHoldsIterm());

    setBungeeThrow();
    t = run(t, 41);
    EXPECT_EQ(LAUNCH_WING_MOTOR_DELAY, launchWingGetState());
    EXPECT_TRUE(launchWingHoldsIterm());

    t = run(t, launchWingConfig()->motorDelayMs + 5);
    EXPECT_EQ(LAUNCH_WING_SPINUP, launchWingGetState());
    EXPECT_FALSE(launchWingHoldsIterm());
}

TEST(LaunchWingTest, TimeoutFinishesRatherThanAborts)
{
    resetForTest();
    timeUs_t t = run(0, 1);
    t = reachWaitDetection(t);
    setBungeeThrow();
    t = run(t, 41);
    t = run(t, launchWingConfig()->motorDelayMs + launchWingConfig()->spinupTimeMs + 10);
    ASSERT_EQ(LAUNCH_WING_IN_PROGRESS, launchWingGetState());

    t = run(t, launchWingConfig()->timeoutMs + 5);
    EXPECT_EQ(LAUNCH_WING_FINISH, launchWingGetState());
}

TEST(LaunchWingTest, MaxAltitudeFinishesTheLaunch)
{
    resetForTest();
    launchWingConfigMutable()->maxAltitudeM = 30;
    timeUs_t t = run(0, 1);
    t = reachWaitDetection(t);
    setBungeeThrow();
    t = run(t, 41);
    t = run(t, launchWingConfig()->motorDelayMs + launchWingConfig()->spinupTimeMs + 10);
    ASSERT_EQ(LAUNCH_WING_IN_PROGRESS, launchWingGetState());

    testAltitudeCm = 2500.0f;
    t = run(t, 10);
    EXPECT_EQ(LAUNCH_WING_IN_PROGRESS, launchWingGetState());

    testAltitudeCm = 3100.0f;
    t = run(t, 10);
    EXPECT_EQ(LAUNCH_WING_FINISH, launchWingGetState());
}

TEST(LaunchWingTest, FinishCrossFadesThrottleAndPitchBackToThePilot)
{
    resetForTest();
    timeUs_t t = run(0, 1);
    t = reachWaitDetection(t);
    setBungeeThrow();
    t = run(t, 41);
    t = run(t, launchWingConfig()->motorDelayMs + launchWingConfig()->spinupTimeMs + 10);

    t = run(t, launchWingConfig()->timeoutMs + 5);
    ASSERT_EQ(LAUNCH_WING_FINISH, launchWingGetState());

    const float launch = launchWingConfig()->throttlePercent * 0.01f;

    // the launch-side demands are held; only the handover factor moves, and it
    // must ramp monotonically from 0 to 1 across launch_end_time
    float previous = launchWingHandoverFactor();
    EXPECT_NEAR(0.0f, previous, 0.01f);
    for (uint32_t i = 0; i < launchWingConfig()->endTimeMs; i++) {
        t += US_PER_MS;
        launchWingUpdate(t);
        if (launchWingGetState() != LAUNCH_WING_FINISH) {
            break;
        }
        ASSERT_GE(launchWingHandoverFactor(), previous - 1e-6f);
        previous = launchWingHandoverFactor();
        ASSERT_FLOAT_EQ(launch, launchWingGetThrottle());
    }
    EXPECT_EQ(LAUNCH_WING_FLYING, launchWingGetState());
    EXPECT_FALSE(launchWingThrottleValid());
}

TEST(LaunchWingTest, HandoverFactorIsZeroUntilTheHandBack)
{
    resetForTest();
    timeUs_t t = run(0, 1);
    EXPECT_FLOAT_EQ(0.0f, launchWingHandoverFactor());
    t = reachWaitDetection(t);
    EXPECT_FLOAT_EQ(0.0f, launchWingHandoverFactor());
    setBungeeThrow();
    t = run(t, 41);
    t = run(t, launchWingConfig()->motorDelayMs + launchWingConfig()->spinupTimeMs + 10);
    ASSERT_EQ(LAUNCH_WING_IN_PROGRESS, launchWingGetState());
    EXPECT_FLOAT_EQ(0.0f, launchWingHandoverFactor());
}

TEST(LaunchWingTest, StickAbortIsInhibitedUntilMinTime)
{
    resetForTest();
    launchWingConfigMutable()->minTimeMs = 1000;
    timeUs_t t = run(0, 1);
    t = reachWaitDetection(t);
    setBungeeThrow();
    t = run(t, 41);
    ASSERT_EQ(LAUNCH_WING_MOTOR_DELAY, launchWingGetState());

    // full roll deflection inside the inhibit window is ignored, across every
    // post-detection state
    testRcDeflection[FD_ROLL] = 1.0f;
    t = run(t, 400);
    EXPECT_EQ(LAUNCH_WING_MOTOR_DELAY, launchWingGetState());
    t = run(t, 150);
    EXPECT_EQ(LAUNCH_WING_SPINUP, launchWingGetState());
    t = run(t, 150);
    EXPECT_EQ(LAUNCH_WING_IN_PROGRESS, launchWingGetState());

    // once the window closes the same deflection aborts
    t = run(t, 350);
    EXPECT_EQ(LAUNCH_WING_ABORTED, launchWingGetState());
}

TEST(LaunchWingTest, StickAbortHandsBackControlImmediately)
{
    resetForTest();
    timeUs_t t = run(0, 1);
    t = reachWaitDetection(t);
    setBungeeThrow();
    t = run(t, 41);
    t = run(t, launchWingConfig()->motorDelayMs + launchWingConfig()->spinupTimeMs + 10);
    ASSERT_EQ(LAUNCH_WING_IN_PROGRESS, launchWingGetState());
    ASSERT_TRUE(launchWingThrottleValid());

    testRcDeflection[FD_PITCH] = 0.5f;
    t = run(t, 1);
    EXPECT_EQ(LAUNCH_WING_ABORTED, launchWingGetState());
    EXPECT_FALSE(launchWingThrottleValid());
    EXPECT_FALSE(launchWingIsActive());
    EXPECT_TRUE(launchWingIsTerminal());
}

TEST(LaunchWingTest, DeflectionInsideTheDeadbandDoesNotAbort)
{
    resetForTest();
    timeUs_t t = run(0, 1);
    t = reachWaitDetection(t);
    setBungeeThrow();
    t = run(t, 41);
    t = run(t, launchWingConfig()->motorDelayMs + launchWingConfig()->spinupTimeMs + 10);
    ASSERT_EQ(LAUNCH_WING_IN_PROGRESS, launchWingGetState());

    // default abort deadband is 20%
    testRcDeflection[FD_ROLL] = 0.15f;
    t = run(t, 100);
    EXPECT_EQ(LAUNCH_WING_IN_PROGRESS, launchWingGetState());

    testRcDeflection[FD_ROLL] = 0.25f;
    t = run(t, 1);
    EXPECT_EQ(LAUNCH_WING_ABORTED, launchWingGetState());
}

TEST(LaunchWingTest, ModeDroppingAbortsAndTerminalStatesAreSticky)
{
    resetForTest();
    timeUs_t t = run(0, 1);
    t = reachWaitDetection(t);

    // the mode going away mid-sequence aborts
    flightModeFlags = 0;
    t = run(t, 1);
    EXPECT_EQ(LAUNCH_WING_ABORTED, launchWingGetState());

    // and it cannot be re-entered within the same arm
    flightModeFlags = LAUNCH_MODE;
    t = run(t, 100);
    EXPECT_EQ(LAUNCH_WING_ABORTED, launchWingGetState());
    EXPECT_FALSE(launchWingThrottleValid());
}

TEST(LaunchWingTest, SwitchOffAbortsFromAnyActiveState)
{
    resetForTest();
    timeUs_t t = run(0, 1);
    t = reachWaitDetection(t);
    launchWingSwitchOff();
    EXPECT_EQ(LAUNCH_WING_ABORTED, launchWingGetState());

    // and is a no-op once terminal
    launchWingSwitchOff();
    EXPECT_EQ(LAUNCH_WING_ABORTED, launchWingGetState());
}

TEST(LaunchWingTest, ClimbAngleIsCommandedNoseUp)
{
    resetForTest();
    timeUs_t t = run(0, 1);
    t = reachWaitDetection(t);

    // autopilotAngle is positive nose-down, so a climb must be negative
    EXPECT_LT(autopilotAngle[AI_PITCH], 0.0f);
    EXPECT_FLOAT_EQ(-(float)launchWingConfig()->climbAngleDeg, autopilotAngle[AI_PITCH]);
}

TEST(LaunchWingTest, LatchRejectsMotionWithoutGps)
{
    resetForTest();

    // rotating at 1 g: a steady glide looks stationary to the accelerometer alone
    setQuiescent();
    gyro.gyroADCf[FD_PITCH] = 60.0f;
    launchWingArm();
    EXPECT_FALSE(launchWingLatched());

    // descending at 1 g
    setQuiescent();
    testAltitudeDerivative = -250.0f;
    launchWingArm();
    EXPECT_FALSE(launchWingLatched());

    setQuiescent();
    launchWingArm();
    EXPECT_TRUE(launchWingLatched());
}

TEST(LaunchWingTest, FailsafeAbortsTheLaunch)
{
    resetForTest();
    timeUs_t t = run(0, 1);
    t = reachWaitDetection(t);
    setBungeeThrow();
    t = run(t, 41);
    t = run(t, launchWingConfig()->motorDelayMs + launchWingConfig()->spinupTimeMs + 10);
    ASSERT_EQ(LAUNCH_WING_IN_PROGRESS, launchWingGetState());

    // the mode bit only clears on the next rx cycle, so the launch must stand
    // down on the failsafe itself
    testFailsafeActive = true;
    t = run(t, 1);
    EXPECT_EQ(LAUNCH_WING_ABORTED, launchWingGetState());
    EXPECT_FALSE(launchWingThrottleValid());
}

TEST(LaunchWingTest, EveryExitHandsBackFully)
{
    // pidLevel and mixTable blend on the handover factor until the mode bit
    // clears, so a launch that ends early must not leave it part-way
    resetForTest();
    timeUs_t t = run(0, 1);
    t = reachWaitDetection(t);
    setBungeeThrow();
    t = run(t, 41);
    t = run(t, launchWingConfig()->motorDelayMs + launchWingConfig()->spinupTimeMs + 10);
    ASSERT_EQ(LAUNCH_WING_IN_PROGRESS, launchWingGetState());
    ASSERT_FLOAT_EQ(0.0f, launchWingHandoverFactor());

    testRcDeflection[FD_PITCH] = 1.0f;
    t = run(t, 1);
    ASSERT_EQ(LAUNCH_WING_ABORTED, launchWingGetState());
    EXPECT_FLOAT_EQ(1.0f, launchWingHandoverFactor());

    // and the same for a stick abort part-way through the cross-fade
    resetForTest();
    t = run(0, 1);
    t = reachWaitDetection(t);
    setBungeeThrow();
    t = run(t, 41);
    t = run(t, launchWingConfig()->motorDelayMs + launchWingConfig()->spinupTimeMs + 10);
    t = run(t, launchWingConfig()->timeoutMs + 5);
    ASSERT_EQ(LAUNCH_WING_FINISH, launchWingGetState());
    t = run(t, launchWingConfig()->endTimeMs / 4);
    ASSERT_EQ(LAUNCH_WING_FINISH, launchWingGetState());
    ASSERT_LT(launchWingHandoverFactor(), 0.5f);

    testRcDeflection[FD_PITCH] = 1.0f;
    t = run(t, 1);
    EXPECT_EQ(LAUNCH_WING_FLYING, launchWingGetState());
    EXPECT_FLOAT_EQ(1.0f, launchWingHandoverFactor());
}
