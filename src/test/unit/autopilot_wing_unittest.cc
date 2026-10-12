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
    #include "fc/core.h"
    #include "fc/rc.h"
    #include "fc/runtime_config.h"
    #include "flight/alt_hold.h"
    #include "flight/autopilot.h"
    #include "flight/failsafe.h"
    #include "flight/gps_rescue.h"
    #include "flight/imu.h"
    #include "flight/landing_wing.h"
    #include "flight/launch_wing.h"
    #include "flight/mixer.h"
    #include "flight/pid.h"
    #include "flight/pos_hold.h"
    #include "flight/position.h"
    #include "flight/position_estimator.h"
    #include "flight/position_nav.h"
    #include "scheduler/scheduler.h"
    #include "sensors/acceleration.h"
    #include "sensors/gyro.h"
    #include "pg/alt_hold.h"
    #include "pg/autopilot_wing.h"
    #include "pg/gps.h"
    #include "pg/pg_ids.h"
    #include "pg/pos_hold.h"
    #include "pg/rx.h"

    extern uint8_t __config_start;
    extern uint8_t __config_end;

    attitudeEulerAngles_t attitude;
    matrix33_t rMat;
    int16_t debug[DEBUG16_VALUE_COUNT];
    uint8_t debugMode;
    uint16_t flightModeFlags;
    uint8_t armingFlags;
    acc_t acc;
    gyro_t gyro;

    static pidProfile_t testPidProfile;
    pidProfile_t *currentPidProfile = &testPidProfile;

    timeUs_t testMicros = 0;
    float testAltitudeCm = 0.0f;
    float testClimbRateCmS = 0.0f;
    float testMixerThrottle = 0.5f;
    bool testLaunchThrottleValid = false;
    float testLaunchThrottle = 0.8f;
    float testPitchDeflection = 0.0f;
    float testRollDeflection = 0.0f;
    positionEstimate3d_t testEstimate;
    int testNavUpdates = 0;
    bool testFailsafe = false;
    bool testNavActive = false;
    positionNavCommand_t testNavCommand;
    float testNavAltitudeCm = 0.0f;
    float testNavRateLimitCmS = 0.0f;
    vector3_t testNavVelocityCmS;
    int testDisarms = 0;

    timeUs_t micros(void) { return testMicros; }
    float getAltitudeCmControl(void) { return testAltitudeCm; }
    float getAltitudeDerivativeControl(void) { return testClimbRateCmS; }
    float mixerGetThrottle(void) { return testMixerThrottle; }
    bool launchWingThrottleValid(void) { return testLaunchThrottleValid; }
    bool testThrown = false;
    bool launchWingThrown(void) { return testThrown; }
    float launchWingGetThrottle(void) { return testLaunchThrottle; }
    timeDelta_t getTaskDeltaTimeUs(taskId_e) { return 10000; }
    float getRcDeflection(int axis) { return axis == FD_PITCH ? testPitchDeflection : testRollDeflection; }
    float getRcDeflectionAbs(int axis) { return fabsf(getRcDeflection(axis)); }
    bool failsafeIsActive(void) { return testFailsafe; }
    bool positionEstimatorTakeUpdate(positionEstimatorConsumer_e) { return true; }
    const positionEstimate3d_t *positionEstimatorGetEstimate(void) { return &testEstimate; }
    bool positionEstimatorIsValidXY(void) { return testEstimate.isValidXY; }
    void positionNavUpdate(float, const positionEstimate3d_t *) { testNavUpdates++; }
    bool positionNavHasActiveTarget(void) { return testNavActive; }
    const positionNavCommand_t *positionNavGetActiveCommand(void) { return &testNavCommand; }
    float positionNavGetTargetAltitudeCm(void) { return testNavAltitudeCm; }
    float positionNavGetVerticalRateLimitCmS(void) { return testNavRateLimitCmS; }
    vector3_t positionNavGetTargetVelocityCmS(void) { return testNavVelocityCmS; }
    void disarm(flightLogDisarmReason_e) { testDisarms++; }
    uint8_t stateFlags;
    PG_REGISTER(gpsConfig_t, gpsConfig, PG_GPS_CONFIG, 0);
    PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);
    bool gpsIsHealthy(void) { return true; }
    bool isAltitudeAvailable(void) { return true; }
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

static const timeUs_t STEP_US = 10000;

static float pitchTargetDeg(void)
{
    return autopilotAngle[AI_PITCH];
}

// Level, slowing along the nose at g.
static void brake(float g)
{
    acc.dev.acc_1G = 512;
    acc.dev.acc_1G_rec = 1.0f / acc.dev.acc_1G;
    acc.accADC.v[X] = -g * acc.dev.acc_1G;
}

static void resetForTest(void)
{
    pgResetAll();
    testPidProfile.angle_pitch_offset = 0;
    attitude.values.roll = 0;
    attitude.values.pitch = 0;
    attitude.values.yaw = 0;
    testMicros = 1000000;
    testAltitudeCm = 10000.0f;
    testClimbRateCmS = 0.0f;
    testMixerThrottle = 0.5f;
    testLaunchThrottleValid = false;
    testPitchDeflection = 0.0f;
    testRollDeflection = 0.0f;
    testEstimate = {};
    testEstimate.isValidXY = true;
    testNavUpdates = 0;
    testFailsafe = false;
    testNavActive = false;
    testNavCommand = {};
    testNavVelocityCmS = {};
    testDisarms = 0;
    testThrown = false;
    acc.accMagnitude = 1.0f;
    brake(0.0f);
    gyro = {};
    flightModeFlags = 0;
    debugMode = DEBUG_AUTOPILOT_CLIMB;
    autopilotAngle[AI_ROLL] = 0.0f;
    autopilotAngle[AI_PITCH] = 0.0f;
    autopilotInit();
}

static void runVertical(float targetCm, int steps, float targetVelCmS = 0.0f, float velLimitCmS = 0.0f)
{
    for (int i = 0; i < steps; i++) {
        testMicros += STEP_US;
        altitudeControl(targetCm, STEP_US, targetVelCmS, velLimitCmS);
    }
}

static void setBankDeg(float bankDeg)
{
    attitude.values.roll = lrintf(bankDeg * 10.0f);
}

static float integratorDeg(void)
{
    return autopilotWingGetPitchIntegratorDeg();
}

// Vertical controller

TEST(AutopilotWingTest, LevelAtTargetHoldsTrimAndCruiseThrottle)
{
    resetForTest();
    runVertical(testAltitudeCm, 300);
    EXPECT_NEAR(0.0f, pitchTargetDeg(), 0.01f);
    EXPECT_NEAR(autopilotWingConfig()->cruiseThrottle * 0.01f, getAutopilotThrottle(), 0.001f);
}

TEST(AutopilotWingTest, BelowTargetPitchesUpAboveTargetPitchesDown)
{
    resetForTest();
    runVertical(testAltitudeCm + 5000.0f, 100);
    EXPECT_LT(pitchTargetDeg(), -1.0f);    // autopilotAngle is nose-down positive

    resetForTest();
    runVertical(testAltitudeCm - 5000.0f, 100);
    EXPECT_GT(pitchTargetDeg(), 1.0f);
}

TEST(AutopilotWingTest, ThrottleRunsFromMinAtFullDiveToMaxAtFullClimb)
{
    resetForTest();
    runVertical(testAltitudeCm + 5000.0f, 1000);
    EXPECT_NEAR(-(float)autopilotWingConfig()->maxClimbAngle, pitchTargetDeg(), 0.01f);
    EXPECT_NEAR(autopilotWingConfig()->maxThrottle * 0.01f, getAutopilotThrottle(), 0.001f);

    resetForTest();
    runVertical(testAltitudeCm - 5000.0f, 1000);
    EXPECT_NEAR((float)autopilotWingConfig()->maxDiveAngle, pitchTargetDeg(), 0.01f);
    EXPECT_NEAR(autopilotWingConfig()->minThrottle * 0.01f, getAutopilotThrottle(), 0.001f);
}

TEST(AutopilotWingTest, BankAddsThrottleForTheLoadFactor)
{
    resetForTest();
    setBankDeg(35.0f);
    runVertical(testAltitudeCm, 300);
    // 10% at 45 degrees, scaling with tan^2: 4.9% at 35 degrees
    EXPECT_NEAR(0.5f + 0.1f * sq(tanf(DEGREES_TO_RADIANS(35.0f))), getAutopilotThrottle(), 0.002f);

    resetForTest();
    autopilotWingConfigMutable()->bankThrottle = 100;
    setBankDeg(70.0f);
    runVertical(testAltitudeCm, 1000);
    EXPECT_NEAR(autopilotWingConfig()->maxThrottle * 0.01f, getAutopilotThrottle(), 0.001f);
}

// Coming down 2 m/s on the throttle for the path, the nose held at pitchDeg, flying north at groundspeedCmS.
static void runForThePath(float pitchDeg, float groundspeedCmS, int steps)
{
    autopilotWingLimits_t limits;
    autopilotWingDefaultLimits(&limits);
    limits.throttleForPath = true;
    limits.throttleMin = 0.0f;
    limits.climbRateOverride = true;
    limits.climbRateCmS = -200.0f;
    limits.pitchMinDeg = pitchDeg;
    limits.pitchMaxDeg = pitchDeg;
    autopilotWingSetLimits(AP_WING_LIMITS_LANDING, &limits);
    testEstimate.velocity.v[ENU_N] = groundspeedCmS;
    testClimbRateCmS = -200.0f;
    runVertical(testAltitudeCm, steps);
}

TEST(AutopilotWingTest, ForThePathTheThrottleIdlesWithTheNoseNearItAndOpensAsTheNoseRises)
{
    // 2 m/s down at 15 m/s is a 7.6 deg path
    const float pathDeg = -RADIANS_TO_DEGREES(atanf(200.0f / 1500.0f));
    resetForTest();
    runForThePath(pathDeg + 1.5f, 1500.0f, 300);
    EXPECT_FLOAT_EQ(0.0f, getAutopilotThrottle());

    // half way from idling to the cruise throttle
    resetForTest();
    runForThePath(pathDeg + 3.5f, 1500.0f, 300);
    EXPECT_NEAR(0.5f * autopilotWingConfig()->cruiseThrottle * 0.01f, getAutopilotThrottle(), 0.01f);

    resetForTest();
    runForThePath(pathDeg + 8.0f, 1500.0f, 300);
    EXPECT_NEAR(autopilotWingConfig()->cruiseThrottle * 0.01f, getAutopilotThrottle(), 0.001f);
}

TEST(AutopilotWingTest, IntoAHeadwindThePathOverTheGroundIsSteeperAndTheThrottleOpensSooner)
{
    const float pathDeg = -RADIANS_TO_DEGREES(atanf(200.0f / 1500.0f));
    resetForTest();
    runForThePath(pathDeg + 2.5f, 1500.0f, 300);
    const float throttle = getAutopilotThrottle();
    resetForTest();
    runForThePath(pathDeg + 2.5f, 1000.0f, 300);
    EXPECT_GT(getAutopilotThrottle(), throttle + 0.2f);
}

TEST(AutopilotWingTest, PushedDownToTheDiveLimitTheThrottleForThePathIdlesHoweverSlowOverTheGround)
{
    // stopped on the ground, the path is as steep as it gets
    resetForTest();
    runForThePath(-autopilotWingConfig()->maxDiveAngle, 0.0f, 300);
    EXPECT_FLOAT_EQ(0.0f, getAutopilotThrottle());
}

TEST(AutopilotWingTest, ForThePathABankAddsThrottleForTheLoadFactor)
{
    const float pathDeg = -RADIANS_TO_DEGREES(atanf(200.0f / 1500.0f));
    resetForTest();
    setBankDeg(35.0f);
    runForThePath(pathDeg + 1.5f, 1500.0f, 300);
    EXPECT_NEAR(autopilotWingConfig()->bankThrottle * 0.01f * sq(tanf(DEGREES_TO_RADIANS(35.0f))), getAutopilotThrottle(), 0.002f);
}

TEST(AutopilotWingTest, PitchDemandSlewsAtTheVerticalAccelerationLimit)
{
    resetForTest();
    runVertical(testAltitudeCm, 1);
    const float before = pitchTargetDeg();
    runVertical(testAltitudeCm + 5000.0f, 1);
    // 5 m/s^2 at 15 m/s is 19.1 deg/s
    const float stepDeg = RADIANS_TO_DEGREES(5.0f / 15.0f) * 0.01f;
    EXPECT_NEAR(stepDeg, before - pitchTargetDeg(), 0.01f);
}

TEST(AutopilotWingTest, IntegratorDoesNotWindAgainstThePitchLimit)
{
    resetForTest();
    autopilotWingConfigMutable()->maxClimbAngle = 10;
    // demanding a climb the aircraft never achieves: the pitch saturates
    runVertical(testAltitudeCm + 5000.0f, 3000);
    EXPECT_NEAR(0.0f, integratorDeg(), 0.1f);

    // so the moment the demand is met the nose comes straight back down
    runVertical(testAltitudeCm, 100);
    EXPECT_GT(pitchTargetDeg(), -1.0f);
}

TEST(AutopilotWingTest, IntegratorTrimsOutAPersistentSink)
{
    resetForTest();
    runVertical(testAltitudeCm, 1);
    testClimbRateCmS = -50.0f;
    runVertical(testAltitudeCm, 300);
    EXPECT_GT(integratorDeg(), 1.0f);
}

TEST(AutopilotWingTest, IntegratorFreezesInASteepBank)
{
    resetForTest();
    runVertical(testAltitudeCm, 1);
    testClimbRateCmS = -50.0f;
    setBankDeg(50.0f);
    runVertical(testAltitudeCm, 300);
    EXPECT_NEAR(0.0f, integratorDeg(), 0.05f);

    setBankDeg(30.0f);
    runVertical(testAltitudeCm, 300);
    EXPECT_GT(integratorDeg(), 1.0f);
}

TEST(AutopilotWingTest, IntegratorFreezesJustPastTheBankGuidanceMayCommand)
{
    resetForTest();
    autopilotWingConfigMutable()->maxBank = 50;
    runVertical(testAltitudeCm, 1);
    testClimbRateCmS = -50.0f;
    setBankDeg(54.0f);
    runVertical(testAltitudeCm, 300);
    EXPECT_GT(integratorDeg(), 1.0f);

    resetForTest();
    autopilotWingConfigMutable()->maxBank = 20;
    runVertical(testAltitudeCm, 1);
    testClimbRateCmS = -50.0f;
    setBankDeg(26.0f);
    runVertical(testAltitudeCm, 300);
    EXPECT_NEAR(0.0f, integratorDeg(), 0.05f);
}

TEST(AutopilotWingTest, EntryContinuesAtTheCurrentPitchAndThrottle)
{
    resetForTest();
    attitude.values.pitch = -100;   // 10 degrees nose-up, level flight
    testMixerThrottle = 0.63f;
    runVertical(testAltitudeCm, 1);
    EXPECT_NEAR(-10.0f, pitchTargetDeg(), 0.01f);
    EXPECT_NEAR(0.63f, getAutopilotThrottle(), 0.006f);
}

TEST(AutopilotWingTest, EntryPitchIsMeasuredFromTheTrim)
{
    resetForTest();
    testPidProfile.angle_pitch_offset = -60;    // trimmed 6 degrees nose-up
    attitude.values.pitch = -60;
    runVertical(testAltitudeCm, 1);
    EXPECT_NEAR(0.0f, pitchTargetDeg(), 0.01f);
}

TEST(AutopilotWingTest, EntryFromALaunchStartsFromTheLaunchThrottle)
{
    // the launch is still flying while the hold it hands to starts up
    resetForTest();
    flightModeFlags = LAUNCH_MODE | ALT_HOLD_MODE;
    testLaunchThrottleValid = true;
    testLaunchThrottle = 0.8f;
    runVertical(testAltitudeCm, 1);
    EXPECT_NEAR(0.8f, getAutopilotThrottle(), 0.006f);

    // once it has ended, a re-entry is the pilot's throttle again
    flightModeFlags = ALT_HOLD_MODE;
    testLaunchThrottleValid = false;
    resetAltitudeControl();
    runVertical(testAltitudeCm, 1);
    EXPECT_NEAR(0.5f, getAutopilotThrottle(), 0.006f);
}

TEST(AutopilotWingTest, ThrottleIsValidOnlyOnceTheControllerHasRun)
{
    resetForTest();
    EXPECT_FALSE(autopilotThrottleValid());
    runVertical(testAltitudeCm, 1);
    EXPECT_TRUE(autopilotThrottleValid());
    resetAltitudeControl();
    EXPECT_FALSE(autopilotThrottleValid());
}

TEST(AutopilotWingTest, ThrottleSlewsRatherThanSteps)
{
    // at half the throttle range a second, a step at a time
    const float stepThrottle = 0.5f * STEP_US * 1e-6f;
    resetForTest();
    testMixerThrottle = 0.2f;
    runVertical(testAltitudeCm, 1);
    EXPECT_NEAR(0.2f + stepThrottle, getAutopilotThrottle(), 0.001f);
    runVertical(testAltitudeCm, 20);
    EXPECT_NEAR(0.2f + 21 * stepThrottle, getAutopilotThrottle(), 0.001f);
}

TEST(AutopilotWingTest, ALongPauseReseedsFromTheAttitude)
{
    resetForTest();
    runVertical(testAltitudeCm + 5000.0f, 300);
    ASSERT_LT(pitchTargetDeg(), -10.0f);

    attitude.values.pitch = -30;
    testMicros += 300000;
    runVertical(testAltitudeCm, 1);
    EXPECT_NEAR(-3.0f, pitchTargetDeg(), 0.01f);
}

TEST(AutopilotWingTest, ClimbDemandRespectsTheVelocityLimit)
{
    resetForTest();
    runVertical(testAltitudeCm + 5000.0f, 1, 0.0f, 100.0f);
    EXPECT_FLOAT_EQ(100.0f, autopilotWingGetClimbDemandCmS());
    runVertical(testAltitudeCm + 5000.0f, 1);
    EXPECT_FLOAT_EQ(autopilotWingConfig()->maxClimbRate * 10.0f, autopilotWingGetClimbDemandCmS());
    runVertical(testAltitudeCm - 5000.0f, 1);
    EXPECT_FLOAT_EQ(-autopilotWingConfig()->maxSinkRate * 10.0f, autopilotWingGetClimbDemandCmS());
}

// Turn pitch feedforward

TEST(AutopilotWingTest, TurnNeedsTheSameNoseUpRateEitherWay)
{
    // g * tan(30) * sin(30) / 15 m/s = 10.8 deg/s nose-up, negative in the angle loop
    resetForTest();
    setBankDeg(30.0f);
    runVertical(testAltitudeCm, 1);
    EXPECT_NEAR(-10.81f, autopilotGetTurnPitchRateDps(), 0.05f);

    setBankDeg(-30.0f);
    runVertical(testAltitudeCm, 1);
    EXPECT_NEAR(-10.81f, autopilotGetTurnPitchRateDps(), 0.05f);
}

TEST(AutopilotWingTest, NoTurnPitchRateWhenLevelOrDisengaged)
{
    resetForTest();
    runVertical(testAltitudeCm, 1);
    EXPECT_FLOAT_EQ(0.0f, autopilotGetTurnPitchRateDps());

    setBankDeg(30.0f);
    runVertical(testAltitudeCm, 1);
    resetAltitudeControl();
    EXPECT_FLOAT_EQ(0.0f, autopilotGetTurnPitchRateDps());
}

TEST(AutopilotWingTest, TurnPitchRateScalesWithTheSetting)
{
    resetForTest();
    autopilotWingConfigMutable()->turnPitchFf = 50;
    setBankDeg(30.0f);
    runVertical(testAltitudeCm, 1);
    EXPECT_NEAR(-5.40f, autopilotGetTurnPitchRateDps(), 0.05f);
}

TEST(AutopilotWingTest, TheTurnFeedforwardFadesOutRolledTowardsInverted)
{
    resetForTest();
    setBankDeg(60.0f);
    runVertical(testAltitudeCm, 1);
    const float steepestDps = autopilotGetTurnPitchRateDps();
    ASSERT_LT(steepestDps, -50.0f);

    setBankDeg(75.0f);
    runVertical(testAltitudeCm, 1);
    EXPECT_NEAR(0.5f * steepestDps, autopilotGetTurnPitchRateDps(), 0.1f);

    // nose up past the vertical is towards the ground
    for (const float bank : { 80.0f, 95.0f, 120.0f, 180.0f, -100.0f, -179.0f }) {
        setBankDeg(bank);
        runVertical(testAltitudeCm, 1);
        EXPECT_FLOAT_EQ(0.0f, autopilotGetTurnPitchRateDps()) << bank;
    }
}

// Alt hold

static void runAltHold(int steps)
{
    for (int i = 0; i < steps; i++) {
        testMicros += STEP_US;
        updateAltHold(testMicros);
    }
}

static float holdTargetCm(void)
{
    return altHoldGetTargetAltitudeCm();
}

static void engageAltHold(void)
{
    resetForTest();
    altHoldInit();
    flightModeFlags = ALT_HOLD_MODE;
    runAltHold(1);
}

TEST(AutopilotWingTest, AltHoldLocksTheAltitudeWithTheStickCentred)
{
    engageAltHold();
    const float entryCm = testAltitudeCm;
    testAltitudeCm += 300.0f;
    runAltHold(500);
    EXPECT_NEAR(entryCm, holdTargetCm(), 1.0f);
    EXPECT_TRUE(isAltHoldActive());
}

TEST(AutopilotWingTest, AltHoldPitchStickBackClimbsForwardDescends)
{
    engageAltHold();
    const float entryCm = testAltitudeCm;
    testPitchDeflection = -1.0f;
    runAltHold(50);
    // full stick is the configured climb rate: 2 m/s for half a second
    EXPECT_NEAR(entryCm + 100.0f, holdTargetCm(), 3.0f);

    engageAltHold();
    testPitchDeflection = 1.0f;
    runAltHold(50);
    EXPECT_NEAR(entryCm - 100.0f, holdTargetCm(), 3.0f);
}

TEST(AutopilotWingTest, AltHoldIgnoresTheStickInsideTheDeadband)
{
    engageAltHold();
    const float entryCm = testAltitudeCm;
    testPitchDeflection = -0.09f;
    runAltHold(100);
    EXPECT_NEAR(entryCm, holdTargetCm(), 1.0f);

    // half way through the travel past the deadband is half the rate
    testPitchDeflection = -0.55f;
    runAltHold(50);
    EXPECT_NEAR(entryCm + 50.0f, holdTargetCm(), 3.0f);
}

TEST(AutopilotWingTest, AltHoldTargetStaysWithinASecondOfTheAircraft)
{
    engageAltHold();
    const float entryCm = testAltitudeCm;
    testPitchDeflection = -1.0f;
    runAltHold(1000);
    EXPECT_LT(holdTargetCm() - entryCm, altHoldGetClimbRateCmS() + 5.0f);
}

TEST(AutopilotWingTest, AltHoldTargetComesBackTowardsAnAircraftThatFellBehind)
{
    engageAltHold();
    testAltitudeCm -= 300.0f;
    const float targetCm = holdTargetCm();
    testPitchDeflection = 1.0f;
    runAltHold(50);
    EXPECT_NEAR(targetCm - 100.0f, holdTargetCm(), 3.0f);

    // but climbing it stays put until the aircraft catches up
    testPitchDeflection = -1.0f;
    runAltHold(50);
    EXPECT_NEAR(targetCm - 100.0f, holdTargetCm(), 3.0f);
}

TEST(AutopilotWingTest, AltHoldFailsafeDescendsWhateverTheSticks)
{
    engageAltHold();
    const float entryCm = testAltitudeCm;
    testFailsafe = true;
    testPitchDeflection = -1.0f;
    runAltHold(50);
    EXPECT_NEAR(entryCm - 100.0f, holdTargetCm(), 3.0f);
}

TEST(AutopilotWingTest, AltHoldFailsafeDescendsWithoutAStickClimbRate)
{
    altHoldConfigMutable()->climbRate = 0;
    engageAltHold();
    altHoldConfigMutable()->climbRate = 0;
    const float entryCm = testAltitudeCm;
    runAltHold(50);
    EXPECT_FLOAT_EQ(entryCm, holdTargetCm());

    testFailsafe = true;
    runAltHold(50);
    EXPECT_NEAR(entryCm - 50.0f, holdTargetCm(), 2.0f);
}

TEST(AutopilotWingTest, AltHoldEmergencyDescentRunsAtItsOwnRate)
{
    engageAltHold();
    const float entryCm = testAltitudeCm;
    altHoldSetEmergencyDescent(true, 300.0f);
    runAltHold(50);
    EXPECT_NEAR(entryCm - 150.0f, holdTargetCm(), 4.0f);
    EXPECT_GE(autopilotWingGetClimbDemandCmS(), -300.0f);
    altHoldSetEmergencyDescent(false, 0.0f);
}

TEST(AutopilotWingTest, AltHoldEmergencyDescentComesDownAtLeastAMetreASecond)
{
    engageAltHold();
    const float entryCm = testAltitudeCm;
    altHoldSetEmergencyDescent(true, 30.0f);
    runAltHold(50);
    EXPECT_NEAR(entryCm - 50.0f, holdTargetCm(), 2.0f);
    altHoldSetEmergencyDescent(false, 0.0f);
}

TEST(AutopilotWingTest, AltHoldYieldsTheVerticalToANavLeg)
{
    engageAltHold();
    testNavActive = true;
    testNavCommand.includeAltitude = true;
    testNavAltitudeCm = testAltitudeCm + 2000.0f;
    testNavRateLimitCmS = 150.0f;
    testNavVelocityCmS = {{ 0.0f, 0.0f, 150.0f }};
    runAltHold(1);
    EXPECT_NEAR(testNavAltitudeCm, holdTargetCm(), 1.0f);
    EXPECT_FLOAT_EQ(150.0f, autopilotWingGetClimbDemandCmS());

    // and the hold keeps the leg's altitude once it ends
    testNavActive = false;
    runAltHold(1);
    EXPECT_NEAR(testNavAltitudeCm, holdTargetCm(), 1.0f);
}

TEST(AutopilotWingTest, AltHoldExitReleasesTheThrottle)
{
    engageAltHold();
    EXPECT_TRUE(autopilotThrottleValid());
    flightModeFlags = 0;
    runAltHold(1);
    EXPECT_FALSE(isAltHoldActive());
    EXPECT_FALSE(autopilotThrottleValid());
}

// Lateral guidance

static void setPositionCm(float northCm, float eastCm)
{
    testEstimate.position.v[ENU_N] = northCm;
    testEstimate.position.v[ENU_E] = eastCm;
}

static void setVelocityCmS(float northCmS, float eastCmS)
{
    testEstimate.velocity.v[ENU_N] = northCmS;
    testEstimate.velocity.v[ENU_E] = eastCmS;
}

static void runLateral(int steps)
{
    for (int i = 0; i < steps; i++) {
        testMicros += STEP_US;
        positionControl();
    }
}

static float bankDeg(void)
{
    return autopilotAngle[AI_ROLL];
}

// The loiter centres on the origin, where the estimate is when it engages.
static void engageLoiter(wingLoiterDirection_e direction)
{
    resetForTest();
    autopilotWingConfigMutable()->loiterDirection = direction;
    resetPositionControl(POSHOLD_TASK_RATE_HZ);
    setPositionCm(0.0f, 0.0f);
    setVelocityCmS(1500.0f, 0.0f);
    runLateral(1);
}

static vector2_t centreCm(void)
{
    const vector2_t errorCm = autopilotGetPositionErrorCm();
    return (vector2_t){{ testEstimate.position.v[ENU_E] + errorCm.v[ENU_E], testEstimate.position.v[ENU_N] + errorCm.v[ENU_N] }};
}

TEST(AutopilotWingTest, OnTheCircleBanksForTheTurnItsSpeedNeeds)
{
    // atan(15^2 / (g * 60)): 20.9 deg at 15 m/s on the default 60 m circle
    engageLoiter(WING_LOITER_RIGHT);
    setPositionCm(6000.0f, 0.0f);
    setVelocityCmS(0.0f, 1500.0f);      // north of the centre heading east: clockwise
    runLateral(100);
    EXPECT_NEAR(20.9f, bankDeg(), 0.3f);

    engageLoiter(WING_LOITER_LEFT);
    setPositionCm(6000.0f, 0.0f);
    setVelocityCmS(0.0f, -1500.0f);
    runLateral(100);
    EXPECT_NEAR(-20.9f, bankDeg(), 0.3f);
}

TEST(AutopilotWingTest, CentreToTheEastWhileFlyingNorthBanksRight)
{
    engageLoiter(WING_LOITER_RIGHT);
    setPositionCm(0.0f, -20000.0f);
    setVelocityCmS(1500.0f, 0.0f);
    runLateral(100);
    EXPECT_GT(bankDeg(), 10.0f);

    engageLoiter(WING_LOITER_RIGHT);
    setPositionCm(0.0f, 20000.0f);
    runLateral(100);
    EXPECT_LT(bankDeg(), -10.0f);
}

TEST(AutopilotWingTest, HeadingOutTheWrongWayRoundItTurnsTheLoitersWay)
{
    // well inside the circle, the radius error alone would turn it away from the centre
    engageLoiter(WING_LOITER_RIGHT);
    setPositionCm(1000.0f, 0.0f);
    setVelocityCmS(300.0f, -1470.0f);   // north of the centre heading out and west: anticlockwise
    runLateral(100);
    EXPECT_GT(bankDeg(), 5.0f);

    engageLoiter(WING_LOITER_LEFT);
    setPositionCm(1000.0f, 0.0f);
    setVelocityCmS(300.0f, 1470.0f);
    runLateral(100);
    EXPECT_LT(bankDeg(), -5.0f);
}

TEST(AutopilotWingTest, ComingInTheWrongWayRoundItTurnsOntoTheCircle)
{
    // east of the centre heading in and drifting north, it turns left to join the circle heading south
    engageLoiter(WING_LOITER_RIGHT);
    setPositionCm(0.0f, 8000.0f);
    setVelocityCmS(100.0f, -1500.0f);
    runLateral(100);
    EXPECT_LT(bankDeg(), -5.0f);
}

TEST(AutopilotWingTest, CloseOutsideTheCircleTheWrongWayRoundItTurnsTheLoitersWayRatherThanAtTheCentre)
{
    // 20 m outside the circle east of the centre, heading north: anticlockwise
    engageLoiter(WING_LOITER_RIGHT);
    setPositionCm(0.0f, 8000.0f);
    setVelocityCmS(1500.0f, 0.0f);
    runLateral(100);
    EXPECT_GT(bankDeg(), 10.0f);

    engageLoiter(WING_LOITER_LEFT);
    setPositionCm(0.0f, 8000.0f);
    setVelocityCmS(-1500.0f, 0.0f);
    runLateral(100);
    EXPECT_LT(bankDeg(), -10.0f);
}

TEST(AutopilotWingTest, ANarrowerBankWidensTheLoiterToACircleItCanFly)
{
    // 10 deg, with a fifth of it in hand, circles 15 m/s on 162 m: on that circle it banks for it, steady
    engageLoiter(WING_LOITER_RIGHT);
    autopilotWingLimits_t limits;
    autopilotWingDefaultLimits(&limits);
    limits.bankLimitDeg = 10.0f;
    autopilotWingSetLimits(AP_WING_LIMITS_DESCENT, &limits);
    const float radiusCm = 100.0f * sq(15.0f) / (G_ACCELERATION * tanf(DEGREES_TO_RADIANS(8.0f)));
    setPositionCm(0.0f, -radiusCm);
    setVelocityCmS(1500.0f, 0.0f);
    runLateral(200);
    EXPECT_NEAR(8.0f, bankDeg(), 0.3f);
    autopilotWingClearLimits(AP_WING_LIMITS_DESCENT);
}

TEST(AutopilotWingTest, BankStaysWithinTheLimit)
{
    engageLoiter(WING_LOITER_RIGHT);
    setPositionCm(0.0f, -20000.0f);
    runLateral(200);
    EXPECT_FLOAT_EQ((float)autopilotWingConfig()->maxBank, bankDeg());

    engageLoiter(WING_LOITER_RIGHT);
    autopilotWingConfigMutable()->maxBank = 20;
    setPositionCm(0.0f, -20000.0f);
    runLateral(200);
    EXPECT_FLOAT_EQ(20.0f, bankDeg());
}

TEST(AutopilotWingTest, BankSlewsRatherThanSteps)
{
    engageLoiter(WING_LOITER_RIGHT);
    setPositionCm(0.0f, -20000.0f);
    const float before = bankDeg();
    runLateral(10);
    EXPECT_NEAR(4.5f, bankDeg() - before, 0.01f);    // 45 deg/s for 0.1 s
}

TEST(AutopilotWingTest, EntryContinuesFromTheCurrentBank)
{
    engageLoiter(WING_LOITER_RIGHT);
    setPositionCm(0.0f, -20000.0f);
    runLateral(10);
    setBankDeg(-30.0f);
    resetPositionControl(POSHOLD_TASK_RATE_HZ);
    runLateral(1);
    EXPECT_NEAR(-30.0f, bankDeg(), 0.46f);
}

TEST(AutopilotWingTest, BelowWalkingPaceTheNoseGivesTheDirection)
{
    engageLoiter(WING_LOITER_RIGHT);
    setPositionCm(0.0f, -20000.0f);
    setVelocityCmS(0.0f, 0.0f);
    attitude.values.yaw = 0;            // nose north, centre to the east
    runLateral(100);
    EXPECT_GT(bankDeg(), 10.0f);

    attitude.values.yaw = 1800;         // nose south
    runLateral(200);
    EXPECT_LT(bankDeg(), -10.0f);
}

TEST(AutopilotWingTest, LoiterCentresWhereItEngages)
{
    resetForTest();
    resetPositionControl(POSHOLD_TASK_RATE_HZ);
    setPositionCm(1234.0f, -567.0f);
    runLateral(1);
    setPositionCm(0.0f, 0.0f);
    EXPECT_NEAR(1234.0f, centreCm().v[ENU_N], 0.1f);
    EXPECT_NEAR(-567.0f, centreCm().v[ENU_E], 0.1f);
    EXPECT_TRUE(isAutopilotInControl());
}

TEST(AutopilotWingTest, PilotSteersAndTheLoiterRecentresWhereTheyLetGo)
{
    engageLoiter(WING_LOITER_RIGHT);
    setSticksActiveStatus(true);
    setPositionCm(5000.0f, 3000.0f);
    runLateral(10);
    EXPECT_FALSE(isAutopilotInControl());

    setSticksActiveStatus(false);
    runLateral(1);
    EXPECT_TRUE(isAutopilotInControl());
    EXPECT_NEAR(5000.0f, centreCm().v[ENU_N], 0.1f);
    EXPECT_NEAR(3000.0f, centreCm().v[ENU_E], 0.1f);
}

TEST(AutopilotWingTest, WithoutAPositionItCirclesAndKeepsTheCentre)
{
    engageLoiter(WING_LOITER_RIGHT);
    setPositionCm(3000.0f, 4000.0f);
    testEstimate.isValidXY = false;
    runLateral(100);
    EXPECT_NEAR(20.9f, bankDeg(), 0.3f);   // the still air bank for the cruise speed and radius
    EXPECT_FALSE(positionControl());
    EXPECT_TRUE(isAutopilotInControl());

    autopilotWingConfigMutable()->loiterDirection = WING_LOITER_LEFT;
    runLateral(200);
    EXPECT_NEAR(-20.9f, bankDeg(), 0.3f);

    testEstimate.isValidXY = true;
    EXPECT_TRUE(positionControl());
    EXPECT_NEAR(0.0f, centreCm().v[ENU_N], 0.1f);
    EXPECT_NEAR(0.0f, centreCm().v[ENU_E], 0.1f);
}

TEST(AutopilotWingTest, WithoutAPositionALargeCircleStillTurns)
{
    engageLoiter(WING_LOITER_RIGHT);
    autopilotWingConfigMutable()->loiterRadius = 2000;
    testEstimate.isValidXY = false;
    runLateral(100);
    EXPECT_NEAR(10.0f, bankDeg(), 0.1f);
}

TEST(AutopilotWingTest, PositionControlAlwaysTicksTheNavTarget)
{
    engageLoiter(WING_LOITER_RIGHT);
    testNavUpdates = 0;
    runLateral(3);
    testEstimate.isValidXY = false;
    runLateral(3);
    setSticksActiveStatus(true);
    runLateral(3);
    EXPECT_EQ(9, testNavUpdates);
}

// Pos hold

static void runPosHold(int steps)
{
    for (int i = 0; i < steps; i++) {
        testMicros += STEP_US;
        updatePosHold(testMicros);
    }
}

static void engagePosHold(void)
{
    resetForTest();
    posHoldInit();
    setPositionCm(1000.0f, 2000.0f);
    setVelocityCmS(1500.0f, 0.0f);
    flightModeFlags = ALT_HOLD_MODE | POS_HOLD_MODE;
    runPosHold(1);
}

TEST(AutopilotWingTest, PosHoldLoitersAboutWhereItEngaged)
{
    engagePosHold();
    EXPECT_TRUE(isAutopilotInControl());
    EXPECT_NEAR(1000.0f, centreCm().v[ENU_N], 0.1f);
    EXPECT_NEAR(2000.0f, centreCm().v[ENU_E], 0.1f);
    EXPECT_FALSE(posHoldFailure());
    EXPECT_TRUE(posHoldReady());
}

TEST(AutopilotWingTest, PosHoldRollStickTakesOverPastTheDeadband)
{
    engagePosHold();
    testRollDeflection = 0.09f;
    runPosHold(1);
    EXPECT_TRUE(isAutopilotInControl());

    testRollDeflection = -0.2f;
    runPosHold(1);
    EXPECT_FALSE(isAutopilotInControl());

    // failsafe ignores the sticks
    testFailsafe = true;
    runPosHold(1);
    EXPECT_TRUE(isAutopilotInControl());
}

TEST(AutopilotWingTest, PosHoldPitchStickLeavesTheLoiterAlone)
{
    engagePosHold();
    testPitchDeflection = 1.0f;
    runPosHold(1);
    EXPECT_TRUE(isAutopilotInControl());
}

TEST(AutopilotWingTest, PosHoldReportsALostPosition)
{
    engagePosHold();
    testEstimate.isValidXY = false;
    runPosHold(1);
    EXPECT_TRUE(posHoldFailure());
    EXPECT_FALSE(posHoldReady());
    EXPECT_TRUE(isAutopilotInControl());
}

TEST(AutopilotWingTest, PosHoldExitHandsTheRollBack)
{
    engagePosHold();
    flightModeFlags = ALT_HOLD_MODE;
    runPosHold(1);
    EXPECT_FALSE(isAutopilotInControl());
    EXPECT_FALSE(posHoldFailure());
}

// Coming down to land where it is

static void runHolds(int steps)
{
    for (int i = 0; i < steps; i++) {
        testMicros += STEP_US;
        updateAltHold(testMicros);
        updatePosHold(testMicros);
    }
}

// Circling clockwise on the default 60 m circle about the origin, at altitudeM.
static void engageHoldsOnTheCircle(float altitudeM)
{
    resetForTest();
    altHoldInit();
    posHoldInit();
    testAltitudeCm = altitudeM * 100.0f;
    setPositionCm(0.0f, 0.0f);
    setVelocityCmS(1500.0f, 0.0f);
    flightModeFlags = ALT_HOLD_MODE | POS_HOLD_MODE;
    runHolds(1);
    setPositionCm(6000.0f, 0.0f);
    setVelocityCmS(0.0f, 1500.0f);
}

TEST(AutopilotWingTest, AFailsafeLandingLevelsTheWingsNearTheGround)
{
    // below three flare heights
    engageHoldsOnTheCircle(10.0f);
    testFailsafe = true;
    runHolds(100);
    EXPECT_NEAR(20.9f, bankDeg(), 0.3f);

    testAltitudeCm = 800.0f;
    runHolds(100);
    EXPECT_FLOAT_EQ(0.0f, bankDeg());
}

TEST(AutopilotWingTest, TheWingsLevelHigherForAHigherFlare)
{
    engageHoldsOnTheCircle(13.0f);
    autopilotWingConfigMutable()->landFlareHeight = 500;
    testFailsafe = true;
    runHolds(100);
    EXPECT_FLOAT_EQ(0.0f, bankDeg());
}

TEST(AutopilotWingTest, AfterAThrowTheWingsLevelOverTheGroundNotTheHand)
{
    // armed in the hand 1.5 m up, 8 m above it is 9.5 m above the ground
    engageHoldsOnTheCircle(8.0f);
    testThrown = true;
    testFailsafe = true;
    runHolds(100);
    EXPECT_NEAR(20.9f, bankDeg(), 0.3f);

    testAltitudeCm = 700.0f;
    runHolds(100);
    EXPECT_FLOAT_EQ(0.0f, bankDeg());
}

TEST(AutopilotWingTest, AnEmergencyDescentLevelsTheWingsNearTheGround)
{
    engageHoldsOnTheCircle(8.0f);
    altHoldSetEmergencyDescent(true, 200.0f);
    runHolds(100);
    EXPECT_FLOAT_EQ(0.0f, bankDeg());

    // without a position, not knowing the ground under it, it circles wide
    testEstimate.isValidXY = false;
    runHolds(100);
    EXPECT_NEAR(10.0f, bankDeg(), 0.01f);
    altHoldSetEmergencyDescent(false, 0.0f);
}

TEST(AutopilotWingTest, OnlyALandingLevelsTheWingsLow)
{
    // the pilot's own hold
    engageHoldsOnTheCircle(8.0f);
    runHolds(100);
    EXPECT_NEAR(20.9f, bankDeg(), 0.3f);

    // a mission flying through a failsafe owns the descent, and turns
    engageHoldsOnTheCircle(8.0f);
    testFailsafe = true;
    testNavActive = true;
    testNavCommand.includeAltitude = true;
    testNavCommand.track = NAV_TRACK_LOITER;
    testNavCommand.loiterRadiusM = 60.0f;
    testNavCommand.loiterDirection = 1;
    testNavAltitudeCm = testAltitudeCm;
    runHolds(100);
    EXPECT_NEAR(20.9f, bankDeg(), 0.3f);

    // and a landing that has ended leaves the hold free to turn again
    engageHoldsOnTheCircle(8.0f);
    testFailsafe = true;
    runHolds(100);
    testFailsafe = false;
    runHolds(100);
    EXPECT_NEAR(20.9f, bankDeg(), 0.3f);
}

// On the ground after coming down to land where it is: still, sink stopped, at the arming point's height.
static void sitOnTheGround(void)
{
    testAltitudeCm = 50.0f;
    testEstimate.velocity = {};
    acc.accMagnitude = 1.0f;
    gyro = {};
}

TEST(AutopilotWingTest, AFailsafeDescentIdlesDownItsPath)
{
    engageHoldsOnTheCircle(20.0f);
    testFailsafe = true;
    testClimbRateCmS = -200.0f;
    runHolds(300);
    EXPECT_LT(getAutopilotThrottle(), 0.05f);
    EXPECT_FALSE(autopilotWingMotorStopRequested());
}

TEST(AutopilotWingTest, AFailsafeLandingFlaresOverHomesGround)
{
    engageHoldsOnTheCircle(5.0f);
    testFailsafe = true;
    testEstimate.velocity.v[ENU_U] = -200.0f;
    testClimbRateCmS = -200.0f;
    runHolds(10);
    EXPECT_FALSE(autopilotWingMotorStopRequested());

    // once low for long enough to be no spike in the height
    testAltitudeCm = 290.0f;
    runHolds(1);
    EXPECT_FALSE(autopilotWingMotorStopRequested());
    runHolds(20);
    EXPECT_TRUE(autopilotWingMotorStopRequested());
    // the sink eases off with the height from the glide's at the cruise speed down the default 8 deg slope
    testAltitudeCm = 150.0f;
    runHolds(1);
    EXPECT_NEAR(-0.5f * 1500.0f * tanf(DEGREES_TO_RADIANS(8.0f)), autopilotWingGetClimbDemandCmS(), 1.0f);
}

TEST(AutopilotWingTest, AFailsafeFlareStartsFromTheAttitudeNoHigherThanItGlidesAt)
{
    // the controller pushing the nose right down to start the descent, the attitude 6 deg nose down
    engageHoldsOnTheCircle(20.0f);
    testFailsafe = true;
    runHolds(200);
    ASSERT_FLOAT_EQ(-(float)autopilotWingConfig()->maxDiveAngle, -pitchTargetDeg());
    attitude.values.pitch = 60;
    testAltitudeCm = 250.0f;
    testEstimate.velocity.v[ENU_U] = -150.0f;
    testClimbRateCmS = -150.0f;
    runHolds(100);
    ASSERT_TRUE(autopilotWingMotorStopRequested());
    EXPECT_NEAR(-6.0f, -pitchTargetDeg(), 0.1f);

    // pitched up under power, no higher than the throttle for its path would idle at
    engageHoldsOnTheCircle(20.0f);
    testFailsafe = true;
    runHolds(200);
    attitude.values.pitch = -30;
    testAltitudeCm = 250.0f;
    testEstimate.velocity.v[ENU_U] = -150.0f;
    testClimbRateCmS = -150.0f;
    runHolds(100);
    ASSERT_TRUE(autopilotWingMotorStopRequested());
    EXPECT_NEAR(-RADIANS_TO_DEGREES(atanf(150.0f / 1500.0f)) + 2.0f, -pitchTargetDeg(), 0.1f);
}

TEST(AutopilotWingTest, AwayFromHomeAFailsafeLandingCirclesDownGently)
{
    // the circle needs 20.9 deg
    engageHoldsOnTheCircle(40.0f);
    setPositionCm(50000.0f, 0.0f);
    testFailsafe = true;
    runHolds(100);
    EXPECT_FLOAT_EQ(15.0f, bankDeg());
}

TEST(AutopilotWingTest, AFailsafeLandingAwayFromHomeComesDownUnderPowerOntoGroundOfUnknownHeight)
{
    // 500 m out, below where home's ground would be
    engageHoldsOnTheCircle(20.0f);
    setPositionCm(50000.0f, 0.0f);
    testFailsafe = true;
    testAltitudeCm = -2000.0f;
    testEstimate.velocity.v[ENU_U] = -60.0f;
    testClimbRateCmS = -60.0f;
    runHolds(300);
    EXPECT_FALSE(autopilotWingMotorStopRequested());
    EXPECT_FLOAT_EQ(-autopilotWingConfig()->landFlareSink * 10.0f, autopilotWingGetClimbDemandCmS());
    EXPECT_FLOAT_EQ(10.0f, bankDeg());

    // the sink stopping does not close the throttle; the ground braking it does
    testEstimate.velocity = {};
    runHolds(100);
    EXPECT_FALSE(autopilotWingMotorStopRequested());
    brake(0.4f);
    runHolds(25);
    EXPECT_TRUE(autopilotWingMotorStopRequested());
}

// On ground 5 m above home's, above where a descent flares: still, its sink stopped.
static void sitOnHigherGround(void)
{
    testAltitudeCm = 500.0f;
    testEstimate.velocity = {};
    acc.accMagnitude = 1.0f;
    gyro = {};
}

TEST(AutopilotWingTest, TheThrottleClosesAsSoonAsTheDescentMeetsTheGround)
{
    engageHoldsOnTheCircle(5.0f);
    testFailsafe = true;
    testEstimate.velocity.v[ENU_U] = -150.0f;
    runHolds(10);
    EXPECT_FALSE(autopilotWingMotorStopRequested());

    sitOnHigherGround();
    runHolds(1);
    EXPECT_TRUE(autopilotWingMotorStopRequested());
    EXPECT_EQ(0, testDisarms);
}

TEST(AutopilotWingTest, TheThrottleStaysClosedOnTheGroundButNotAfterAWrongGuessInTheAir)
{
    engageHoldsOnTheCircle(5.0f);
    testFailsafe = true;
    sitOnHigherGround();
    runHolds(500);
    ASSERT_TRUE(autopilotWingMotorStopRequested());

    // bounced back up, and falling back
    testEstimate.velocity.v[ENU_U] = 100.0f;
    runHolds(50);
    EXPECT_TRUE(autopilotWingMotorStopRequested());
    testEstimate.velocity.v[ENU_U] = -100.0f;
    runHolds(150);
    EXPECT_TRUE(autopilotWingMotorStopRequested());

    // still coming down two seconds after the cut, it is flying: the throttle comes back
    runHolds(55);
    EXPECT_FALSE(autopilotWingMotorStopRequested());
}

// Sliding at speedCmS north, coming down at sinkCmS, for steps.
static void slide(float speedCmS, float sinkCmS, int steps)
{
    for (int i = 0; i < steps; i++) {
        setVelocityCmS(speedCmS, 0.0f);
        testEstimate.velocity.v[ENU_U] = -sinkCmS;
        runHolds(1);
    }
}

TEST(AutopilotWingTest, SlidingDownASlopeAfterTheContactTheThrottleStaysClosed)
{
    // met the ground, sliding down a 10 deg slope as the ground brakes it at 0.2 g
    engageHoldsOnTheCircle(5.0f);
    testFailsafe = true;
    sitOnHigherGround();
    runHolds(1);
    ASSERT_TRUE(autopilotWingMotorStopRequested());
    brake(0.2f);
    for (int i = 0; i < 400; i++) {
        const float speedCmS = fmaxf(1000.0f - 200.0f * i * STEP_US * 1e-6f, 0.0f);
        slide(speedCmS, speedCmS * tanf(DEGREES_TO_RADIANS(10.0f)), 1);
        ASSERT_TRUE(autopilotWingMotorStopRequested()) << i;
    }

    // shaken by the ground
    engageHoldsOnTheCircle(5.0f);
    testFailsafe = true;
    sitOnHigherGround();
    runHolds(1);
    ASSERT_TRUE(autopilotWingMotorStopRequested());
    for (int i = 0; i < 400; i++) {
        acc.accMagnitude = (i % 20 == 0) ? 1.4f : 1.0f;
        slide(500.0f, 80.0f, 1);
        ASSERT_TRUE(autopilotWingMotorStopRequested()) << i;
    }

    // gliding on in the air instead, neither braked nor shaken, the throttle comes back
    engageHoldsOnTheCircle(5.0f);
    testFailsafe = true;
    sitOnHigherGround();
    runHolds(1);
    ASSERT_TRUE(autopilotWingMotorStopRequested());
    acc.accMagnitude = 1.0f;
    slide(1000.0f, 80.0f, 205);
    EXPECT_FALSE(autopilotWingMotorStopRequested());
}

TEST(AutopilotWingTest, AnImpactClosesTheThrottle)
{
    engageHoldsOnTheCircle(5.0f);
    testFailsafe = true;
    testEstimate.velocity.v[ENU_U] = -150.0f;
    runHolds(10);
    ASSERT_FALSE(autopilotWingMotorStopRequested());
    acc.accMagnitude = 2.5f;
    runHolds(1);
    acc.accMagnitude = 1.0f;
    EXPECT_TRUE(autopilotWingMotorStopRequested());
    testEstimate.velocity = {};
    runHolds(300);
    EXPECT_TRUE(autopilotWingMotorStopRequested());
}

TEST(AutopilotWingTest, OnlyALandingLowDownClosesTheThrottle)
{
    // still in the air above the height the wings level from
    engageHoldsOnTheCircle(10.0f);
    testFailsafe = true;
    testEstimate.velocity = {};
    runHolds(10);
    EXPECT_FALSE(autopilotWingMotorStopRequested());

    // the pilot's own hold
    engageHoldsOnTheCircle(5.0f);
    sitOnTheGround();
    runHolds(10);
    EXPECT_FALSE(autopilotWingMotorStopRequested());

    // a nav leg's own landing owns the throttle
    engageHoldsOnTheCircle(5.0f);
    testFailsafe = true;
    testNavActive = true;
    testNavCommand.includeAltitude = true;
    testNavVelocityCmS = {{ 0.0f, 0.0f, -150.0f }};
    sitOnTheGround();
    runHolds(10);
    EXPECT_FALSE(autopilotWingMotorStopRequested());
    testNavActive = false;
    testNavVelocityCmS = {};

    // and leaving the descent gives the throttle back
    engageHoldsOnTheCircle(5.0f);
    testFailsafe = true;
    sitOnTheGround();
    runHolds(10);
    ASSERT_TRUE(autopilotWingMotorStopRequested());
    flightModeFlags = 0;
    runHolds(1);
    EXPECT_FALSE(autopilotWingMotorStopRequested());
}

TEST(AutopilotWingTest, AFailsafeLandingDisarmsOnceStillOnTheGround)
{
    engageHoldsOnTheCircle(5.0f);
    testFailsafe = true;
    runHolds(10);
    sitOnTheGround();
    runHolds(195);      // ap_landing_detection_time 2 s
    EXPECT_EQ(0, testDisarms);
    runHolds(10);
    EXPECT_LT(0, testDisarms);
}

TEST(AutopilotWingTest, AnEmergencyDescentDisarmsOnceStillOnTheGround)
{
    engageHoldsOnTheCircle(5.0f);
    altHoldSetEmergencyDescent(true, 200.0f);
    runHolds(10);
    sitOnTheGround();
    testEstimate.isValidXY = false;     // without a position too
    runHolds(205);
    EXPECT_LT(0, testDisarms);
    altHoldSetEmergencyDescent(false, 0.0f);
}

TEST(AutopilotWingTest, OnlyALandingDisarmsOnTheGround)
{
    // the pilot's own hold, sitting still: nothing commands it down
    engageHoldsOnTheCircle(5.0f);
    sitOnTheGround();
    runHolds(500);
    EXPECT_EQ(0, testDisarms);

    // a nav leg owns the descent, and its landing its own touchdown
    engageHoldsOnTheCircle(5.0f);
    testFailsafe = true;
    testNavActive = true;
    testNavCommand.includeAltitude = true;
    testNavAltitudeCm = 0.0f;
    testNavVelocityCmS = {{ 0.0f, 0.0f, -150.0f }};
    sitOnTheGround();
    runHolds(500);
    EXPECT_EQ(0, testDisarms);
}

TEST(AutopilotWingTest, TheHighestItHasBeenIsNotedWhateverTheModeAndWhetherOrNotARescueIsConfigured)
{
    resetForTest();
    posHoldInit();
    ENABLE_ARMING_FLAG(ARMED);
    testAltitudeCm = 4000.0f;
    runHolds(1);
    testAltitudeCm = 2500.0f;
    runHolds(1);
    EXPECT_FLOAT_EQ(4000.0f, gpsRescueGetMaxAltitudeCm());

    // forgotten once disarmed
    DISABLE_ARMING_FLAG(ARMED);
    runHolds(1);
    EXPECT_FLOAT_EQ(0.0f, gpsRescueGetMaxAltitudeCm());
}

TEST(AutopilotWingTest, ALandingSinkingThroughTheGroundHeightIsNotDown)
{
    engageHoldsOnTheCircle(5.0f);
    testFailsafe = true;
    sitOnTheGround();
    testEstimate.velocity.v[ENU_U] = -150.0f;
    runHolds(500);
    EXPECT_EQ(0, testDisarms);
}

TEST(AutopilotWingTest, AFailsafeLandingOnGroundAboveHomeDisarmsOnceStillAndFlat)
{
    engageHoldsOnTheCircle(25.0f);
    testFailsafe = true;
    runHolds(10);
    sitOnTheGround();
    testAltitudeCm = 2000.0f;           // the ground is 20 m above where it was armed
    attitude.values.roll = 30;
    runHolds(295);
    EXPECT_EQ(0, testDisarms);
    runHolds(10);
    EXPECT_LT(0, testDisarms);

    // banked as it would be circling, it is still in the air
    engageHoldsOnTheCircle(25.0f);
    testFailsafe = true;
    runHolds(10);
    sitOnTheGround();
    testAltitudeCm = 2000.0f;
    attitude.values.roll = 200;
    runHolds(1000);
    EXPECT_EQ(0, testDisarms);
}

// Line guidance

extern "C" float wingLineLateralAccelCmSS(const vector2_t *pos, const vector2_t *vel, const vector2_t *a, const vector2_t *b,
                                          float periodS, float damping, int8_t *turnLatch);

// A line from the origin to 1 km north, at the default guidance period and damping. ENU cm.
static float lineAccel(float eastCm, float northCm, float velEastCmS, float velNorthCmS, int8_t *latch = nullptr)
{
    int8_t unlatched = 0;
    const vector2_t pos = {{ eastCm, northCm }};
    const vector2_t vel = {{ velEastCmS, velNorthCmS }};
    const vector2_t a = {{ 0.0f, 0.0f }};
    const vector2_t b = {{ 0.0f, 100000.0f }};
    return wingLineLateralAccelCmSS(&pos, &vel, &a, &b, 13.0f, 0.75f, latch ? latch : &unlatched);
}

TEST(AutopilotWingTest, OnTheLineAndAlongItNoTurn)
{
    EXPECT_NEAR(0.0f, lineAccel(0.0f, 50000.0f, 0.0f, 1500.0f), 1.0f);
}

TEST(AutopilotWingTest, RightOfTheLineTurnsLeftLeftOfItTurnsRight)
{
    EXPECT_LT(lineAccel(2000.0f, 50000.0f, 0.0f, 1500.0f), -100.0f);
    EXPECT_GT(lineAccel(-2000.0f, 50000.0f, 0.0f, 1500.0f), 100.0f);
}

TEST(AutopilotWingTest, ACrossTrackErrorClosesAtNoSteeperThanSixtyDegrees)
{
    // 4 zeta^2 V^2 / L1 sin(60): heading along the line 200 m off it, and heading in at 60 degrees
    // it is already closing as fast as it may
    const float l1Cm = 0.75f * 13.0f * 1500.0f / M_PIf;
    EXPECT_NEAR(-4.0f * sq(0.75f) * sq(1500.0f) / l1Cm * sinf(M_PIf / 3.0f), lineAccel(20000.0f, 50000.0f, 0.0f, 1500.0f), 5.0f);
    const float inRad = DEGREES_TO_RADIANS(60.0f);
    EXPECT_NEAR(0.0f, lineAccel(20000.0f, 50000.0f, -1500.0f * sinf(inRad), 1500.0f * cosf(inRad)), 5.0f);
}

TEST(AutopilotWingTest, AnEastboundLineNorthOfTheAircraftTurnsLeft)
{
    // pins the frame: x east, y north, a right turn positive
    const vector2_t pos = {{ 50000.0f, -3000.0f }};
    const vector2_t vel = {{ 1500.0f, 0.0f }};
    const vector2_t a = {{ 0.0f, 0.0f }};
    const vector2_t b = {{ 100000.0f, 0.0f }};
    int8_t latch = 0;
    EXPECT_LT(wingLineLateralAccelCmSS(&pos, &vel, &a, &b, 13.0f, 0.75f, &latch), -100.0f);
}

TEST(AutopilotWingTest, FarBehindTheStartItFliesToTheStartFirst)
{
    // 300 m south of the start, 50 m east of the line, heading north-west: the line alone would turn
    // it gently left onto the line, but the start is off to its right
    EXPECT_GT(lineAccel(5000.0f, -30000.0f, -1000.0f, 1100.0f), 300.0f);
}

TEST(AutopilotWingTest, PastTheEndItCarriesOnAlongTheLine)
{
    EXPECT_NEAR(0.0f, lineAccel(0.0f, 110000.0f, 0.0f, 1500.0f), 1.0f);
}

TEST(AutopilotWingTest, AReversalKeepsTurningTheWayItStarted)
{
    int8_t latch = 0;
    EXPECT_GT(lineAccel(0.0f, 50000.0f, -10.0f, -1500.0f, &latch), 100.0f);
    EXPECT_GT(lineAccel(0.0f, 50000.0f, 10.0f, -1500.0f, &latch), 100.0f);

    // until it is round far enough to be clear of the reverse heading
    EXPECT_LT(lineAccel(0.0f, 50000.0f, 1500.0f, 100.0f, &latch), -100.0f);
    latch = 0;
    EXPECT_LT(lineAccel(0.0f, 50000.0f, 10.0f, -1500.0f, &latch), -100.0f);
}

TEST(AutopilotWingTest, LookAheadFollowsTheGroundspeedWithAFloor)
{
    resetForTest();
    setVelocityCmS(1500.0f, 0.0f);
    EXPECT_NEAR(0.75f * 13.0f * 15.0f / M_PIf, autopilotWingL1DistanceM(), 0.05f);
    setVelocityCmS(0.0f, 0.0f);
    EXPECT_NEAR(10.0f, autopilotWingL1DistanceM(), 0.01f);
}

TEST(AutopilotWingTest, TurnDistanceScalesWithTheTurnUpToTheLookAhead)
{
    resetForTest();
    setVelocityCmS(1500.0f, 0.0f);
    const float l1M = autopilotWingL1DistanceM();
    EXPECT_NEAR(0.5f * l1M, autopilotWingTurnDistanceM(45.0f), 0.01f);
    EXPECT_NEAR(l1M, autopilotWingTurnDistanceM(-90.0f), 0.01f);
    EXPECT_NEAR(l1M, autopilotWingTurnDistanceM(150.0f), 0.01f);
    EXPECT_NEAR(0.0f, autopilotWingTurnDistanceM(0.0f), 0.01f);
}

TEST(AutopilotWingTest, ALineTurnedOntoIsSettledOnInTwoLookAheadsAtTheCruiseSpeed)
{
    resetForTest();
    setVelocityCmS(500.0f, 0.0f);
    EXPECT_NEAR(2.0f * 0.75f * 13.0f * 15.0f / M_PIf, autopilotWingLineSettleDistanceM(), 0.05f);
}

TEST(AutopilotWingTest, MinimumTurnRadiusLeavesSomeBankInHand)
{
    // 15 m/s at 0.8 of the 35 degree bank limit
    resetForTest();
    EXPECT_NEAR(225.0f / (9.80665f * tanf(DEGREES_TO_RADIANS(28.0f))), autopilotWingMinTurnRadiusM(), 0.2f);
}

// Navigation targets

static void setNavTarget(positionNavTrack_e track, float eastM, float northM, uint32_t sequence = 1)
{
    testNavActive = true;
    testNavCommand.active = true;
    testNavCommand.sequence = sequence;
    testNavCommand.track = track;
    testNavCommand.targetPosEfM.v[ENU_E] = eastM;
    testNavCommand.targetPosEfM.v[ENU_N] = northM;
}

TEST(AutopilotWingTest, ANavLineIsFlownFromItsStart)
{
    engageLoiter(WING_LOITER_RIGHT);
    setNavTarget(NAV_TRACK_LINE, 0.0f, 1000.0f);
    testNavCommand.trackStartEfM = (vector2_t){{ 0.0f, 0.0f }};
    setPositionCm(50000.0f, 3000.0f);     // 30 m east of the line, heading north
    setVelocityCmS(1500.0f, 0.0f);
    runLateral(100);
    EXPECT_LT(bankDeg(), -10.0f);

    testNavCommand.trackStartEfM = (vector2_t){{ 60.0f, 0.0f }};    // now 30 m west of it
    testNavCommand.targetPosEfM.v[ENU_E] = 60.0f;
    runLateral(200);
    EXPECT_GT(bankDeg(), 10.0f);
}

TEST(AutopilotWingTest, ANavLoiterCirclesTheTargetItsOwnWay)
{
    engageLoiter(WING_LOITER_RIGHT);
    setNavTarget(NAV_TRACK_LOITER, 200.0f, 300.0f);
    testNavCommand.loiterRadiusM = 100.0f;
    testNavCommand.loiterDirection = -1;
    setPositionCm(40000.0f, 20000.0f);      // north of the target heading west: anticlockwise
    setVelocityCmS(0.0f, -1500.0f);
    runLateral(100);
    EXPECT_NEAR(-RADIANS_TO_DEGREES(atanf(225.0f / (9.80665f * 100.0f))), bankDeg(), 0.3f);
}

TEST(AutopilotWingTest, ANavPointIsFlownFromWhereTheAircraftWasWhenItWasSet)
{
    engageLoiter(WING_LOITER_RIGHT);
    setNavTarget(NAV_TRACK_POINT, 0.0f, 1000.0f, 1);
    setPositionCm(0.0f, 0.0f);
    setVelocityCmS(1500.0f, 0.0f);
    runLateral(1);

    // 100 m east of the line from where it was set
    setPositionCm(50000.0f, 10000.0f);
    runLateral(200);
    EXPECT_LT(bankDeg(), -20.0f);

    // a new target starts a new line from where the aircraft is now
    setNavTarget(NAV_TRACK_POINT, 100.0f, 1000.0f, 2);
    runLateral(200);
    EXPECT_NEAR(0.0f, bankDeg(), 0.5f);
}

TEST(AutopilotWingTest, WhenTheNavTargetEndsItLoitersWhereItIs)
{
    engageLoiter(WING_LOITER_RIGHT);
    setNavTarget(NAV_TRACK_LINE, 0.0f, 1000.0f);
    setPositionCm(40000.0f, 7000.0f);
    runLateral(10);
    testNavActive = false;
    testNavCommand.active = false;
    runLateral(1);
    EXPECT_NEAR(40000.0f, centreCm().v[ENU_N], 0.1f);
    EXPECT_NEAR(7000.0f, centreCm().v[ENU_E], 0.1f);
}

TEST(AutopilotWingTest, ThePilotStillSteersANavTarget)
{
    engageLoiter(WING_LOITER_RIGHT);
    setNavTarget(NAV_TRACK_LINE, 0.0f, 1000.0f);
    setSticksActiveStatus(true);
    runLateral(1);
    EXPECT_FALSE(isAutopilotInControl());
    setSticksActiveStatus(false);
}

// Limits a navigation leg sets

TEST(AutopilotWingTest, ALegsBankLimitNarrowsTheBank)
{
    engageLoiter(WING_LOITER_RIGHT);
    autopilotWingLimits_t limits;
    autopilotWingDefaultLimits(&limits);
    limits.bankLimitDeg = 15.0f;
    autopilotWingSetLimits(AP_WING_LIMITS_LEG, &limits);
    setPositionCm(0.0f, -20000.0f);
    runLateral(200);
    EXPECT_FLOAT_EQ(15.0f, bankDeg());

    autopilotWingClearLimits(AP_WING_LIMITS_LEG);
    runLateral(200);
    EXPECT_FLOAT_EQ((float)autopilotWingConfig()->maxBank, bankDeg());

    limits.bankLimitDeg = 50.0f;            // never past the configured limit
    autopilotWingSetLimits(AP_WING_LIMITS_LEG, &limits);
    runLateral(200);
    EXPECT_FLOAT_EQ((float)autopilotWingConfig()->maxBank, bankDeg());
}

TEST(AutopilotWingTest, WingsLevelHoldsTheBankAtZero)
{
    engageLoiter(WING_LOITER_RIGHT);
    autopilotWingLimits_t limits;
    autopilotWingDefaultLimits(&limits);
    limits.wingsLevel = true;
    autopilotWingSetLimits(AP_WING_LIMITS_LEG, &limits);
    setPositionCm(0.0f, -20000.0f);
    runLateral(200);
    EXPECT_FLOAT_EQ(0.0f, bankDeg());
}

TEST(AutopilotWingTest, ALegsPitchRangeNarrowsTheClimbAndTheDive)
{
    resetForTest();
    autopilotWingLimits_t limits;
    autopilotWingDefaultLimits(&limits);
    limits.pitchMaxDeg = 5.0f;
    autopilotWingSetLimits(AP_WING_LIMITS_LEG, &limits);
    runVertical(testAltitudeCm + 5000.0f, 1000);
    EXPECT_NEAR(-5.0f, pitchTargetDeg(), 0.01f);

    resetForTest();
    limits.pitchMinDeg = 3.0f;              // a floor above trim holds the nose up even descending
    autopilotWingSetLimits(AP_WING_LIMITS_LEG, &limits);
    runVertical(testAltitudeCm - 5000.0f, 1000);
    EXPECT_NEAR(-3.0f, pitchTargetDeg(), 0.01f);
}

TEST(AutopilotWingTest, ALegsThrottleRangeMayGoBelowTheConfiguredMinimum)
{
    resetForTest();
    autopilotWingLimits_t limits;
    autopilotWingDefaultLimits(&limits);
    limits.throttleMin = 0.0f;
    limits.throttleMax = 0.0f;
    autopilotWingSetLimits(AP_WING_LIMITS_LEG, &limits);
    runVertical(testAltitudeCm, 300);
    EXPECT_FLOAT_EQ(0.0f, getAutopilotThrottle());
}

TEST(AutopilotWingTest, AThrottleFloorRaisedAboveTheMinimumIsMetAtOnce)
{
    resetForTest();
    autopilotWingLimits_t limits;
    autopilotWingDefaultLimits(&limits);
    limits.throttleMin = 0.0f;
    limits.throttleMax = 0.0f;
    autopilotWingSetLimits(AP_WING_LIMITS_LANDING, &limits);
    runVertical(testAltitudeCm, 300);
    ASSERT_FLOAT_EQ(0.0f, getAutopilotThrottle());

    // going around from the flare
    autopilotWingDefaultLimits(&limits);
    limits.throttleMin = limits.throttleMax;
    autopilotWingSetLimits(AP_WING_LIMITS_LANDING, &limits);
    runVertical(testAltitudeCm, 1);
    EXPECT_FLOAT_EQ(autopilotWingConfig()->maxThrottle * 0.01f, getAutopilotThrottle());
}

TEST(AutopilotWingTest, EachOwnersLimitsStandUntilThatOwnerClearsThem)
{
    engageLoiter(WING_LOITER_RIGHT);
    autopilotWingLimits_t leg;
    autopilotWingDefaultLimits(&leg);
    leg.bankLimitDeg = 15.0f;
    autopilotWingSetLimits(AP_WING_LIMITS_LEG, &leg);
    autopilotWingLimits_t descent;
    autopilotWingDefaultLimits(&descent);
    descent.throttleMin = 0.0f;
    descent.throttleMax = 0.0f;
    descent.motorStop = true;
    autopilotWingSetLimits(AP_WING_LIMITS_DESCENT, &descent);
    setPositionCm(0.0f, -20000.0f);
    runLateral(200);
    runVertical(testAltitudeCm, 300);
    EXPECT_FLOAT_EQ(15.0f, bankDeg());
    EXPECT_FLOAT_EQ(0.0f, getAutopilotThrottle());
    EXPECT_TRUE(autopilotWingMotorStopRequested());

    autopilotWingClearLimits(AP_WING_LIMITS_DESCENT);
    runLateral(200);
    EXPECT_FLOAT_EQ(15.0f, bankDeg());
    EXPECT_FALSE(autopilotWingMotorStopRequested());

    autopilotWingClearLimits(AP_WING_LIMITS_LEG);
    runLateral(200);
    EXPECT_FLOAT_EQ((float)autopilotWingConfig()->maxBank, bankDeg());
}

TEST(AutopilotWingTest, AClimbRateOverrideReplacesTheAltitudeError)
{
    resetForTest();
    autopilotWingLimits_t limits;
    autopilotWingDefaultLimits(&limits);
    limits.climbRateOverride = true;
    limits.climbRateCmS = -100.0f;
    autopilotWingSetLimits(AP_WING_LIMITS_LEG, &limits);
    runVertical(testAltitudeCm + 2000.0f, 100);     // 20 m below the target, yet it descends
    EXPECT_GT(pitchTargetDeg(), 1.0f);
}

TEST(AutopilotWingTest, MotorStopOnlyWhileALegAsksForIt)
{
    resetForTest();
    EXPECT_FALSE(autopilotWingMotorStopRequested());
    autopilotWingLimits_t limits;
    autopilotWingDefaultLimits(&limits);
    limits.motorStop = true;
    autopilotWingSetLimits(AP_WING_LIMITS_LEG, &limits);
    EXPECT_TRUE(autopilotWingMotorStopRequested());
    autopilotWingClearLimits(AP_WING_LIMITS_LEG);
    EXPECT_FALSE(autopilotWingMotorStopRequested());
}
