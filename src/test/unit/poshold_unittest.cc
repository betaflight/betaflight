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
#include <string.h>
#include <math.h>

extern "C" {

    #include "platform.h"
    #include "build/debug.h"
    #include "pg/pg_ids.h"

    #include "common/axis.h"
    #include "common/filter.h"
    #include "common/maths.h"
    #include "common/vector.h"

    #include "fc/rc_controls.h"
    #include "fc/runtime_config.h"

    #include "flight/imu.h"
    #include "flight/pid.h"
    #include "flight/position.h"
    #include "flight/position_estimator.h"
    #include "flight/position_nav.h"

    #include "io/gps.h"

    #include "rx/rx.h"
    #include "scheduler/scheduler.h"
    #include "sensors/gyro.h"

    #include "pg/autopilot.h"
    #include "flight/autopilot.h"

    PG_REGISTER(autopilotConfig_t, autopilotConfig, PG_AUTOPILOT, 0);
    PG_REGISTER(positionConfig_t, positionConfig, PG_POSITION, 0);
    PG_REGISTER(gyroConfig_t, gyroConfig, PG_GYRO_CONFIG, 0);
    PG_REGISTER(rcControlsConfig_t, rcControlsConfig, PG_RC_CONTROLS_CONFIG, 0);

    // Test-controllable estimate
    static positionEstimate3d_t testEstimate;

    const positionEstimate3d_t *positionEstimatorGetEstimate(void) {
        return &testEstimate;
    }
    void positionEstimatorEnableXY(bool enable) { UNUSED(enable); }
    bool positionEstimatorIsValidXY(void) { return testEstimate.isValidXY; }

    // Nav stubs: default to no active navigation (plain position hold);
    // yaw-control tests drive them via the mockNav* variables.
    static bool mockNavHasActiveTarget = false;
    static positionNavCommand_t mockNavCommand;
    static vector3_t mockTargetVelCmS;

    void positionNavInit(void) { }
    void positionNavReset(void) { }
    void positionNavUpdate(float /*dt*/, const positionEstimate3d_t * /*est*/) { }
    bool positionNavHasActiveTarget(void) { return mockNavHasActiveTarget; }
    bool positionNavTargetReached(void) { return false; }
    vector3_t positionNavGetTargetVelocityCmS(void) { return mockTargetVelCmS; }
    const positionNavCommand_t *positionNavGetActiveCommand(void) { return mockNavHasActiveTarget ? &mockNavCommand : NULL; }

    float getAltitudeCm(void) { return 0.0f; }
    float getAltitudeDerivative(void) { return 0.0f; }
    float getAltitudeCmControl(void) { return 0.0f; }
    float getAltitudeDerivativeControl(void) { return 0.0f; }
    float getCosTiltAngle(void) { return 1.0f; }

    uint8_t armingFlags = 0;
    int16_t debug[DEBUG16_VALUE_COUNT];
    uint8_t debugMode;
    uint16_t flightModeFlags = 0;
    uint8_t stateFlags = 0;

    acc_t acc;
    attitudeEulerAngles_t attitude;
    gpsSolutionData_t gpsSol;
    gyro_t gyro;
    float rcCommand[4];

    bool failsafeIsActive(void) { return false; }

    void parseRcChannels(const char *input, rxConfig_t *rxConfig) {
        UNUSED(input);
        UNUSED(rxConfig);
    }


    float simulatedStickRoll = 0.0f;
    float simulatedStickPitch = 0.0f;
    float getSetpointRate(int axis)
    {
        if (axis == FD_ROLL) {
            return simulatedStickRoll;
        }
        if (axis == FD_PITCH) {
            return simulatedStickPitch;
        }
        return 0.0f;
    }

    timeDelta_t getTaskDeltaTimeUs(taskId_e taskId)
    {
        UNUSED(taskId);
        return TASK_PERIOD_HZ(100); // default poshold rate in tests
    }

    throttleStatus_e calculateThrottleStatus() {
        return THROTTLE_LOW;
    }
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

static const int SETTLE_ITERATIONS = 200;

static void runIterations(int n)
{
    for (int i = 0; i < n; i++) {
        positionControl();
    }
}

static void initAndSettleAt(float eastCm, float northCm, int16_t yawDecidegrees)
{
    memset(&testEstimate, 0, sizeof(testEstimate));
    testEstimate.isValidXY = true;
    testEstimate.trustXY = 1.0f;
    testEstimate.position.x = eastCm;
    testEstimate.position.y = northCm;

    attitude.values.yaw   = yawDecidegrees;
    attitude.values.pitch = 0;
    attitude.values.roll  = 0;

    autopilotConfig_t *cfg = autopilotConfigMutable();
    cfg->positionP  = 30;
    cfg->positionI  = 30;
    cfg->positionD  = 30;
    cfg->positionA  = 30;
    cfg->positionF  = 30;
    cfg->stopThreshold = 10;
    cfg->maxAngle   = 30;
    cfg->hoverThrottle = 1500;
    cfg->throttleMin   = 1000;
    cfg->throttleMax   = 2000;
    cfg->altitudeP = 50;
    cfg->altitudeI = 50;
    cfg->altitudeD = 50;
    cfg->altitudeF = 0;
    cfg->landingAltitudeM = 5;

    autopilotInit();
    resetPositionControl(100);

    runIterations(SETTLE_ITERATIONS);
}

class PosHoldTest : public ::testing::Test {
protected:
    void SetUp() override {
        memset(&attitude, 0, sizeof(attitude));
        memset(&testEstimate, 0, sizeof(testEstimate));
        memset(autopilotAngle, 0, sizeof(float) * RP_AXIS_COUNT);
        memset(&gyro, 0, sizeof(gyro));
        memset(&mockNavCommand, 0, sizeof(mockNavCommand));
        mockNavHasActiveTarget = false;
        mockTargetVelCmS = (vector3_t){{0.0f, 0.0f, 0.0f}};
        flightModeFlags = 0;
    }
};

// -- Basic sanity --

TEST_F(PosHoldTest, InvalidEstimateReturnsFalse)
{
    initAndSettleAt(0, 0, 0);
    testEstimate.isValidXY = false;
    EXPECT_FALSE(positionControl());
}

TEST_F(PosHoldTest, ValidEstimateReturnsTrue)
{
    initAndSettleAt(0, 0, 0);
    EXPECT_TRUE(positionControl());
}

TEST_F(PosHoldTest, StationaryAtTargetProducesNearZeroOutput)
{
    initAndSettleAt(0, 0, 0);
    runIterations(SETTLE_ITERATIONS);

    EXPECT_NEAR(autopilotAngle[AI_ROLL],  0.0f, 0.01f);
    EXPECT_NEAR(autopilotAngle[AI_PITCH], 0.0f, 0.01f);
}

// -- Flyaway detection --

TEST_F(PosHoldTest, FlyawayDetectionTriggersAtLargeDistance)
{
    initAndSettleAt(0, 0, 0);

    // exceed sanity check distance
    testEstimate.position.x = 3000.0f; 
    // expect positionControl to be false
    EXPECT_FALSE(positionControl()); 
}

// -- Displacement response: heading North (yaw = 0) --

TEST_F(PosHoldTest, EastDisplacementProducesNegativeRollResponse)
{
    initAndSettleAt(0, 0, 0);

    // Drifting East requires a leftward correction (Negative Roll)
    testEstimate.position.x = 100.0f;
    runIterations(SETTLE_ITERATIONS);

    EXPECT_LT(autopilotAngle[AI_ROLL], 0.0f); // Verifies output is  negative (Roll Left)
    EXPECT_NEAR(autopilotAngle[AI_PITCH], 0.0f, 0.5f);
}
TEST_F(PosHoldTest, WestDisplacementProducesPositiveRollResponse)
{
    initAndSettleAt(0, 0, 0);

    // Drifting West requires a rightward correction (Positive Roll)
    testEstimate.position.x = -100.0f;
    runIterations(SETTLE_ITERATIONS);

    EXPECT_GT(autopilotAngle[AI_ROLL], 0.0f); // Must be strictly POSITIVE (Roll Right)
    EXPECT_NEAR(autopilotAngle[AI_PITCH], 0.0f, 0.5f);
}

TEST_F(PosHoldTest, NortDisplacementProducesNegativePitchResponse)
{
    initAndSettleAt(0, 0, 0);

    // Drifting South requires a forward correction (Positive Pitch)
    testEstimate.position.y = 100.0f;
    runIterations(SETTLE_ITERATIONS);

    EXPECT_NEAR(autopilotAngle[AI_ROLL], 0.0f, 0.5f);
    EXPECT_LT(autopilotAngle[AI_PITCH], 0.0f); // Must be Negative (Pitch Back)
}

TEST_F(PosHoldTest, SouthDisplacementProducesPositivePitchResponse)
{
    initAndSettleAt(0, 0, 0);

    // Drifting South requires a forward correction (Positive Pitch)
    testEstimate.position.y = -100.0f;
    runIterations(SETTLE_ITERATIONS);

    EXPECT_NEAR(autopilotAngle[AI_ROLL], 0.0f, 0.5f);
    EXPECT_GT(autopilotAngle[AI_PITCH], 0.0f); // Must be positive (Pitch Forward)
}

// -- Velocity damping (P term) --

TEST_F(PosHoldTest, EastwardVelocityProducesOpposingRoll)
{
    initAndSettleAt(0, 0, 0);

    testEstimate.velocity.x = 50.0f;
    runIterations(SETTLE_ITERATIONS);

    EXPECT_NE(autopilotAngle[AI_ROLL], 0.0f);
}

TEST_F(PosHoldTest, EastVelocityProducesNegativeRollResponse)
{
    initAndSettleAt(0, 0, 0);

    // drifting East requires a leftward braking lean (Negative Roll)
    testEstimate.velocity.x = 50.0f;
    runIterations(SETTLE_ITERATIONS);

    EXPECT_LT(autopilotAngle[AI_ROLL], 0.0f); // Roll must be NEGATIVE (Roll Left)
    EXPECT_NEAR(autopilotAngle[AI_PITCH], 0.0f, 0.1f); // Pitch must be flat
}

TEST_F(PosHoldTest, WestVelocityProducesBrakingRollResponse)
{
    initAndSettleAt(0, 0, 0);

    // drifting West requires a rightward roll (Positive Roll)
    testEstimate.velocity.x = -50.0f;
    runIterations(SETTLE_ITERATIONS);

    EXPECT_GT(autopilotAngle[AI_ROLL], 0.0f); // Roll must be POSITIVE (Roll Right)
    EXPECT_NEAR(autopilotAngle[AI_PITCH], 0.0f, 0.1f); // Pitch must be flat
}


// -- Heading rotation: verify body-frame transform --
 TEST_F(PosHoldTest, DynamicHeadingRotationUnderDrift)
{
    // PHASE 1: Initialize with nose pointed due EAST (900 decidegrees)
    initAndSettleAt(0, 0, 900);

    // Drone drifts East
    testEstimate.position.x = 100.0f;
    runIterations(SETTLE_ITERATIONS);

    EXPECT_LT(autopilotAngle[AI_PITCH], 0.0f);        // Pitch must be  NEGATIVE (Pitch Back)
    EXPECT_NEAR(autopilotAngle[AI_ROLL], 0.0f, 0.1f);  // Roll must stay flat

    // PHASE 2: Pivot the nose to due NORTH (0 decidegrees) mid-flight
    // maintain the exact same 100.0f East displacement
    attitude.values.yaw = 0;
    runIterations(SETTLE_ITERATIONS);

    EXPECT_NEAR(autopilotAngle[AI_PITCH], 0.0f, 0.1f); // Pitch  must now be flat
    EXPECT_LT(autopilotAngle[AI_ROLL], 0.0f);         // Roll must now be NEGATIVE (Roll Left)
}

TEST_F(PosHoldTest, HeadingSouthReversesRollSign)
{
    // 1. Nose pointed North: Drifting East requires Roll Left (Negative)
    initAndSettleAt(0, 0, 0);
    testEstimate.position.x = 100.0f;
    runIterations(SETTLE_ITERATIONS);
    EXPECT_LT(autopilotAngle[AI_ROLL], 0.0f); 

    // 2. Nose pointed South: Drifting East requires Roll Right (Positive)
    initAndSettleAt(0, 0, 1800);
    testEstimate.position.x = 100.0f;
    runIterations(SETTLE_ITERATIONS);
    
    EXPECT_GT(autopilotAngle[AI_ROLL], 0.0f); // Roll must be  POSITIVE (Roll Right)
    EXPECT_NEAR(autopilotAngle[AI_PITCH], 0.0f, 0.1f); // Pitch  must be flat
}



// -- Sticks active reduces the response --

TEST_F(PosHoldTest, SticksActiveButCentered)
{
    initAndSettleAt(0, 0, 0);
    testEstimate.position.x = 100.0f; // Drone is offset to the right
    runIterations(SETTLE_ITERATIONS);

    // Ensure sticks are simulated as perfectly centered
    simulatedStickRoll = 0.0f;
    simulatedStickPitch = 0.0f;

    setSticksActiveStatus(true);
    runIterations(SETTLE_ITERATIONS);

    // Assert your new baseline calculation output
    EXPECT_NEAR(autopilotAngle[AI_ROLL], -0.9045f, 0.01f);
    EXPECT_NEAR(autopilotAngle[AI_PITCH], 0.0f, 0.01f);
}

TEST_F(PosHoldTest, EstimateValidityTransitionsUnavailableAvailableUnavailable)
{
    initAndSettleAt(0, 0, 0);

    testEstimate.isValidXY = false;
    EXPECT_FALSE(positionControl());

    testEstimate.isValidXY = true;
    testEstimate.position.x = 80.0f;
    runIterations(SETTLE_ITERATIONS);
    EXPECT_TRUE(positionControl());
    EXPECT_NE(autopilotAngle[AI_ROLL], 0.0f);

    testEstimate.isValidXY = false;
    EXPECT_FALSE(positionControl());
}

TEST_F(PosHoldTest, VelocityTransitionSimulatesFallbackAndRecovery)
{
    initAndSettleAt(0, 0, 0);

    testEstimate.velocity.x = 120.0f;
    runIterations(SETTLE_ITERATIONS);
    const float highVelocityRoll = autopilotAngle[AI_ROLL];
    EXPECT_NE(highVelocityRoll, 0.0f);

    testEstimate.velocity.x = 20.0f;
    runIterations(SETTLE_ITERATIONS);
    const float lowVelocityRoll = autopilotAngle[AI_ROLL];
    EXPECT_LT(fabsf(lowVelocityRoll), fabsf(highVelocityRoll));

    testEstimate.velocity.x = 0.0f;
    runIterations(SETTLE_ITERATIONS);
    EXPECT_NEAR(-0.7f, autopilotAngle[AI_ROLL], 0.1f);
}

// -- Feedforward (stick push) is a term of its own, apart from damping --

TEST_F(PosHoldTest, FeedforwardProducesStickProportionalLean)
{
    initAndSettleAt(0, 0, 0);

    // Craft stationary at target, so pidP and pidD are both ~0; the lean comes
    // from the F feedforward driven by the stick-commanded target velocity.
    setSticksActiveStatus(true);

    simulatedStickRoll = 100.0f;
    runIterations(SETTLE_ITERATIONS);
    const float leanSmall = autopilotAngle[AI_ROLL];

    simulatedStickRoll = 200.0f;
    runIterations(SETTLE_ITERATIONS);
    const float leanLarge = autopilotAngle[AI_ROLL];

    EXPECT_GT(fabsf(leanSmall), 0.5f);                 // a real push, not noise
    EXPECT_GT(fabsf(leanLarge), fabsf(leanSmall) + 1.0f); // twice the stick, more lean
}

// With no stick input the commanded target velocity, and therefore the F
// feedforward, must be zero. sticksMoveTarget() latches its last value as the
// stick eases back through the deadband; that residue must not stay driving F.
TEST_F(PosHoldTest, FeedforwardZeroWhenSticksInactive)
{
    initAndSettleAt(0, 0, 0);
    debugMode = DEBUG_AUTOPILOT_PID; // slot 6 carries pidF on the debug axis (East here) * 10

    // Deflect the roll stick, then centre it and release.
    setSticksActiveStatus(true);
    simulatedStickRoll = 150.0f;
    runIterations(50);
    simulatedStickRoll = 0.0f;
    setSticksActiveStatus(false);
    runIterations(SETTLE_ITERATIONS);

    EXPECT_EQ(debug[6], 0); // pidF, flat with no stick input
    debugMode = DEBUG_NONE;
}

// Guards the fix: on stick release the stale target velocity is zeroed, so the
// feedforward can't keep pushing in the direction of travel and fight braking.
TEST_F(PosHoldTest, ReleaseDropsFeedforwardSoBrakingOpposesMotion)
{
    initAndSettleAt(0, 0, 0);

    // Cruise East under a large stick deflection while actually moving East.
    setSticksActiveStatus(true);
    simulatedStickRoll = 300.0f;
    testEstimate.velocity.x = 150.0f;
    runIterations(SETTLE_ITERATIONS);

    // Release the stick but the craft is still moving East at the moment of release.
    setSticksActiveStatus(false);
    simulatedStickRoll = 0.0f;
    testEstimate.velocity.x = 150.0f;
    positionControl(); // first braking loop

    // Must lean West (negative roll) to brake. If the feedforward push survived
    // the release it would dominate and roll would be positive (still pushing East).
    EXPECT_LT(autopilotAngle[AI_ROLL], 0.0f);
}

TEST_F(PosHoldTest, ReleasingSticksBrakesThenHolds)
{
    initAndSettleAt(0, 0, 0);

    // Cruise East, stick centered, moving at 120 cm/s.
    setSticksActiveStatus(true);
    simulatedStickRoll = 0.0f;
    testEstimate.velocity.x = 120.0f;
    runIterations(SETTLE_ITERATIONS);
    EXPECT_LT(autopilotAngle[AI_ROLL], 0.0f);          // damping opposes the drift
    EXPECT_NEAR(autopilotAngle[AI_PITCH], 0.0f, 0.01f);

    // Release: decay velocity toward zero while advancing position, as a real brake would.
    setSticksActiveStatus(false);
    for (int i = 0; i < 600; i++) {
        testEstimate.velocity.x *= 0.96f;
        testEstimate.position.x += testEstimate.velocity.x * 0.01f;
        positionControl();
    }

    // Stop captured: output settles back to level once the craft has stopped.
    EXPECT_NEAR(autopilotAngle[AI_ROLL],  0.0f, 0.5f);
    EXPECT_NEAR(autopilotAngle[AI_PITCH], 0.0f, 0.1f);
}

// -- GPS-like scenario: large displacement, position + velocity --

TEST_F(PosHoldTest, GpsScenarioDriftAndReturn)
{
    initAndSettleAt(0, 0, 0);

    testEstimate.position.x = 300.0f;
    testEstimate.velocity.x = 50.0f;
    runIterations(SETTLE_ITERATIONS);

    const float rollDrifting = autopilotAngle[AI_ROLL];
    EXPECT_NE(rollDrifting, 0.0f);

    testEstimate.position.x = 50.0f;
    testEstimate.velocity.x = -30.0f;
    runIterations(SETTLE_ITERATIONS);

    const float rollReturning = autopilotAngle[AI_ROLL];

    EXPECT_LT(fabsf(rollReturning), fabsf(rollDrifting));
}

// -- Optical flow scenario: small displacement, velocity-dominated --

TEST_F(PosHoldTest, OpticalFlowScenarioVelocityDamping)
{
    initAndSettleAt(0, 0, 0);

    testEstimate.position.x = 20.0f;
    testEstimate.velocity.x = 30.0f;
    runIterations(SETTLE_ITERATIONS);

    EXPECT_NE(autopilotAngle[AI_ROLL], 0.0f);

    testEstimate.velocity.x = 0.0f;
    runIterations(SETTLE_ITERATIONS);

    const float rollStationary = autopilotAngle[AI_ROLL];
    EXPECT_NE(rollStationary, 0.0f);
}

// -- Angle limiting --

TEST_F(PosHoldTest, OutputIsLimitedToMaxAngle)
{
    initAndSettleAt(0, 0, 0);

    testEstimate.position.x = 800.0f;
    testEstimate.velocity.x = 200.0f;
    runIterations(SETTLE_ITERATIONS);

    const float mag = sqrtf(autopilotAngle[AI_ROLL] * autopilotAngle[AI_ROLL] +
                            autopilotAngle[AI_PITCH] * autopilotAngle[AI_PITCH]);
    EXPECT_LE(mag, 30.0f + 1.0f);
}

// -- Combined diagonal displacement --

TEST_F(PosHoldTest, DiagonalDisplacementProducesBothAxes)
{
    initAndSettleAt(0, 0, 0);

    testEstimate.position.x = 100.0f;
    testEstimate.position.y = 100.0f;
    runIterations(SETTLE_ITERATIONS);

    EXPECT_NE(autopilotAngle[AI_ROLL], 0.0f);
    EXPECT_NE(autopilotAngle[AI_PITCH], 0.0f);

    EXPECT_NEAR(fabsf(autopilotAngle[AI_ROLL]),
                fabsf(autopilotAngle[AI_PITCH]), 1.0f);
}

// -- Mission yaw control --

class AutopilotYawTest : public PosHoldTest {
protected:
    void engageNavLeg(uint8_t yawMode) {
        initAndSettleAt(0, 0, 0);

        autopilotConfig_t *cfg = autopilotConfigMutable();
        cfg->yawMode = yawMode;
        cfg->yawP = 50;                // 0.5 deg/s per deg of heading error
        cfg->yawD = 0;                 // deterministic P-only response
        cfg->maxYawRate = 30;
        cfg->minForwardVelocity = 100; // 1 m/s

        mockNavHasActiveTarget = true;
        mockNavCommand.active = true;
        mockNavCommand.acceptanceRadiusM = 5.0f;
        flightModeFlags |= AUTOPILOT_MODE;
    }

    // Enough iterations at 100 Hz for the 1 s engage attenuator to saturate.
    void settleYaw() { runIterations(SETTLE_ITERATIONS); }
};

TEST_F(AutopilotYawTest, InactiveWithoutAutopilotMode)
{
    engageNavLeg(YAW_MODE_VELOCITY);
    flightModeFlags = 0;
    testEstimate.velocity.x = 300.0f; // moving east, well above min speed

    settleYaw();
    EXPECT_FALSE(autopilotYawControlActive());
    EXPECT_FLOAT_EQ(autopilotGetYawRate(), 0.0f);
}

TEST_F(AutopilotYawTest, InactiveInFixedMode)
{
    engageNavLeg(YAW_MODE_FIXED);
    testEstimate.velocity.x = 300.0f;

    settleYaw();
    EXPECT_FALSE(autopilotYawControlActive());
}

TEST_F(AutopilotYawTest, InactiveWithoutNavTarget)
{
    engageNavLeg(YAW_MODE_VELOCITY);
    mockNavHasActiveTarget = false;
    testEstimate.velocity.x = 300.0f;

    settleYaw();
    EXPECT_FALSE(autopilotYawControlActive());
}

TEST_F(AutopilotYawTest, VelocityModeYawsTowardCourse)
{
    engageNavLeg(YAW_MODE_VELOCITY);
    // Heading north, flying east: a right turn is a negative (CCW-positive
    // convention) yaw rate, clamped at the max rate.
    testEstimate.velocity.x = 300.0f;

    settleYaw();
    EXPECT_TRUE(autopilotYawControlActive());
    EXPECT_NEAR(autopilotGetYawRate(), -30.0f, 0.1f);

    // Flying west instead: yaw the other way.
    testEstimate.velocity.x = -300.0f;
    settleYaw();
    EXPECT_NEAR(autopilotGetYawRate(), 30.0f, 0.1f);
}

TEST_F(AutopilotYawTest, VelocityModeInactiveBelowMinSpeed)
{
    engageNavLeg(YAW_MODE_VELOCITY);
    testEstimate.velocity.x = 50.0f; // below the 100 cm/s course gate

    settleYaw();
    EXPECT_FALSE(autopilotYawControlActive());
}

TEST_F(AutopilotYawTest, VelocityModeProportionalBelowClamp)
{
    engageNavLeg(YAW_MODE_VELOCITY);
    // Heading east already (900 decidegrees), flying east-north-east:
    // small negative error yaws gently left, unclamped.
    attitude.values.yaw = 900;
    testEstimate.velocity.x = 300.0f;
    testEstimate.velocity.y = 120.0f; // course ~68 deg: a ~22 deg left turn -> ~+11 deg/s

    settleYaw();
    EXPECT_TRUE(autopilotYawControlActive());
    EXPECT_NEAR(autopilotGetYawRate(), 11.0f, 1.0f);
}

TEST_F(AutopilotYawTest, BearingModeYawsTowardTarget)
{
    engageNavLeg(YAW_MODE_BEARING);
    mockNavCommand.targetPosEfM.v[0] = 50.0f; // 50 m east: a right turn from north

    settleYaw();
    EXPECT_TRUE(autopilotYawControlActive());
    EXPECT_NEAR(autopilotGetYawRate(), -30.0f, 0.1f);
}

TEST_F(AutopilotYawTest, BearingModeInactiveInsideAcceptanceRadius)
{
    engageNavLeg(YAW_MODE_BEARING);
    mockNavCommand.targetPosEfM.v[0] = 3.0f; // inside the 5 m radius

    settleYaw();
    EXPECT_FALSE(autopilotYawControlActive());
}

TEST_F(AutopilotYawTest, HybridFallsBackToBearingWhenSlow)
{
    engageNavLeg(YAW_MODE_HYBRID);
    mockNavCommand.targetPosEfM.v[0] = 50.0f;
    testEstimate.velocity.x = 50.0f; // too slow for a course heading

    settleYaw();
    EXPECT_TRUE(autopilotYawControlActive());
    EXPECT_NEAR(autopilotGetYawRate(), -30.0f, 0.1f);
}

TEST_F(AutopilotYawTest, MissionYawRateCapApplies)
{
    engageNavLeg(YAW_MODE_VELOCITY);
    testEstimate.velocity.x = 300.0f;
    autopilotSetYawRateLimit(10.0f);

    settleYaw();
    EXPECT_NEAR(autopilotGetYawRate(), -10.0f, 0.1f);

    autopilotSetYawRateLimit(0.0f); // no cap: back to ap_max_yaw_rate
    settleYaw();
    EXPECT_NEAR(autopilotGetYawRate(), -30.0f, 0.1f);
}

TEST_F(AutopilotYawTest, EngageRampsRateIn)
{
    engageNavLeg(YAW_MODE_VELOCITY);
    testEstimate.velocity.x = 300.0f;

    // A quarter of the 1 s ramp: attenuated well below the clamp.
    runIterations(25);
    EXPECT_TRUE(autopilotYawControlActive());
    EXPECT_GT(autopilotGetYawRate(), -15.0f);
    EXPECT_LT(autopilotGetYawRate(), 0.0f);
}

// -- Nav velocity mode --

static uint16_t dragCoeffForTarget(float kDragPerS, float targetCmS)
{
    const float ffDeg = RADIANS_TO_DEGREES(atanf(kDragPerS * targetCmS / 981.0f));
    return (uint16_t)lrintf(ffDeg / (targetCmS * 0.0001f));
}

static void stepVelocityPlant(float kDragPerS, float *vNorthCmS)
{
    positionControl();
    const float pitchRad = DEGREES_TO_RADIANS(autopilotAngle[AI_PITCH]);
    *vNorthCmS += (981.0f * tanf(pitchRad) - kDragPerS * (*vNorthCmS)) * 0.01f;
    testEstimate.velocity.y = *vNorthCmS;
}

class VelocityModeTest : public PosHoldTest {
protected:
    void engageVelocityNav(uint8_t velocityP, uint8_t velocityI, uint8_t velocityD,
                            uint16_t velocityDragCoeff, uint8_t velocityBuildupMaxPitch,
                            uint8_t maxAngle = 45, bool enable = true)
    {
        initAndSettleAt(0, 0, 0);

        autopilotConfig_t *cfg = autopilotConfigMutable();
        cfg->maxAngle = maxAngle;
        cfg->positionCutoff = 30;
        cfg->velocityControlEnable = enable ? 1 : 0;
        cfg->velocityP = velocityP;
        cfg->velocityI = velocityI;
        cfg->velocityD = velocityD;
        cfg->velocityDragCoeff = velocityDragCoeff;
        cfg->velocityBuildupMaxPitch = velocityBuildupMaxPitch;
        autopilotInit();

        mockNavHasActiveTarget = true;
        mockNavCommand.active = true;
    }

    void setTargetVelocityNorth(float cmS)
    {
        mockTargetVelCmS = (vector3_t){{0.0f, cmS, 0.0f}};
    }
};

TEST_F(VelocityModeTest, CruiseConvergence)
{
    const float kDrag = 0.8f;
    const float targetCmS = 500.0f;
    engageVelocityNav(50, 20, 0, dragCoeffForTarget(kDrag, targetCmS), 30);
    setTargetVelocityNorth(targetCmS);

    float vNorth = 0.0f;
    for (int i = 0; i < 1500; i++) {
        stepVelocityPlant(kDrag, &vNorth);
    }

    EXPECT_NEAR(vNorth, targetCmS, targetCmS * 0.05f);
}

TEST_F(VelocityModeTest, NoOvershoot)
{
    const float kDrag = 0.8f;
    const float targetCmS = 500.0f;
    engageVelocityNav(50, 20, 0, dragCoeffForTarget(kDrag, targetCmS), 30);
    setTargetVelocityNorth(targetCmS);

    float vNorth = 0.0f;
    float maxV = 0.0f;
    for (int i = 0; i < 1500; i++) {
        stepVelocityPlant(kDrag, &vNorth);
        maxV = fmaxf(maxV, vNorth);
    }

    EXPECT_LE(maxV, targetCmS * 1.10f);
}

TEST_F(VelocityModeTest, DragFeedforwardMagnitude)
{
    const float targetCmS = 400.0f;
    const uint16_t dragCoeff = 300;
    engageVelocityNav(30, 10, 20, dragCoeff, 30);
    setTargetVelocityNorth(targetCmS);
    testEstimate.velocity.y = targetCmS;

    runIterations(300);

    const float expectedFFDeg = dragCoeff * 0.0001f * targetCmS;
    EXPECT_NEAR(autopilotAngle[AI_PITCH], expectedFFDeg, 1.0f);
}

TEST_F(VelocityModeTest, BuildupClampBoundsP)
{
    const uint8_t buildupMaxPitch = 10;
    engageVelocityNav(50, 0, 0, 0, buildupMaxPitch, 45);
    setTargetVelocityNorth(1500.0f);

    for (int i = 0; i < 30; i++) {
        positionControl();
        EXPECT_LE(fabsf(autopilotAngle[AI_PITCH]), buildupMaxPitch + 0.05f);
    }
}

TEST_F(VelocityModeTest, IntegralSeparationAndFreeze)
{
    engageVelocityNav(30, 50, 0, 0, 30, 45);
    setTargetVelocityNorth(2000.0f); // filtered velocity stays near 0: error stays well above the 250 cm/s relax gate

    positionControl();
    const float pitchFirst = autopilotAngle[AI_PITCH];
    runIterations(100);
    EXPECT_NEAR(autopilotAngle[AI_PITCH], pitchFirst, 0.05f);

    testEstimate.velocity.y = 2000.0f - 100.0f; // step to a small, sustained error
    runIterations(40); // let the velocity filter settle into the relax gate
    const float pitchSmallErrEarly = autopilotAngle[AI_PITCH];
    runIterations(100);
    const float pitchSmallErrLater = autopilotAngle[AI_PITCH];
    EXPECT_GT(fabsf(pitchSmallErrLater), fabsf(pitchSmallErrEarly) + 0.5f);
}

TEST_F(VelocityModeTest, VelocityModeDisabledUsesLegacyPath)
{
    engageVelocityNav(30, 50, 0, 0, 30, 45, false);
    setTargetVelocityNorth(300.0f);

    runIterations(20);
    const float pitchEarly = autopilotAngle[AI_PITCH];

    runIterations(100);
    const float pitchLater = autopilotAngle[AI_PITCH];

    EXPECT_GT(fabsf(pitchLater), fabsf(pitchEarly) + 1.0f);
}

TEST_F(VelocityModeTest, ResetOnNavReentry)
{
    engageVelocityNav(30, 50, 0, 0, 30, 45);
    setTargetVelocityNorth(150.0f); // stays inside the relax gate throughout: the integral builds every cycle

    runIterations(150);
    const float pitchBeforeReentry = autopilotAngle[AI_PITCH];

    mockNavHasActiveTarget = false;
    positionControl(); // one cycle of pos-hold fallback

    mockNavHasActiveTarget = true;
    positionControl(); // nav re-entry: initNavMode() resets the velocity integral

    EXPECT_LT(fabsf(autopilotAngle[AI_PITCH]), fabsf(pitchBeforeReentry) * 0.5f);
}

TEST_F(VelocityModeTest, PosHoldUnaffectedByDefaultOn)
{
    const int cycles = 50;
    float baselineRoll[cycles];
    float baselinePitch[cycles];

    initAndSettleAt(0, 0, 0);
    autopilotConfigMutable()->velocityControlEnable = 0;
    autopilotInit();
    testEstimate.position.x = 100.0f;
    for (int i = 0; i < cycles; i++) {
        positionControl();
        baselineRoll[i] = autopilotAngle[AI_ROLL];
        baselinePitch[i] = autopilotAngle[AI_PITCH];
    }

    initAndSettleAt(0, 0, 0);
    autopilotConfigMutable()->velocityControlEnable = 1;
    autopilotInit();
    testEstimate.position.x = 100.0f;
    for (int i = 0; i < cycles; i++) {
        positionControl();
        EXPECT_FLOAT_EQ(autopilotAngle[AI_ROLL], baselineRoll[i]);
        EXPECT_FLOAT_EQ(autopilotAngle[AI_PITCH], baselinePitch[i]);
    }
}
