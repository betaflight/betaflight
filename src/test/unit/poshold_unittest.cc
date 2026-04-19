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

    // Test-controllable estimate
    static positionEstimate3d_t testEstimate;

    const positionEstimate3d_t *positionEstimatorGetEstimate(void) {
        return &testEstimate;
    }
    void positionEstimatorEnableXY(bool enable) { UNUSED(enable); }
    bool positionEstimatorIsValidXY(void) { return testEstimate.isValidXY; }

    // Nav stubs: position hold tests run without active navigation
    void positionNavInit(void) { }
    void positionNavReset(void) { }
    void positionNavUpdate(float /*dt*/, const positionEstimate3d_t * /*est*/) { }
    bool positionNavHasActiveTarget(void) { return false; }
    bool positionNavTargetReached(void) { return false; }
    vector3_t positionNavGetTargetVelocityCmS(void) { return (vector3_t){{0, 0, 0}}; }
    const positionNavCommand_t *positionNavGetActiveCommand(void) { return NULL; }

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
    float rcCommand[4];

    bool failsafeIsActive(void) { return false; }

    void parseRcChannels(const char *input, rxConfig_t *rxConfig) {
        UNUSED(input);
        UNUSED(rxConfig);
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
    cfg->positionP  = 50;
    cfg->positionI  = 50;
    cfg->positionII = 50;
    cfg->positionD  = 50;
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

    // After settling, sanityCheckDistance = fmaxf(1000, 1000*2) = 2000cm.
    // Exceed it to trigger the flyaway check.
    testEstimate.position.x = 3000.0f;

    EXPECT_FALSE(positionControl());
}

// -- Displacement response: heading North (yaw = 0) --

TEST_F(PosHoldTest, EastDisplacementProducesRollResponse)
{
    initAndSettleAt(0, 0, 0);

    testEstimate.position.x = 100.0f;
    runIterations(SETTLE_ITERATIONS);

    EXPECT_NE(autopilotAngle[AI_ROLL], 0.0f);
    EXPECT_NEAR(autopilotAngle[AI_PITCH], 0.0f, 0.5f);
}

TEST_F(PosHoldTest, NorthDisplacementProducesPitchResponse)
{
    initAndSettleAt(0, 0, 0);

    testEstimate.position.y = 100.0f;
    runIterations(SETTLE_ITERATIONS);

    EXPECT_NEAR(autopilotAngle[AI_ROLL], 0.0f, 0.5f);
    EXPECT_NE(autopilotAngle[AI_PITCH], 0.0f);
}

// -- Symmetry: opposite displacement gives opposite sign --

TEST_F(PosHoldTest, OppositeEastDisplacementsGiveOppositeRoll)
{
    initAndSettleAt(0, 0, 0);
    testEstimate.position.x = 100.0f;
    runIterations(SETTLE_ITERATIONS);
    const float rollPositive = autopilotAngle[AI_ROLL];

    initAndSettleAt(0, 0, 0);
    testEstimate.position.x = -100.0f;
    runIterations(SETTLE_ITERATIONS);
    const float rollNegative = autopilotAngle[AI_ROLL];

    EXPECT_NE(rollPositive, 0.0f);
    EXPECT_NE(rollNegative, 0.0f);
    EXPECT_LT(rollPositive * rollNegative, 0.0f);
    EXPECT_NEAR(fabsf(rollPositive), fabsf(rollNegative), 0.5f);
}

TEST_F(PosHoldTest, OppositeNorthDisplacementsGiveOppositePitch)
{
    initAndSettleAt(0, 0, 0);
    testEstimate.position.y = 100.0f;
    runIterations(SETTLE_ITERATIONS);
    const float pitchPositive = autopilotAngle[AI_PITCH];

    initAndSettleAt(0, 0, 0);
    testEstimate.position.y = -100.0f;
    runIterations(SETTLE_ITERATIONS);
    const float pitchNegative = autopilotAngle[AI_PITCH];

    EXPECT_NE(pitchPositive, 0.0f);
    EXPECT_NE(pitchNegative, 0.0f);
    EXPECT_LT(pitchPositive * pitchNegative, 0.0f);
    EXPECT_NEAR(fabsf(pitchPositive), fabsf(pitchNegative), 0.5f);
}

// -- Velocity damping (P term) --

TEST_F(PosHoldTest, EastwardVelocityProducesOpposingRoll)
{
    initAndSettleAt(0, 0, 0);

    testEstimate.velocity.x = 50.0f;
    runIterations(SETTLE_ITERATIONS);

    EXPECT_NE(autopilotAngle[AI_ROLL], 0.0f);
}

TEST_F(PosHoldTest, OppositeVelocitiesGiveOppositeAngles)
{
    initAndSettleAt(0, 0, 0);
    testEstimate.velocity.x = 50.0f;
    runIterations(SETTLE_ITERATIONS);
    const float rollEast = autopilotAngle[AI_ROLL];

    initAndSettleAt(0, 0, 0);
    testEstimate.velocity.x = -50.0f;
    runIterations(SETTLE_ITERATIONS);
    const float rollWest = autopilotAngle[AI_ROLL];

    EXPECT_LT(rollEast * rollWest, 0.0f);
}

// -- Heading rotation: verify body-frame transform --

TEST_F(PosHoldTest, HeadingEastSwapsAxes)
{
    initAndSettleAt(0, 0, 900);

    testEstimate.position.x = 100.0f;
    runIterations(SETTLE_ITERATIONS);

    EXPECT_NE(autopilotAngle[AI_PITCH], 0.0f);
    EXPECT_NEAR(autopilotAngle[AI_ROLL], 0.0f, 0.5f);
}

TEST_F(PosHoldTest, HeadingSouthReversesRollSign)
{
    initAndSettleAt(0, 0, 0);
    testEstimate.position.x = 100.0f;
    runIterations(SETTLE_ITERATIONS);
    const float rollNorth = autopilotAngle[AI_ROLL];

    initAndSettleAt(0, 0, 1800);
    testEstimate.position.x = 100.0f;
    runIterations(SETTLE_ITERATIONS);
    const float rollSouth = autopilotAngle[AI_ROLL];

    EXPECT_NE(rollNorth, 0.0f);
    EXPECT_NE(rollSouth, 0.0f);
    EXPECT_LT(rollNorth * rollSouth, 0.0f);
}

// -- Sticks active zeros angle output --

TEST_F(PosHoldTest, SticksActiveZerosOutput)
{
    initAndSettleAt(0, 0, 0);

    testEstimate.position.x = 100.0f;
    runIterations(SETTLE_ITERATIONS);
    EXPECT_NE(autopilotAngle[AI_ROLL], 0.0f);

    setSticksActiveStatus(true);
    runIterations(SETTLE_ITERATIONS);

    EXPECT_NEAR(autopilotAngle[AI_ROLL],  0.0f, 0.01f);
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
    EXPECT_NEAR(autopilotAngle[AI_ROLL], 0.0f, 0.2f);
}

TEST_F(PosHoldTest, ReleasingSticksBrakesThenSettles)
{
    initAndSettleAt(0, 0, 0);

    // Pilot is moving with sticks active: controller should output zero angles.
    setSticksActiveStatus(true);
    testEstimate.velocity.x = 120.0f;
    runIterations(5);
    EXPECT_NEAR(autopilotAngle[AI_ROLL], 0.0f, 0.01f);

    // Stick release at high speed should enter braking (not yet settled hold).
    setSticksActiveStatus(false);
    runIterations(5);
    const float brakingRoll = autopilotAngle[AI_ROLL];
    EXPECT_NE(brakingRoll, 0.0f);

    // Once velocity settles below threshold, hold point capture should settle output.
    testEstimate.velocity.x = 0.0f;
    runIterations(SETTLE_ITERATIONS);
    EXPECT_NEAR(autopilotAngle[AI_ROLL], 0.0f, 0.1f);
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
