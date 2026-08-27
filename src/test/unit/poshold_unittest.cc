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
    float getAltitudeAccelerationControl(void) { return 0.0f; }
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

    float simulatedMaxRcRate = 720.0f;
    float getMaxRcRate(int axis)
    {
        UNUSED(axis);
        return simulatedMaxRcRate; // full-stick maps to this rate; autopilotInit scales maxVelocity by it
    }

    // TASK_POSHOLD is rescheduled to the flow sensor's data rate, so the loop
    // interval is not always POSHOLD_TASK_RATE_HZ. Tests set this to check that
    // the rate-dependent parts of the loop hold up.
    int simulatedTaskRateHz = 100;
    timeDelta_t getTaskDeltaTimeUs(taskId_e taskId)
    {
        UNUSED(taskId);
        return 1000000 / simulatedTaskRateHz;
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
    cfg->maxVelocity = 500;   // 5 m/s full-stick target; drives the stick-velocity gain
    cfg->stopThreshold = 5;
    cfg->maxAngle   = 30;
    cfg->hoverThrottle = 1500;
    cfg->throttleMin   = 1000;
    cfg->throttleMax   = 2000;
    cfg->altitudeP = 50;
    cfg->altitudeI = 50;
    cfg->altitudeD = 50;
    cfg->altitudeA = 50;
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
        simulatedTaskRateHz = 100;
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

TEST_F(PosHoldTest, FlyawayDetectionRetriesOnceThenTriggers)
{
    initAndSettleAt(0, 0, 0);

    // Progressive runaway at a constant 15 m/s (bad-mag style): the estimate
    // keeps moving and never meets the braking stop/stall conditions, so the
    // fence has to catch it in BOTH the settled phase (first sustained trip,
    // which spends the one-shot retry) and the post-retry braking phase
    // (second trip, which must fail). No artificial settling in between.
    bool failed = false;
    int failedAt = -1;
    for (int i = 0; i < 800 && !failed; i++) {
        testEstimate.position.x += 15.0f;      // 15 cm per 10 ms tick
        testEstimate.velocity.x = 1500.0f;
        failed = !positionControl();
        failedAt = i;
    }
    EXPECT_TRUE(failed);
    // The failure must land AFTER the retry re-anchored (first sustained trip
    // is ~2.3 s in): a single trip alone no longer fails the hold.
    EXPECT_GT(failedAt, 300);
}

TEST_F(PosHoldTest, SingleOutlierFixIsAbsorbedHoldingLastOutput)
{
    initAndSettleAt(0, 0, 0);
    const float rollBefore = autopilotAngle[AI_ROLL];
    const float pitchBefore = autopilotAngle[AI_PITCH];

    // A single 30 m multipath spike, well beyond the 20 m fence.
    testEstimate.position.x = 3000.0f;
    EXPECT_TRUE(positionControl());
    // The spike is not fed to the PIDs: the previous output is held, no
    // lunge toward the angle clamp and no POSHOLD FAIL.
    EXPECT_NEAR(autopilotAngle[AI_ROLL], rollBefore, 0.01f);
    EXPECT_NEAR(autopilotAngle[AI_PITCH], pitchBefore, 0.01f);

    // The next fixes are normal again: control simply carries on.
    testEstimate.position.x = 0.0f;
    for (int i = 0; i < 100; i++) {
        EXPECT_TRUE(positionControl());
    }
}

TEST_F(PosHoldTest, ResumeAfterFrozenExcursionDoesNotSpikeTheOutput)
{
    initAndSettleAt(0, 0, 0);

    // real motion first, so the A-term history holds a meaningful velocity
    testEstimate.velocity.x = 300.0f;
    for (int i = 0; i < 50; i++) {
        EXPECT_TRUE(positionControl());
    }

    // a sub-latch excursion freezes the output; meanwhile the craft stops,
    // so the frozen A-term history goes badly stale
    testEstimate.position.x = 3000.0f;
    testEstimate.velocity.x = 0.0f;
    for (int i = 0; i < 80; i++) {   // 0.8 s, inside the 1 s latch window
        EXPECT_TRUE(positionControl());
    }
    const float rollFrozen = autopilotAngle[AI_ROLL];

    // Resume. previousVelocity was not refreshed during the frozen window, so
    // without the A-term one-shot re-baseline the loop would differentiate the
    // 300 cm/s of stale velocity into an enormous acceleration term and slam the
    // angle to maxAngle. The one-shot forces A = 0 for the first resumed loop, so
    // the output reflects the now-stopped craft rather than a spike.
    testEstimate.position.x = 0.0f;
    EXPECT_TRUE(positionControl());
    EXPECT_LT(fabsf(autopilotAngle[AI_ROLL]), fabsf(rollFrozen)); // no upward spike past the held value
    EXPECT_LT(fabsf(autopilotAngle[AI_ROLL]), 5.0f);             // settles toward the stopped state
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

    // Centred sticks command zero target velocity, and the anchor-off virtual
    // distance error is reset on stick engagement, so P, D, A and F are all 0.
    // Sticks-active uses the I_FREEZE policy, which retains (does not accumulate)
    // the distance integral built up while holding the 1 m offset before the
    // sticks engaged; that frozen integral is the whole residual lean:
    //   Ki * integral = (30 * 0.00017) * (-100 cm * 200 * 10 ms) = -1.02 deg
    EXPECT_NEAR(autopilotAngle[AI_ROLL], -1.02f, 0.01f);
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
    // With the craft stopped the damping subsides further, toward the settled hold.
    EXPECT_LT(fabsf(autopilotAngle[AI_ROLL]), fabsf(lowVelocityRoll));
    EXPECT_NEAR(autopilotAngle[AI_ROLL], 0.0f, 0.5f);
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

// -- Braking entry threshold (ap_stop_threshold, default 5 cm/s) --
// Slot 6 of DEBUG_AUTOPILOT_STOP carries the hold status with +1 added while
// braking, so a settled hold reads BRAKING_STATUS_HELD and a braking entry
// reads BRAKING_STATUS_BRAKING.
static const int BRAKING_STATUS_HELD = 3;
static const int BRAKING_STATUS_BRAKING = 4;

TEST_F(PosHoldTest, EntrySpeedAboveStopThresholdStartsBraking)
{
    initAndSettleAt(0, 0, 0);
    debugMode = DEBUG_AUTOPILOT_STOP;

    // Release the sticks while carrying 40 cm/s East, well above the 5 cm/s stop
    // threshold: the hold must start in braking mode to arrest it.
    setSticksActiveStatus(true);
    testEstimate.velocity.x = 40.0f;
    runIterations(50);
    setSticksActiveStatus(false);
    positionControl(); // capture the point and decide whether to brake

    EXPECT_EQ(debug[6], BRAKING_STATUS_BRAKING);
    debugMode = DEBUG_NONE;
}

TEST_F(PosHoldTest, EntrySpeedBelowStopThresholdHoldsImmediately)
{
    initAndSettleAt(0, 0, 0);
    debugMode = DEBUG_AUTOPILOT_STOP;

    // Same release, but at 3 cm/s: below the 5 cm/s threshold there is no entry
    // speed worth arresting, so the hold locks the captured point straight away
    // with full P authority rather than dragging the target to the craft.
    setSticksActiveStatus(true);
    testEstimate.velocity.x = 3.0f;
    runIterations(50);
    setSticksActiveStatus(false);
    positionControl();

    EXPECT_EQ(debug[6], BRAKING_STATUS_HELD);
    debugMode = DEBUG_NONE;
}

TEST_F(PosHoldTest, StopThresholdSettingSetsTheBrakingEntrySpeed)
{
    initAndSettleAt(0, 0, 0);
    debugMode = DEBUG_AUTOPILOT_STOP;
    autopilotConfigMutable()->stopThreshold = 100; // raise it above the entry speed

    setSticksActiveStatus(true);
    testEstimate.velocity.x = 40.0f;
    runIterations(50);
    setSticksActiveStatus(false);
    positionControl();

    EXPECT_EQ(debug[6], BRAKING_STATUS_HELD); // 40 cm/s no longer brakes
    debugMode = DEBUG_NONE;
}

// TASK_POSHOLD runs at the flow sensor's rate, which is not POSHOLD_TASK_RATE_HZ,
// so the braking phase has to give up after a fixed wall-clock time at whatever
// rate the loop is running. Keep this in step with BRAKING_TIMEOUT_S in
// autopilot_multirotor.c; the point of the tests is rate-independence, not the
// particular value.
static const float EXPECTED_BRAKING_TIMEOUT_S = 1.2f;

static int loopsUntilBrakingEnds(int taskRateHz)
{
    simulatedTaskRateHz = taskRateHz;
    initAndSettleAt(0, 0, 0);
    debugMode = DEBUG_AUTOPILOT_STOP;

    // Release the sticks while moving East fast enough to brake, and hold that
    // speed so braking can only end on the timeout.
    setSticksActiveStatus(true);
    testEstimate.velocity.x = 40.0f;
    runIterations(50);
    setSticksActiveStatus(false);

    int loops = 0;
    for (int i = 0; i < 10 * taskRateHz; i++) {
        testEstimate.velocity.x = 40.0f;
        positionControl();
        loops++;
        if (debug[6] == BRAKING_STATUS_HELD) {
            break;
        }
    }
    debugMode = DEBUG_NONE;
    return loops;
}

TEST_F(PosHoldTest, BrakingTimeoutIsWallClockAtTheNominalTaskRate)
{
    const int loops = loopsUntilBrakingEnds(100);
    EXPECT_NEAR(loops / 100.0f, EXPECTED_BRAKING_TIMEOUT_S, 0.03f);
}

TEST_F(PosHoldTest, BrakingTimeoutIsWallClockAtHalfTheNominalTaskRate)
{
    const int loops = loopsUntilBrakingEnds(50);
    EXPECT_NEAR(loops / 50.0f, EXPECTED_BRAKING_TIMEOUT_S, 0.06f);
}

// The target-velocity feedforward is a rate of change, so the same commanded
// acceleration must produce the same F contribution whatever the loop interval.
// F also carries Kd * targetVelocity, which is rate-independent by construction;
// running to the same final target velocity with and without acceleration
// isolates the acceleration part.
static float navFeedforwardAtRate(int taskRateHz, float accelCmSS)
{
    simulatedTaskRateHz = taskRateHz;
    initAndSettleAt(0, 0, 0);
    debugMode = DEBUG_AUTOPILOT_PID;   // slot 6 is F * 10 on the East axis, before smoothing

    // Nav with an active command anchors to a target position, which keeps the
    // velocity-buildup clamp (an anchor-off-only feature) out of the way.
    mockNavHasActiveTarget = true;
    mockNavCommand.active = true;
    mockNavCommand.targetPosEfM = (vector3_t){{ 0.0f, 0.0f, 0.0f }};

    const float finalVel = 200.0f;
    const float dt = 1.0f / taskRateHz;
    for (int i = 0; i < taskRateHz; i++) {
        // Ramp at accelCmSS, then sit at finalVel: both rates reach the same
        // target velocity, so only the acceleration term can differ.
        const float ramping = accelCmSS * dt * (i + 1);
        mockTargetVelCmS = (vector3_t){{ (accelCmSS > 0.0f) ? ramping : finalVel, 0.0f, 0.0f }};
        positionControl();
    }
    const float f = debug[6] / 10.0f;
    debugMode = DEBUG_NONE;
    return f;
}

TEST_F(PosHoldTest, FeedforwardIsIndependentOfTaskRate)
{
    // At each rate: F while accelerating to 200 cm/s, minus F holding 200 cm/s.
    const float accel100 = navFeedforwardAtRate(100, 200.0f) - navFeedforwardAtRate(100, 0.0f);
    const float accel50 = navFeedforwardAtRate(50, 200.0f) - navFeedforwardAtRate(50, 0.0f);

    // 200 cm/s^2 * positionF 30 * XY_F_SCALE 0.0001 = 0.6 deg, at any rate.
    EXPECT_NEAR(accel100, 0.6f, 0.1f);
    EXPECT_NEAR(accel50, accel100, 0.1f);
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

// -- Nav mode --
// The unified controller flies nav by anchoring position to positionNav's
// carrot (targetPosEfM) with the commanded velocity as feedforward; a bounded
// nav position error stops the speed-proportional carrot lead running away.

class NavModeTest : public PosHoldTest {
protected:
    void engageNav(uint8_t positionD, uint8_t positionP, uint8_t positionA,
                            uint8_t velocityDragCoeff, uint8_t velocityBuildupMaxPitch,
                            uint8_t maxAngle = 45)
    {
        initAndSettleAt(0, 0, 0);

        autopilotConfig_t *cfg = autopilotConfigMutable();
        cfg->maxAngle = maxAngle;
        cfg->positionCutoff = 30;
        cfg->positionD = positionD;
        cfg->positionP = positionP;
        cfg->positionA = positionA;
        cfg->velocityDragCoeff = velocityDragCoeff;
        cfg->velocityBuildupMaxPitch = velocityBuildupMaxPitch;
        autopilotInit();

        mockNavHasActiveTarget = true;
        mockNavCommand.active = true;
    }

    // Place the nav carrot (metres, ENU) that the controller position-anchors to.
    void setNavCarrot(float eastM, float northM)
    {
        mockNavCommand.targetPosEfM.v[0] = eastM;  // ENU_E
        mockNavCommand.targetPosEfM.v[1] = northM; // ENU_N
    }

    void setTargetVelocityNorth(float cmS)
    {
        mockTargetVelCmS = (vector3_t){{0.0f, cmS, 0.0f}};
    }
};

TEST_F(NavModeTest, NavAnchorsToCarrotAhead)
{
    // Carrot 50 m north, craft at the origin: the position anchor produces a
    // lean toward the carrot (pitch), with negligible roll.
    engageNav(30, 30, 0, 0, 30, 45);
    setNavCarrot(0.0f, 50.0f);
    setTargetVelocityNorth(0.0f);

    runIterations(SETTLE_ITERATIONS);

    EXPECT_GT(fabsf(autopilotAngle[AI_PITCH]), 5.0f);
    EXPECT_LT(fabsf(autopilotAngle[AI_ROLL]), 2.0f);
}

TEST_F(NavModeTest, NavPositionErrorIsBounded)
{
    // The carrot lead grows with speed; NAV_ERROR_DISTANCE_LIMIT bounds the
    // position error so a distant carrot cannot drive P without limit. Two
    // carrots well beyond the bound must produce the same (clamped) lean.
    engageNav(30, 30, 0, 0, 30, 45);
    setTargetVelocityNorth(0.0f);

    setNavCarrot(0.0f, 50.0f);
    runIterations(SETTLE_ITERATIONS);
    const float pitchNear = autopilotAngle[AI_PITCH];

    setNavCarrot(0.0f, 500.0f);
    runIterations(SETTLE_ITERATIONS);
    const float pitchFar = autopilotAngle[AI_PITCH];

    EXPECT_NEAR(pitchFar, pitchNear, 0.5f);
}

TEST_F(NavModeTest, NavVelocityFeedforward)
{
    // Carrot coincident with the craft (zero position error), so the lean comes
    // purely from the target-velocity feedforward (F = targetVel * Kd).
    engageNav(30, 30, 0, 0, 30, 45);
    setNavCarrot(0.0f, 0.0f);
    setTargetVelocityNorth(300.0f);
    testEstimate.velocity.y = 0.0f;

    runIterations(SETTLE_ITERATIONS);

    EXPECT_GT(fabsf(autopilotAngle[AI_PITCH]), 5.0f);
    EXPECT_LT(fabsf(autopilotAngle[AI_ROLL]), 2.0f);
}

TEST_F(NavModeTest, NavForcesIntegralZero)
{
    // Nav runs the I_ZERO policy: with a fixed carrot error the output is carried
    // by P alone and must not creep upward over time from an accumulating
    // integral (which is what a settled position hold would do).
    engageNav(30, 30, 0, 0, 30, 45);
    setNavCarrot(0.0f, 3.0f); // 3 m north, inside the nav error bound
    setTargetVelocityNorth(0.0f);

    runIterations(30);
    const float pitchEarly = autopilotAngle[AI_PITCH];
    runIterations(200);
    const float pitchLater = autopilotAngle[AI_PITCH];

    EXPECT_NEAR(pitchLater, pitchEarly, 0.5f);
}

TEST_F(NavModeTest, ResetOnNavReentry)
{
    engageNav(30, 50, 0, 0, 30, 45);
    setTargetVelocityNorth(150.0f); // stays inside the relax gate throughout: the integral builds every cycle

    runIterations(150);
    const float pitchBeforeReentry = autopilotAngle[AI_PITCH];

    mockNavHasActiveTarget = false;
    positionControl(); // one cycle of pos-hold fallback

    mockNavHasActiveTarget = true;
    positionControl(); // nav re-entry: initNavMode() resets the velocity integral

    EXPECT_LT(fabsf(autopilotAngle[AI_PITCH]), fabsf(pitchBeforeReentry) * 0.5f);
}

TEST_F(NavModeTest, PositionControlResetIsDeterministic)
{
    const int cycles = 50;
    float baselineRoll[cycles];
    float baselinePitch[cycles];

    initAndSettleAt(0, 0, 0);
    autopilotInit();
    testEstimate.position.x = 100.0f;
    for (int i = 0; i < cycles; i++) {
        positionControl();
        baselineRoll[i] = autopilotAngle[AI_ROLL];
        baselinePitch[i] = autopilotAngle[AI_PITCH];
    }

    initAndSettleAt(0, 0, 0);
    autopilotInit();
    testEstimate.position.x = 100.0f;
    for (int i = 0; i < cycles; i++) {
        positionControl();
        EXPECT_FLOAT_EQ(autopilotAngle[AI_ROLL], baselineRoll[i]);
        EXPECT_FLOAT_EQ(autopilotAngle[AI_PITCH], baselinePitch[i]);
    }
}
