/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

// Characterization tests for the ported danusha2345/ADRC-betaflight fixes: the liftoff gate
// (fix #8/#10b), the throttle-scaled b0 (fix #10a) and the z3 leaky decay (fix #11). adrc.c has no
// pid.c/pidRuntime coupling, so these exercise its public API directly against a stubbed throttle
// and gyro reading, instead of needing a full pidController() mock harness.

#include <stdint.h>
#include <stdbool.h>

#include <math.h>

static float simulatedThrottle = 0.0f;

extern "C" {
    #include "common/axis.h"
    #include "common/maths.h"
    #include "common/filter.h"

    #include "build/debug.h"

    #include "sensors/gyro.h"

    #include "flight/adrc.h"

    gyro_t gyro;

    float mixerGetThrottle(void) { return simulatedThrottle; }
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

int16_t debug[DEBUG16_VALUE_COUNT];
uint8_t debugMode;

namespace {

constexpr float TEST_DT = 0.008f; // 125 Hz-equivalent test looptime

void resetGyro()
{
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        gyro.gyroADCf[axis] = 0.0f;
    }
}

class AdrcUnittest : public ::testing::Test {
protected:
    adrcProfile_t profile;
    adrcRuntime_t runtime;

    void SetUp() override
    {
        simulatedThrottle = 0.0f;
        resetGyro();
        adrcResetProfile(&profile);
        adrcInitConfig(&profile, &runtime, TEST_DT);
        for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
            adrcResetState(&runtime, axis);
        }
        adrcResetGate(&runtime);
    }
};

} // namespace

TEST_F(AdrcUnittest, GateStaysClosedAtIdle)
{
    simulatedThrottle = 0.0f;
    resetGyro();
    for (int i = 0; i < 200; i++) {
        adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    }
    EXPECT_FALSE(runtime.liftoff);
}

TEST_F(AdrcUnittest, GateOpensImmediatelyOnThrottle)
{
    simulatedThrottle = 0.5f; // above ADRC_LIFTOFF_THROTTLE (0.4)
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    EXPECT_TRUE(runtime.liftoff);
}

TEST_F(AdrcUnittest, GateOpensOnSustainedRotationWithoutThrottle)
{
    simulatedThrottle = 0.0f;
    gyro.gyroADCf[FD_ROLL] = 25.0f; // above ADRC_LIFTOFF_GYRO_DPS (20)

    // A single loop is shorter than the ADRC_LIFTOFF_HOLD_S (0.025s) sustain requirement.
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    EXPECT_FALSE(runtime.liftoff);

    // After enough loops the sustained-rotation hold is satisfied (toss-launch path).
    for (int i = 0; i < 10; i++) {
        adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    }
    EXPECT_TRUE(runtime.liftoff);
}

TEST_F(AdrcUnittest, GateReArmsAfterSustainedIdleStillness)
{
    // Simulate already airborne.
    simulatedThrottle = 0.6f;
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    ASSERT_TRUE(runtime.liftoff);

    // Idle throttle and stillness, sustained past ADRC_LIFTOFF_IDLE_HOLD_S (0.5s @ 8ms = 63 loops).
    simulatedThrottle = 0.0f;
    resetGyro();
    for (int i = 0; i < 70; i++) {
        adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    }
    EXPECT_FALSE(runtime.liftoff);
}

TEST_F(AdrcUnittest, GateDoesNotReArmOnBriefAirborneThrottleChop)
{
    simulatedThrottle = 0.6f;
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    ASSERT_TRUE(runtime.liftoff);

    // A brief chop shorter than the idle-hold requirement (well under 63 loops).
    simulatedThrottle = 0.0f;
    resetGyro();
    for (int i = 0; i < 20; i++) {
        adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    }
    EXPECT_TRUE(runtime.liftoff);

    // Throttle returns - the gate must still be open, and the idle timer must have been reset
    // rather than continuing to accumulate toward re-arm across the interruption.
    simulatedThrottle = 0.6f;
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    EXPECT_TRUE(runtime.liftoff);
}

TEST_F(AdrcUnittest, B0ThrottleScaleTracksSquareOfThrottleRatioAboveHover)
{
    profile.hoverThrottlePercent = 35;

    simulatedThrottle = 0.35f; // at hover
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    EXPECT_NEAR(1.0f, runtime.b0ThrottleScale, 1e-4f);

    simulatedThrottle = 0.70f; // 2x hover -> scale should be 2^2 = 4
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    EXPECT_NEAR(4.0f, runtime.b0ThrottleScale, 1e-3f);
}

TEST_F(AdrcUnittest, B0ThrottleScaleNeverGoesBelowOne)
{
    profile.hoverThrottlePercent = 35;
    simulatedThrottle = 0.0f; // below hover
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    EXPECT_FLOAT_EQ(1.0f, runtime.b0ThrottleScale);
}

TEST_F(AdrcUnittest, B0ThrottleScaleClampsToMax)
{
    profile.hoverThrottlePercent = 10; // low hover setting so full throttle exceeds the clamp
    simulatedThrottle = 1.0f;          // ratio = 10, ratio^2 = 100 -> clamps to 9
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    EXPECT_NEAR(9.0f, runtime.b0ThrottleScale, 1e-3f);
}

TEST_F(AdrcUnittest, Z3LeakyDecayBleedsTowardZero)
{
    profile.sigmaDecay = 30; // decay rate 3.0 - a fast leak, easy to observe over few iterations
    adrcInitConfig(&profile, &runtime, TEST_DT);
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        adrcResetState(&runtime, axis);
    }
    // Exercise the profile's own configured decay rate, not the (also nonzero) gated decay rate
    // that applies while grounded - see ADRC_GATED_Z3_DECAY_RATE in adrc.c.
    runtime.liftoff = true;

    runtime.z3[FD_ROLL] = 1000.0f;
    // With gyroRate == z1 (errorEso == 0), only the decay term drives z3, isolating its effect.
    for (int i = 0; i < 50; i++) {
        adrcApplyControl(&runtime, FD_ROLL, runtime.z1[FD_ROLL], runtime.z1[FD_ROLL], TEST_DT, 500.0f);
    }
    EXPECT_LT(fabsf(runtime.z3[FD_ROLL]), 1000.0f);
}

TEST_F(AdrcUnittest, Z3DecaysFasterWhenGroundedThanConfiguredDecayRate)
{
    // sigmaDecay = 0 (pure integrator) is the worst case for windup while grounded - the gated
    // decay must override it regardless of what the profile configures.
    profile.sigmaDecay = 0;
    adrcInitConfig(&profile, &runtime, TEST_DT);
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        adrcResetState(&runtime, axis);
    }
    ASSERT_FALSE(runtime.liftoff);

    runtime.z3[FD_ROLL] = 1000.0f;
    // With gyroRate == z1 (errorEso == 0), only the decay term drives z3, isolating its effect.
    adrcApplyControl(&runtime, FD_ROLL, runtime.z1[FD_ROLL], runtime.z1[FD_ROLL], TEST_DT, 500.0f);
    // A single step at the gated rate (20/s) should visibly erode z3 even though sigmaDecay == 0
    // would otherwise leave it untouched (see Z3IsPureIntegratorWhenDecayDisabled, airborne case).
    EXPECT_LT(runtime.z3[FD_ROLL], 1000.0f);
}

TEST_F(AdrcUnittest, Z3IsPureIntegratorWhenDecayDisabled)
{
    profile.sigmaDecay = 0;
    adrcInitConfig(&profile, &runtime, TEST_DT);
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        adrcResetState(&runtime, axis);
    }
    // sigmaDecay only governs z3's decay while airborne - while ungated, z3 always uses the much
    // faster ADRC_GATED_Z3_DECAY_RATE regardless of this setting, so it can't wind up while
    // grounded (see adrc.c). Simulate airborne so sigmaDecay == 0 actually yields a pure
    // integrator here.
    runtime.liftoff = true;

    runtime.z3[FD_ROLL] = 1000.0f;
    const float z3Before = runtime.z3[FD_ROLL];
    // errorEso == 0 (gyroRate == z1) and no decay -> z3 must not move at all.
    adrcApplyControl(&runtime, FD_ROLL, runtime.z1[FD_ROLL], runtime.z1[FD_ROLL], TEST_DT, 500.0f);
    EXPECT_FLOAT_EQ(z3Before, runtime.z3[FD_ROLL]);
}

TEST_F(AdrcUnittest, GateBlocksB0uFeedbackWhenClosed)
{
    // Grounded (gate closed): a large lastOutput must not feed b0*u into z2.
    ASSERT_FALSE(runtime.liftoff);
    runtime.lastOutput[FD_ROLL] = 500.0f;
    const float z2Before = runtime.z2[FD_ROLL];
    adrcApplyControl(&runtime, FD_ROLL, 0.0f, 0.0f, TEST_DT, 500.0f);
    // With errorEso == 0 (z1 starts at gyro==0) and the gate closed, z2 should not move from the
    // b0*u term - only from -beta2*errorEso, which is also 0 here.
    EXPECT_FLOAT_EQ(z2Before, runtime.z2[FD_ROLL]);
}

TEST_F(AdrcUnittest, TdDisabledByDefaultTracksSetpointExactly)
{
    // tdHz == 0 (the default) must bypass the tracking differentiator entirely - vRef should
    // follow a setpoint step with zero lag, matching pre-TD behavior exactly.
    ASSERT_EQ(0, profile.tdHz);
    adrcApplyControl(&runtime, FD_ROLL, 0.0f, 500.0f, TEST_DT, 500.0f);
    EXPECT_FLOAT_EQ(500.0f, runtime.vRef[FD_ROLL]);
}

TEST_F(AdrcUnittest, TdEnabledSmoothsSetpointStep)
{
    // With the TD enabled, a setpoint step must not appear in vRef instantly - it should lag
    // behind, unlike the disabled (direct passthrough) case above. tdHz is kept low relative to
    // TEST_DT (125 Hz-equivalent, deliberately slower than any real flight-loop rate) so the
    // discrete Euler integration (dT*tdGain) stays well inside its stable region instead of
    // numerically overshooting - at real loop rates (~1.6 kHz+) this isn't a practical concern
    // at any sane adrc_td_hz value.
    profile.tdHz = 5;
    adrcInitConfig(&profile, &runtime, TEST_DT);
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        adrcResetState(&runtime, axis);
    }

    adrcApplyControl(&runtime, FD_ROLL, 0.0f, 500.0f, TEST_DT, 500.0f);
    EXPECT_GT(500.0f, runtime.vRef[FD_ROLL]);
    EXPECT_LT(0.0f, runtime.vRef[FD_ROLL]);

    // Run it long enough to settle - vRef should converge on the setpoint once it stops moving.
    for (int i = 0; i < 500; i++) {
        adrcApplyControl(&runtime, FD_ROLL, 0.0f, 500.0f, TEST_DT, 500.0f);
    }
    EXPECT_NEAR(500.0f, runtime.vRef[FD_ROLL], 1.0f);
}
