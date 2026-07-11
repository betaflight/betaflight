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

    float mixerGetAdrcThrottle(void) { return simulatedThrottle; }
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
    simulatedThrottle = 0.5f; // above the default liftoffThrottlePercent (40%)
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    EXPECT_TRUE(runtime.liftoff);
}

TEST_F(AdrcUnittest, LiftoffThrottleThresholdIsConfigurable)
{
    profile.liftoffThrottlePercent = 70;

    simulatedThrottle = 0.5f; // above the default (40%) but below this profile's 70%
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    EXPECT_FALSE(runtime.liftoff);

    simulatedThrottle = 0.75f;
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    EXPECT_TRUE(runtime.liftoff);
}

TEST_F(AdrcUnittest, GateOpensOnSustainedRotationWithoutThrottle)
{
    simulatedThrottle = 0.0f;
    gyro.gyroADCf[FD_ROLL] = 25.0f; // above the default liftoffGyroDps (20)

    // A single loop is shorter than the default liftoffHoldMs (25ms) sustain requirement.
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    EXPECT_FALSE(runtime.liftoff);

    // After enough loops the sustained-rotation hold is satisfied (toss-launch path).
    for (int i = 0; i < 10; i++) {
        adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    }
    EXPECT_TRUE(runtime.liftoff);
}

TEST_F(AdrcUnittest, LiftoffGyroAndHoldThresholdsAreConfigurable)
{
    profile.liftoffGyroDps = 50;
    profile.liftoffHoldMs = 100; // needs a longer sustain than the default 25ms

    simulatedThrottle = 0.0f;
    gyro.gyroADCf[FD_ROLL] = 25.0f; // above the default (20dps) but below this profile's 50dps
    for (int i = 0; i < 20; i++) {
        adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    }
    EXPECT_FALSE(runtime.liftoff);

    gyro.gyroADCf[FD_ROLL] = 60.0f;
    // 100ms / 8ms per loop = 12.5 loops - 10 loops (80ms) must not yet satisfy the longer hold.
    for (int i = 0; i < 10; i++) {
        adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    }
    EXPECT_FALSE(runtime.liftoff);

    for (int i = 0; i < 10; i++) {
        adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    }
    EXPECT_TRUE(runtime.liftoff);
}

TEST_F(AdrcUnittest, GateStaysOpenThroughSustainedFloatByDefault)
{
    // Simulate already airborne.
    simulatedThrottle = 0.6f;
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    ASSERT_TRUE(runtime.liftoff);

    // A smooth ballistic float: zero throttle, near-zero rotation, sustained well past any
    // plausible re-arm hold (1.6s). Indistinguishable from a landing by throttle+gyro alone -
    // the first freestyle log on this branch hit exactly this three times (1-4 deg/s floats over
    // 500ms) and lost its live z3 each time. With the default liftoffIdleHoldMs = 0 the mid-air
    // re-arm is disabled outright, so the gate must stay open until disarm.
    simulatedThrottle = 0.0f;
    resetGyro();
    for (int i = 0; i < 200; i++) {
        adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    }
    EXPECT_TRUE(runtime.liftoff);
}

TEST_F(AdrcUnittest, OptInReArmAfterSustainedIdleStillness)
{
    profile.liftoffIdleHoldMs = 500; // opt into the bench/ground-rep re-arm

    // Simulate already airborne.
    simulatedThrottle = 0.6f;
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    ASSERT_TRUE(runtime.liftoff);

    // Idle throttle and stillness, sustained past the configured hold (500ms @ 8ms = 63 loops).
    simulatedThrottle = 0.0f;
    resetGyro();
    for (int i = 0; i < 70; i++) {
        adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    }
    EXPECT_FALSE(runtime.liftoff);
}

TEST_F(AdrcUnittest, LiftoffIdleThresholdsAreConfigurable)
{
    profile.liftoffIdleThrottlePercent = 20; // much higher than the default (5%) - easier to satisfy
    profile.liftoffIdleHoldMs = 1000;        // longer than the default (500ms)

    simulatedThrottle = 0.6f;
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    ASSERT_TRUE(runtime.liftoff);

    // 10% throttle satisfies this profile's raised 20% idle threshold (it would NOT satisfy the
    // default 5%), but idleS accumulates toward this profile's 1000ms hold - 100 loops @ 8ms =
    // 800ms, comfortably under 1000ms, so the gate must still be armed.
    simulatedThrottle = 0.10f;
    resetGyro();
    for (int i = 0; i < 100; i++) {
        adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    }
    EXPECT_TRUE(runtime.liftoff);

    // idleS keeps accumulating across calls as long as the idle condition holds (it never breaks
    // here) - 20 more loops brings the cumulative total to 960ms, still just under the 1000ms hold.
    for (int i = 0; i < 20; i++) {
        adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    }
    EXPECT_TRUE(runtime.liftoff);

    // 10 more loops crosses the cumulative 1000ms mark (1040ms total) - now it must re-arm.
    for (int i = 0; i < 10; i++) {
        adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    }
    EXPECT_FALSE(runtime.liftoff);
}

TEST_F(AdrcUnittest, GateDoesNotReArmOnBriefAirborneThrottleChop)
{
    profile.liftoffIdleHoldMs = 500; // opt into the re-arm - the idle-timer reset is what's under test

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
    profile.b0ThrottleScaleMax = 9; // raise the ceiling out of the way - the quadratic law itself is under test

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
    // The default ceiling is 3: the quadratic law was only community-validated up to ~x3, and the
    // first freestyle log showed the extrapolated region under-gains the control law severalfold
    // on throttle punches (see adrcResetProfile()).
    profile.hoverThrottlePercent = 10; // low hover setting so full throttle exceeds the clamp
    simulatedThrottle = 1.0f;          // ratio = 10, ratio^2 = 100 -> clamps to the default max (3)
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    EXPECT_NEAR(3.0f, runtime.b0ThrottleScale, 1e-3f);
}

TEST_F(AdrcUnittest, B0ThrottleScaleMaxIsConfigurable)
{
    profile.hoverThrottlePercent = 10; // ratio = 10, ratio^2 = 100 at full throttle
    profile.b0ThrottleScaleMax = 20;
    simulatedThrottle = 1.0f;
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    EXPECT_NEAR(20.0f, runtime.b0ThrottleScale, 1e-3f);
}

TEST_F(AdrcUnittest, Z3LeakyDecayBleedsTowardZero)
{
    profile.sigmaDecay = 30; // decay rate 3.0 - a fast leak, easy to observe over few iterations
    adrcInitConfig(&profile, &runtime, TEST_DT);
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        adrcResetState(&runtime, axis);
    }
    // Exercise the profile's own configured decay rate, not the (also nonzero) gated decay rate
    // that applies while grounded - see gatedZ3DecayRate in adrc.h.
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
    // A single step at the default gated rate (20/s) should visibly erode z3 even though
    // sigmaDecay == 0 would otherwise leave it untouched (see Z3IsPureIntegratorWhenDecayDisabled,
    // airborne case).
    EXPECT_LT(runtime.z3[FD_ROLL], 1000.0f);
}

TEST_F(AdrcUnittest, GatedZ3DecayHasProtectiveFloor)
{
    profile.gatedZ3DecayRate = 0; // attempt to disable the gated decay entirely
    profile.sigmaDecay = 0;       // and the airborne decay - the worst case for grounded windup
    adrcInitConfig(&profile, &runtime, TEST_DT);
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        adrcResetState(&runtime, axis);
    }
    ASSERT_FALSE(runtime.liftoff); // grounded - the gated rate is the only one that would apply

    // adrc_gated_z3_decay = 0 must NOT disable the grounded anti-windup - it is floored (tau ~1s)
    // so z3 cannot hold a wound-up value while the craft sits armed at idle.
    EXPECT_FLOAT_EQ(1.0f, runtime.coefficient[FD_ROLL].gatedDecayRate);

    // One step (errorEso == 0, so only the decay term drives z3): 1000 * (1 - 1.0*0.008) = 992 -
    // decaying at the floor rate, neither untouched (the pre-fix 0-disables-it behavior) nor at
    // the default 20/s (which would give 840).
    runtime.z3[FD_ROLL] = 1000.0f;
    adrcApplyControl(&runtime, FD_ROLL, runtime.z1[FD_ROLL], runtime.z1[FD_ROLL], TEST_DT, 500.0f);
    EXPECT_NEAR(992.0f, runtime.z3[FD_ROLL], 0.5f);

    // And the floor also tracks the airborne decay: gated must never be the slower of the two.
    profile.sigmaDecay = 50;      // airborne 5.0/s
    profile.gatedZ3DecayRate = 10; // grounded 1.0/s - slower than airborne, must be lifted to 5.0/s
    adrcInitConfig(&profile, &runtime, TEST_DT);
    EXPECT_FLOAT_EQ(5.0f, runtime.coefficient[FD_ROLL].gatedDecayRate);
}

TEST_F(AdrcUnittest, GateDoesNotChatterWithInvertedThresholds)
{
    // The CLI cannot cross-validate the two throttle thresholds; if the idle (re-arm) threshold
    // is configured at or above the liftoff threshold, one and the same throttle satisfies both
    // the open and the re-arm condition, and an unsanitized gate would chop the ESO's b0*u
    // feedback at loop rate. The idle threshold must be clamped below the liftoff threshold.
    profile.liftoffThrottlePercent = 40;
    profile.liftoffIdleThrottlePercent = 60; // inverted on purpose
    profile.liftoffIdleHoldMs = 0;           // and no hold, the worst case

    simulatedThrottle = 0.5f; // above liftoff, below the (bogus) idle threshold
    resetGyro();
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    ASSERT_TRUE(runtime.liftoff);
    for (int i = 0; i < 200; i++) {
        adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
        ASSERT_TRUE(runtime.liftoff) << "gate chattered at iteration " << i;
    }
}

TEST_F(AdrcUnittest, GateReArmStillnessIsCappedIndependently)
{
    // liftoffGyroDps doubles as the re-arm "stillness" bound; pushing it up for less sensitive
    // liftoff detection must not redefine 100 deg/s of rotation as "still" - that would re-arm
    // the gate mid-air during a throttle chop. The re-arm side is capped at a true-stillness
    // level regardless of this setting.
    profile.liftoffGyroDps = 255;

    simulatedThrottle = 0.6f;
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    ASSERT_TRUE(runtime.liftoff);

    // Mid-air throttle chop: idle throttle, but the craft is clearly still flying/rotating.
    simulatedThrottle = 0.0f;
    gyro.gyroADCf[FD_ROLL] = 100.0f;
    for (int i = 0; i < 300; i++) { // 2.4s - far beyond any hold time
        adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    }
    EXPECT_TRUE(runtime.liftoff);
}

TEST_F(AdrcUnittest, GateReArmHoldHasFloor)
{
    // An absurdly short opt-in hold must not allow a single quiet loop mid-chop to close the gate
    // in flight - nonzero holds are floored so re-arm still requires a sustained idle-and-still
    // period. (0 is not floored: it disables the re-arm outright - covered by
    // GateStaysOpenThroughSustainedFloatByDefault.)
    profile.liftoffIdleHoldMs = 1;

    simulatedThrottle = 0.6f;
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    ASSERT_TRUE(runtime.liftoff);

    simulatedThrottle = 0.0f;
    resetGyro();
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT); // one quiet loop (8ms < 100ms floor)
    EXPECT_TRUE(runtime.liftoff);

    // But a genuinely sustained idle still re-arms (dominated by the 100ms floor, not 500ms).
    for (int i = 0; i < 20; i++) { // +160ms
        adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    }
    EXPECT_FALSE(runtime.liftoff);
}

TEST_F(AdrcUnittest, Z3IsPureIntegratorWhenDecayDisabled)
{
    profile.sigmaDecay = 0;
    adrcInitConfig(&profile, &runtime, TEST_DT);
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        adrcResetState(&runtime, axis);
    }
    // sigmaDecay only governs z3's decay while airborne - while ungated, z3 always uses the much
    // faster gatedZ3DecayRate regardless of this setting, so it can't wind up while grounded (see
    // adrc.c). Simulate airborne so sigmaDecay == 0 actually yields a pure integrator here.
    runtime.liftoff = true;

    runtime.z3[FD_ROLL] = 1000.0f;
    const float z3Before = runtime.z3[FD_ROLL];
    // errorEso == 0 (gyroRate == z1) and no decay -> z3 must not move at all.
    adrcApplyControl(&runtime, FD_ROLL, runtime.z1[FD_ROLL], runtime.z1[FD_ROLL], TEST_DT, 500.0f);
    EXPECT_FLOAT_EQ(z3Before, runtime.z3[FD_ROLL]);
}

TEST_F(AdrcUnittest, ResetSeedsGyroFilterSoHandoverHasNoEsoKick)
{
    // adrcResetState() must seed the gyro filter together with z1: seeding z1 from the raw gyro
    // while the filter re-converges from zero would produce errorEso ~ gyro on the very next
    // loop, kicking z3 by -beta3*errorEso*dT (~ -122 000 at 1000 deg/s and 8 kHz) - the opposite
    // of the smooth handover the reset exists for (mid-air profile switch into ADRC).
    constexpr float dT = 0.000125f; // 8 kHz
    adrcInitConfig(&profile, &runtime, dT);

    gyro.gyroADCf[FD_ROLL] = 1000.0f;
    adrcResetState(&runtime, FD_ROLL);

    adrcApplyControl(&runtime, FD_ROLL, gyro.gyroADCf[FD_ROLL], 1000.0f, dT, 500.0f);
    EXPECT_NEAR(0.0f, runtime.z3[FD_ROLL], 100.0f);   // no false disturbance kick
    EXPECT_NEAR(1000.0f, runtime.z1[FD_ROLL], 10.0f); // observer starts on the measurement
}

TEST_F(AdrcUnittest, GyroLpfZeroIsPassThroughNotFrozen)
{
    // 0 means "filter disabled" everywhere else in Betaflight; a naive pt2FilterGain(0, dT) == 0
    // would instead freeze the ESO's gyro input at zero - a blind controller that flies away on
    // arm. With the filter bypassed the observer must still see the gyro and start tracking it.
    profile.gyroFilterHz = 0;
    adrcInitConfig(&profile, &runtime, TEST_DT);
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        adrcResetState(&runtime, axis);
    }

    adrcApplyControl(&runtime, FD_ROLL, 100.0f, 0.0f, TEST_DT, 500.0f);
    EXPECT_GT(runtime.z1[FD_ROLL], 0.0f); // frozen-at-zero input would leave z1 exactly 0
}

TEST_F(AdrcUnittest, Z2TracksHardManeuverAcceleration)
{
    // A hard 5" snap produces 12-25k deg/s^2 of real angular acceleration; feed a 20 000 deg/s^2
    // gyro ramp at a realistic 8 kHz looptime and require z2 to actually track it. An
    // authority-derived z2 bound (pidSumLimit*b0/kd = 500*2000/120 ~ 8 300 deg/s^2 at the shipped
    // defaults) rails well below this and would fail here.
    constexpr float dT = 0.000125f; // 8 kHz
    adrcInitConfig(&profile, &runtime, dT);
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        adrcResetState(&runtime, axis);
    }

    float rate = 0.0f;
    for (int i = 0; i < 800; i++) { // 0.1 s: 0 -> 2000 deg/s
        rate += 20000.0f * dT;
        adrcApplyControl(&runtime, FD_ROLL, rate, rate, dT, 500.0f);
    }
    EXPECT_GT(runtime.z2[FD_ROLL], 10000.0f);
}

TEST_F(AdrcUnittest, ControlTermsAreNotClampedIndividually)
{
    // Mid-snap P legitimately exceeds pidSumLimit while an opposing D partially cancels it; the
    // final authority clamp belongs to the mixer's constrainf(Sum). Per-term clamps would cut the
    // net drive severalfold here (P clamped to 500 with D = -300 nets 200, instead of the
    // validated 900 - 300 = 600).
    constexpr float dT = 0.000125f;
    adrcInitConfig(&profile, &runtime, dT);
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        adrcResetState(&runtime, axis);
    }

    runtime.z2[FD_ROLL] = 5000.0f; // plausible mid-snap acceleration estimate
    // gyro == z1 == 0 (errorEso == 0), so the preset states pass through the ESO update unchanged.
    const adrcOutput_t out = adrcApplyControl(&runtime, FD_ROLL, 0.0f, 500.0f, dT, 500.0f);
    EXPECT_GT(out.P, 500.0f);          // kp*err/b0 = 3600*500/2000 = 900 - must not clamp to 500
    EXPECT_NEAR(-300.0f, out.D, 5.0f); // -kd*z2/b0 = -120*5000/2000
}

TEST_F(AdrcUnittest, InitConfigFloorsOutOfRangeCoefficients)
{
    // The CLI table constrains wc/wo/b0 to sane minimums, but values can still arrive out of
    // range via PG/EEPROM (e.g. a stale save from before the floors were added). adrcInitConfig()
    // must floor them itself as defense-in-depth: wo == 0 would freeze the observer outright (all
    // betas 0, z1 stuck at its arm-time value while P keeps acting on it), and wc == 0 would zero
    // the whole control law - both fail silently, with nothing in the CLI to hint at why.
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        profile.wc[axis] = 0;
        profile.wo[axis] = 0;
        profile.b0[axis] = 0;
    }
    adrcInitConfig(&profile, &runtime, TEST_DT);

    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        EXPECT_GE(runtime.coefficient[axis].wc, 5.0f);
        EXPECT_GE(runtime.coefficient[axis].wo, 10.0f);
        EXPECT_GE(runtime.coefficient[axis].b0, 100.0f);
        // Derived gains must be computed from the floored values, not the raw zeros.
        EXPECT_GT(runtime.coefficient[axis].kp, 0.0f);
        EXPECT_GT(runtime.coefficient[axis].kd, 0.0f);
        EXPECT_GT(runtime.coefficient[axis].beta1, 0.0f);
        EXPECT_GT(runtime.coefficient[axis].beta2, 0.0f);
        EXPECT_GT(runtime.coefficient[axis].beta3, 0.0f);
    }
}

TEST_F(AdrcUnittest, InitConfigPassesThroughInRangeCoefficientsUnmodified)
{
    // The floor must not perturb legitimate in-range tunes.
    profile.wc[FD_ROLL] = 60;
    profile.wo[FD_ROLL] = 100;
    profile.b0[FD_ROLL] = 2000;
    adrcInitConfig(&profile, &runtime, TEST_DT);

    EXPECT_FLOAT_EQ(60.0f, runtime.coefficient[FD_ROLL].wc);
    EXPECT_FLOAT_EQ(100.0f, runtime.coefficient[FD_ROLL].wo);
    EXPECT_FLOAT_EQ(2000.0f, runtime.coefficient[FD_ROLL].b0);
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
