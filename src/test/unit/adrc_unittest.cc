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
// Commanded collective, tracked separately from the applied one above: airmode raises the applied
// value on the ground while the commanded one stays at zero, and the gate keys on it (ADRC-026).
static float simulatedCommandedThrottle = 0.0f;

extern "C" {
    #include "common/axis.h"
    #include "common/maths.h"
    #include "common/filter.h"

    #include "build/debug.h"

    #include "sensors/gyro.h"

    #include "flight/adrc.h"

    gyro_t gyro;

    float mixerGetAdrcThrottle(void) { return simulatedThrottle; }
    float mixerGetAdrcCommandedThrottle(void) { return simulatedCommandedThrottle; }
}

#include <algorithm>

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
        runtime = {};
        simulatedThrottle = 0.0f;
        simulatedCommandedThrottle = 0.0f;
        resetGyro();
        adrcResetProfile(&profile);
        adrcInitConfig(&profile, &runtime, TEST_DT);
        for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
            adrcResetState(&runtime, axis);
        }
        adrcResetGate(&runtime);
    }

    // The b0 schedule reads a low-passed collective (~80 ms tau); run enough loops for the filter
    // to fully settle on the current simulatedThrottle before asserting steady-state values.
    void settleB0ThrottleScale()
    {
        for (int i = 0; i < 400; i++) {
            adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
        }
    }

    // Feed the ADRC-026 excitation - a ~31 Hz square-wave rotation the observer cannot follow, the
    // measured ground oscillation's signature - and report the largest |z3| reached. The gate state
    // and throttleAtIdle are left as the caller set them.
    float drivePeakZ3UnderGroundOscillation(int loops = 200, float amplitudeDps = 120.0f)
    {
        float peakZ3 = 0.0f;
        for (int i = 0; i < loops; i++) {
            const float gyroRate = ((i % 4) < 2) ? amplitudeDps : -amplitudeDps;
            adrcApplyControl(&runtime, FD_ROLL, gyroRate, 0.0f, TEST_DT, 500.0f);
            peakZ3 = fmaxf(peakZ3, std::fabs(runtime.z3[FD_ROLL]));
        }
        return peakZ3;
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
    // Real pilot throttle: with no axis demand the mixer adds no headroom, so applied == commanded.
    simulatedThrottle = 0.5f; // above the default liftoffThrottlePercent (40%)
    simulatedCommandedThrottle = 0.5f;
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    EXPECT_TRUE(runtime.liftoff);
}

TEST_F(AdrcUnittest, GateOpenDropsGroundEpochOutputBeforeFirstControlStep)
{
    // Reproduce the logged Airmode takeoff failure: the gate opens while the observer still
    // carries a nonzero command from the ground-constrained previous loop. The observer itself
    // remains live and coherent while gated; only that stale actuator input must not cross epochs.
    gyro.gyroADCf[FD_ROLL] = 300.0f;
    runtime.gyroFilter[FD_ROLL].state = 300.0f;
    runtime.gyroFilter[FD_ROLL].state1 = 300.0f;
    runtime.z1[FD_ROLL] = 300.0f;
    runtime.vRef[FD_ROLL] = 300.0f;
    runtime.lastOutput[FD_ROLL] = 500.0f;

    simulatedThrottle = 0.5f;
    simulatedCommandedThrottle = 0.5f;
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);

    ASSERT_TRUE(runtime.liftoff);
    EXPECT_FLOAT_EQ(300.0f, runtime.gyroFilter[FD_ROLL].state);
    EXPECT_FLOAT_EQ(300.0f, runtime.gyroFilter[FD_ROLL].state1);
    EXPECT_FLOAT_EQ(300.0f, runtime.z1[FD_ROLL]);
    EXPECT_FLOAT_EQ(0.0f, runtime.z2[FD_ROLL]);
    EXPECT_FLOAT_EQ(0.0f, runtime.z3[FD_ROLL]);
    EXPECT_FLOAT_EQ(300.0f, runtime.vRef[FD_ROLL]);
    EXPECT_FLOAT_EQ(0.0f, runtime.lastOutput[FD_ROLL]);

    // With measurement == setpoint, the first airborne control step must be exactly neutral;
    // any output here comes solely from stale observer/output state crossing the gate.
    const adrcOutput_t output = adrcApplyControl(&runtime, FD_ROLL, 300.0f, 300.0f, TEST_DT, 500.0f);
    EXPECT_FLOAT_EQ(0.0f, output.P);
    EXPECT_FLOAT_EQ(0.0f, output.I);
    EXPECT_FLOAT_EQ(0.0f, output.D);
}

TEST_F(AdrcUnittest, LiftoffThrottleThresholdIsConfigurable)
{
    profile.liftoffThrottlePercent = 70;

    simulatedThrottle = simulatedCommandedThrottle = 0.5f; // above the default (40%), below this profile's 70%
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    EXPECT_FALSE(runtime.liftoff);

    simulatedThrottle = simulatedCommandedThrottle = 0.75f;
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    EXPECT_TRUE(runtime.liftoff);
}

TEST_F(AdrcUnittest, GateOpensOnSustainedRotationBelowLiftoffThrottle)
{
    simulatedThrottle = 0.0f;
    // The toss-launch path still opens the gate well below liftoffThrottlePercent (40%), but it now
    // also requires the stick to be above half of it (ADRC-026); 25% satisfies that and would never
    // open the direct throttle branch on its own.
    simulatedCommandedThrottle = 0.25f;
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
    simulatedCommandedThrottle = 0.25f; // clears the gyro path's throttle floor (half of 40%)
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

TEST_F(AdrcUnittest, GateStaysOpenThroughSustainedFloat)
{
    // Simulate already airborne.
    simulatedThrottle = simulatedCommandedThrottle = 0.6f;
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    ASSERT_TRUE(runtime.liftoff);

    // A smooth ballistic float: zero throttle, near-zero rotation, sustained well past a second -
    // indistinguishable from a landing by throttle+gyro alone. The first freestyle log on this
    // branch hit exactly this three times (1-4 deg/s floats over 500ms) and lost its live z3 each
    // time before the opt-in mid-air re-arm heuristic that used to cause this was removed entirely
    // (ADRC-020); the gate now has no re-arm path at all and must stay open until disarm.
    simulatedThrottle = 0.0f;
    resetGyro();
    for (int i = 0; i < 200; i++) {
        adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    }
    EXPECT_TRUE(runtime.liftoff);
}

// --- ADRC-026: ground excitation must not open the gate or charge z3 at idle stick ---

TEST_F(AdrcUnittest, AirmodeHeadroomAloneDoesNotOpenGate)
{
    // The logged failure, reduced to its mechanism: on the ground under airmode the mixer raises the
    // APPLIED collective past liftoffThrottlePercent while nothing was commanded. Re-decoding the
    // wo = 150 arms put the applied proxy at 30.3-32.2% against their adrc_liftoff_throttle = 30 at
    // a throttle stick that never moved, so this - not the gyro path - is what opened the gate.
    // Sustained, not bursty: this is the hard case for the applied-collective path, and it is what
    // a wedged craft and launch control both produce. With the throttle stick down the idle
    // interlock must reject it however long it lasts - duration alone cannot tell this apart from
    // flight.
    profile.liftoffThrottlePercent = 30;    // as flown in those logs
    simulatedThrottle = 0.32f;              // applied: airmode headroom, past the 30% threshold
    simulatedCommandedThrottle = 0.0f;      // commanded: nothing asked for it

    for (int i = 0; i < 400; i++) { // 3.2 s, an order past both holds
        gyro.gyroADCf[FD_ROLL] = (i & 1) ? 120.0f : -120.0f;
        adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
        EXPECT_FALSE(runtime.liftoff);
    }
    EXPECT_FLOAT_EQ(0.0f, runtime.gyroActiveS);
    EXPECT_FLOAT_EQ(0.0f, runtime.appliedActiveS);
}

TEST_F(AdrcUnittest, AppliedCollectiveHoldOutlastsTheMeasuredGroundBursts)
{
    // Pins ADRC_LIFTOFF_APPLIED_HOLD_S from below. Across the ten 2026-08-06 logs the longest
    // unbroken run above the threshold before the pilot first moved the stick was 43.5 ms at a 40%
    // threshold and 68.2 ms at 25%; the bursts here are ~104 ms, longer than that worst case with
    // margin, so shortening the hold to anything near those measurements fails this test. The gyro
    // path is kept out of the way (rate below liftoffGyroDps) so only the duration test is exercised.
    profile.liftoffThrottlePercent = 30;
    simulatedCommandedThrottle = 0.20f;     // above the idle floor, below the 30% threshold
    gyro.gyroADCf[FD_ROLL] = 5.0f;          // well under the 20 dps gyro path

    // 1250 loops = 10 s, forty times the hold. Duration matters here: a timer that leaked instead
    // of resetting would integrate any duty cycle above 50% and latch eventually, so a short run
    // would pass while the guarantee was gone.
    for (int i = 0; i < 1250; i++) {
        simulatedThrottle = ((i % 25) < 13) ? 0.32f : 0.10f; // ~104 ms above, ~96 ms below
        adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
        EXPECT_FALSE(runtime.liftoff);
    }
}

TEST_F(AdrcUnittest, AppliedCollectiveHoldIsNotAnIntegratorOfDutyCycle)
{
    // The hold must mean "continuously above the threshold", not "above it more often than not".
    // A timer that drained at its fill rate would latch on any duty cycle over 50% - 1.8 s at 55%,
    // 0.9 s at 60% - and ground oscillation is exactly that kind of signal, so the margin the
    // measured burst lengths provide would be void.
    profile.liftoffThrottlePercent = 30;
    simulatedCommandedThrottle = 0.20f;
    gyro.gyroADCf[FD_ROLL] = 5.0f;

    for (int i = 0; i < 2500; i++) { // 20 s at 60% duty (24 loops above, 16 below)
        simulatedThrottle = ((i % 40) < 24) ? 0.32f : 0.10f;
        adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
        EXPECT_FALSE(runtime.liftoff);
    }
}

TEST_F(AdrcUnittest, AppliedCollectiveHoldHonoursLiftoffHoldMs)
{
    // A pilot who hardens the gate after a false open must not find that this path still latches at
    // the built-in 250 ms; the blackbox header would report a hold that was not in force either.
    profile.liftoffThrottlePercent = 30;
    profile.liftoffHoldMs = 500;            // longer than ADRC_LIFTOFF_APPLIED_HOLD_S
    simulatedThrottle = 0.32f;
    simulatedCommandedThrottle = 0.20f;

    for (int i = 0; i < 50; i++) { // 400 ms - past the built-in hold, short of the configured one
        adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
        EXPECT_FALSE(runtime.liftoff);
    }
    for (int i = 0; i < 20; i++) { // past 500 ms
        adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    }
    EXPECT_TRUE(runtime.liftoff);
}

TEST_F(AdrcUnittest, SustainedAppliedCollectiveOpensGateBelowTheCommandedThreshold)
{
    // The other half of ADRC-026, and the reason the commanded collective cannot be the only input:
    // thrust the mixer applied beyond what the pilot commanded still lifts the craft. Reading only
    // the commanded value would leave the gate shut for the rest of the arm cycle, flying the
    // observer with b0*u held at zero - worse than the false open the test above brackets.
    profile.liftoffThrottlePercent = 30;
    simulatedThrottle = 0.32f;              // applied: sustained past the threshold, as in flight
    simulatedCommandedThrottle = 0.20f;     // commanded: real thrust asked for, but under threshold

    // Short of the hold the gate stays shut...
    for (int i = 0; i < 20; i++) { // 160 ms
        adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
        EXPECT_FALSE(runtime.liftoff);
    }
    // ...and opens once the applied collective has held long enough to mean flight.
    for (int i = 0; i < 20; i++) {
        adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    }
    EXPECT_TRUE(runtime.liftoff);
}

TEST_F(AdrcUnittest, GyroHoldTimerCannotBeBankedBelowTheThrottleFloor)
{
    // Rotation sustained at idle must not accumulate hold time that a later throttle blip completes:
    // the timer resets whenever commanded throttle sits below the floor, so the hold restarts.
    simulatedCommandedThrottle = 0.0f;
    gyro.gyroADCf[FD_ROLL] = 25.0f;
    for (int i = 0; i < 50; i++) {
        adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    }
    ASSERT_FALSE(runtime.liftoff);

    // One loop (8 ms) above the floor is still short of the 25 ms hold.
    simulatedCommandedThrottle = 0.25f;
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    EXPECT_FALSE(runtime.liftoff);
}

TEST_F(AdrcUnittest, AutomaticModeThrottleStillOpensGateWithStickAtIdle)
{
    // ALT_HOLD/GPS_RESCUE command real throttle with the stick at zero. Those overrides are applied
    // upstream of where the mixer samples the commanded collective, so it carries them and the gate
    // still opens - otherwise an autonomous climb would fly with b0*u feedback held at zero.
    simulatedThrottle = 0.5f;
    simulatedCommandedThrottle = 0.5f;
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    EXPECT_TRUE(runtime.liftoff);
}

TEST_F(AdrcUnittest, Z3GrowthIsInhibitedWhileUngatedAtIdleThrottle)
{
    // Drive the observer with the excitation that caused the failure rather than a steady rate: a
    // constant rate is tracked exactly (errorEso -> 0, z3 -> 0 with or without any inhibit), so it
    // would pass this test vacuously. An oscillation the observer cannot follow is what charges z3.
    simulatedCommandedThrottle = 0.0f;
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    ASSERT_FALSE(runtime.liftoff);
    ASSERT_TRUE(runtime.throttleAtIdle);

    const float peakZ3 = drivePeakZ3UnderGroundOscillation();
    // Reference: the same excitation with the gate open charges z3 four orders higher (asserted in
    // Z3StaysLiveThroughAirborneZeroThrottleFloat below).
    EXPECT_LT(peakZ3, 1000.0f);
}

TEST_F(AdrcUnittest, Z3GrowthStaysInhibitedWhileUngatedAboveIdleThrottle)
{
    // The window this closes: throttleAtIdle clears at half the liftoff threshold, so a craft still
    // on the ground with the stick past that point used to charge z3 freely until the gate opened.
    // Measured on a 5" (docs/flight-test-analysis/pr15400-b8-mamba) that window ran 0.6 s unloaded
    // and 5.6 s with a payload, and the estimate plateaus within ~0.25 s of it - so what matters is
    // that the inhibit holds at all, not how long the window is.
    simulatedCommandedThrottle = 0.30f; // above the floor (20%), below liftoffThrottlePercent (40%)
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    ASSERT_FALSE(runtime.liftoff);
    ASSERT_FALSE(runtime.throttleAtIdle); // stick is up; the gate, not the stick, must decide

    // Same excitation that charges z3 four orders higher once the gate is open.
    EXPECT_LT(drivePeakZ3UnderGroundOscillation(), 1000.0f);
}

TEST_F(AdrcUnittest, Z3StaysLiveThroughAirborneZeroThrottleFloat)
{
    // ADRC-020 guard: idle alone must never inhibit z3. Airborne at zero throttle is flying, and its
    // disturbance estimate has to keep tracking - this is the case that broke when the gate itself
    // was allowed to close mid-air.
    simulatedThrottle = simulatedCommandedThrottle = 0.6f;
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    ASSERT_TRUE(runtime.liftoff);

    simulatedThrottle = 0.0f;
    simulatedCommandedThrottle = 0.0f;
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    ASSERT_TRUE(runtime.throttleAtIdle); // idle, but airborne - the inhibit must not engage

    EXPECT_GT(drivePeakZ3UnderGroundOscillation(), 100000.0f);
}

TEST_F(AdrcUnittest, InhibitedZ3StillDecaysFromAWoundUpValue)
{
    // The inhibit blocks growth, not the leak: a z3 charged before the craft settled must still
    // relax toward zero while it sits ungated at idle.
    simulatedCommandedThrottle = 0.0f;
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    ASSERT_FALSE(runtime.liftoff);

    runtime.z3[FD_ROLL] = 50000.0f;
    const float initialZ3 = runtime.z3[FD_ROLL];
    for (int i = 0; i < 50; i++) {
        adrcApplyControl(&runtime, FD_ROLL, 0.0f, 0.0f, TEST_DT, 500.0f);
    }
    EXPECT_LT(std::fabs(runtime.z3[FD_ROLL]), std::fabs(initialZ3) * 0.5f);
}

TEST_F(AdrcUnittest, B0ThrottleScaleTracksSquareOfThrottleRatioAboveHover)
{
    profile.b0Law = ADRC_B0_LAW_QUADRATIC; // the law under test; the b11 default is SQRT
    profile.hoverThrottlePercent = 35;
    profile.b0ThrottleScaleMax = 9; // raise the ceiling out of the way - the quadratic law itself is under test

    simulatedThrottle = 0.35f; // at hover
    settleB0ThrottleScale();
    EXPECT_NEAR(1.0f, runtime.b0ThrottleScale, 1e-4f);

    simulatedThrottle = 0.70f; // 2x hover -> scale should be 2^2 = 4
    settleB0ThrottleScale();
    EXPECT_NEAR(4.0f, runtime.b0ThrottleScale, 1e-3f);
}

TEST_F(AdrcUnittest, B0ThrottleScaleFollowsSelectedLaw)
{
    // ADRC-021 A/B selector: same 2x-hover operating point, one expectation per candidate law.
    // The quadratic default is covered by B0ThrottleScaleTracksSquareOfThrottleRatioAboveHover.
    profile.hoverThrottlePercent = 35;
    profile.b0ThrottleScaleMax = 9; // ceiling out of the way - the law shapes are under test
    simulatedThrottle = 0.70f;      // ratio = 2

    profile.b0Law = ADRC_B0_LAW_SQRT;
    settleB0ThrottleScale();
    EXPECT_NEAR(1.41421f, runtime.b0ThrottleScale, 1e-3f);

    profile.b0Law = ADRC_B0_LAW_LINEAR;
    settleB0ThrottleScale();
    EXPECT_NEAR(2.0f, runtime.b0ThrottleScale, 1e-3f);

    profile.b0Law = ADRC_B0_LAW_FIXED;
    settleB0ThrottleScale();
    EXPECT_FLOAT_EQ(1.0f, runtime.b0ThrottleScale);
}

TEST_F(AdrcUnittest, B0LawAlternativesKeepBothClamps)
{
    // The "never below 1 / never above max" policy is law-independent.
    profile.hoverThrottlePercent = 35;
    profile.b0Law = ADRC_B0_LAW_SQRT;
    simulatedThrottle = 0.0f; // below hover
    settleB0ThrottleScale();
    EXPECT_FLOAT_EQ(1.0f, runtime.b0ThrottleScale);

    profile.hoverThrottlePercent = 10;
    profile.b0Law = ADRC_B0_LAW_LINEAR;
    simulatedThrottle = 1.0f; // ratio = 10 -> clamps to the default max (3)
    settleB0ThrottleScale();
    EXPECT_NEAR(3.0f, runtime.b0ThrottleScale, 1e-3f);
}

TEST_F(AdrcUnittest, B0ThrottleScaleNeverGoesBelowOne)
{
    profile.hoverThrottlePercent = 35;
    simulatedThrottle = 0.0f; // below hover
    settleB0ThrottleScale();
    EXPECT_FLOAT_EQ(1.0f, runtime.b0ThrottleScale);
}

TEST_F(AdrcUnittest, B0ThrottleScaleClampsToMax)
{
    // The default ceiling is 3: the quadratic law was only community-validated up to ~x3, and the
    // first freestyle log showed the extrapolated region under-gains the control law severalfold
    // on throttle punches (see adrcResetProfile()).
    profile.hoverThrottlePercent = 10; // low hover setting so full throttle exceeds the clamp
    simulatedThrottle = 1.0f;          // ratio = 10, ratio^2 = 100 -> clamps to the default max (3)
    settleB0ThrottleScale();
    EXPECT_NEAR(3.0f, runtime.b0ThrottleScale, 1e-3f);
}

TEST_F(AdrcUnittest, B0ThrottleScaleMaxIsConfigurable)
{
    profile.b0Law = ADRC_B0_LAW_QUADRATIC; // the law under test; the b11 default is SQRT
    profile.hoverThrottlePercent = 10; // ratio = 10, ratio^2 = 100 at full throttle
    profile.b0ThrottleScaleMax = 20;
    simulatedThrottle = 1.0f;
    settleB0ThrottleScale();
    EXPECT_NEAR(20.0f, runtime.b0ThrottleScale, 1e-3f);
}

TEST_F(AdrcUnittest, B0ThrottleScaleReleasesGraduallyOnThrottleChop)
{
    profile.b0Law = ADRC_B0_LAW_QUADRATIC; // the law under test; the b11 default is SQRT
    // A throttle chop must not collapse the scale within one loop: the z3 that adapted through
    // the inflated b0 during the high-throttle phase over-applies the moment the divisor snaps
    // back to 1, swinging the craft against the punch (flight-measured ~90 deg/s uncommanded
    // pitch "rebound", 2026-07-12). The low-passed collective (~80 ms tau) releases the scale on
    // the same timescale the ESO re-adapts.
    profile.hoverThrottlePercent = 35;
    profile.b0ThrottleScaleMax = 9;

    simulatedThrottle = 0.70f;
    settleB0ThrottleScale();
    ASSERT_NEAR(4.0f, runtime.b0ThrottleScale, 1e-3f);

    simulatedThrottle = 0.35f; // chop back to hover
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    EXPECT_GT(runtime.b0ThrottleScale, 3.5f);   // still near the pre-chop value one loop later

    settleB0ThrottleScale();
    EXPECT_NEAR(1.0f, runtime.b0ThrottleScale, 1e-3f);
}

TEST_F(AdrcUnittest, B0ThrottleScaleIgnoresLoopRateCollectiveModulation)
{
    // The published collective includes the mixer's per-loop constrain, which tracks the loop's
    // own axis activity (airmode raises collective to make room for the mix). Fed raw, that
    // modulated the effective gain at the ~25 Hz loop resonance (debug d7 swinging 1.0..2.8 at a
    // steady stick - flight A/B 2026-07-12). The b0 schedule must respond to the average, not the
    // per-loop swings.
    profile.hoverThrottlePercent = 35;
    profile.b0ThrottleScaleMax = 9;

    simulatedThrottle = 0.525f; // settle at the mean of the modulation below
    settleB0ThrottleScale();
    const float meanScale = runtime.b0ThrottleScale;

    float minScale = meanScale, maxScale = meanScale;
    for (int i = 0; i < 500; i++) {
        simulatedThrottle = (i & 1) ? 0.70f : 0.35f; // +/-50% collective swing, alternating loops
        adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
        if (i >= 250) { // measure after the transient
            minScale = fminf(minScale, runtime.b0ThrottleScale);
            maxScale = fmaxf(maxScale, runtime.b0ThrottleScale);
        }
    }
    // Unfiltered, the scale would swing the full 1.0..4.0 range; filtered it must stay close to
    // the mean response.
    EXPECT_LT(maxScale - minScale, 0.5f);
    EXPECT_NEAR(meanScale, (maxScale + minScale) * 0.5f, 0.5f);
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
    // of the smooth handover needed after boot, a disarmed controller-type switch, or recovery
    // from invalid observer state.
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
    constexpr float dT = 0.000125f; // 8 kHz, safely above the requested observer bandwidth
    profile.wc[FD_ROLL] = 60;
    profile.wo[FD_ROLL] = 100;
    profile.b0[FD_ROLL] = 2000;
    adrcInitConfig(&profile, &runtime, dT);

    EXPECT_FLOAT_EQ(60.0f, runtime.coefficient[FD_ROLL].wc);
    EXPECT_FLOAT_EQ(100.0f, runtime.coefficient[FD_ROLL].wo);
    EXPECT_FLOAT_EQ(2000.0f, runtime.coefficient[FD_ROLL].b0);
}

TEST_F(AdrcUnittest, InitConfigCapsObserverBandwidthAgainstLoopTime)
{
    profile.wo[FD_ROLL] = 600;

    adrcInitConfig(&profile, &runtime, 1.0f / 200.0f);
    EXPECT_FLOAT_EQ(100.0f, runtime.coefficient[FD_ROLL].wo);

    // The shipped roll/pitch default is exactly safe at the slowest supported 200 Hz loop.
    profile.wo[FD_ROLL] = 100;
    adrcInitConfig(&profile, &runtime, 1.0f / 200.0f);
    EXPECT_FLOAT_EQ(100.0f, runtime.coefficient[FD_ROLL].wo);

    // At 1.6 kHz and faster, the entire CLI range remains available.
    profile.wo[FD_ROLL] = 600;
    adrcInitConfig(&profile, &runtime, 1.0f / 1600.0f);
    EXPECT_FLOAT_EQ(600.0f, runtime.coefficient[FD_ROLL].wo);
}

TEST_F(AdrcUnittest, Z3LogScaleSpansTheProfilesOwnAntiWindupBound)
{
    // ADRC-029. The divisor must be the smallest integer that fits the worst-case z3 bound
    // (pidSumLimit * b0 * b0ThrottleScaleMax, per axis) into the int16 debug field.

    // Shipped defaults: b0 = 2000 everywhere, scale max 3, limits 500/400.
    // Worst axis is roll/pitch: 500 * 2000 * 3 = 3 000 000 -> ceil(/32767) = 92.
    EXPECT_EQ(92u, adrcZ3LogScale(&profile, 500, 400));

    // A flown high-b0 tune (the Air65 corpus that motivated ADRC-029): its roll b0 of 7007
    // clipped the b9-era /16 field in ordinary flight. 500 * 7007 * 3 = 10 510 500 -> 321.
    profile.b0[FD_ROLL] = 7007;
    profile.b0[FD_PITCH] = 4312;
    profile.b0[FD_YAW] = 5848;
    EXPECT_EQ(321u, adrcZ3LogScale(&profile, 500, 400));

    // Small bounds never sharpen the divisor below the legacy 16, so old-log resolution is a floor.
    profile.b0[FD_ROLL] = 100;
    profile.b0[FD_PITCH] = 100;
    profile.b0[FD_YAW] = 100;
    profile.b0ThrottleScaleMax = 1;
    EXPECT_EQ(16u, adrcZ3LogScale(&profile, 500, 400));

    // Review counterexample against the float32 version this replaced: worst = 385 * 28508 * 48
    // = 526 827 840, whose float32 quotient rounds to exactly 16078 before ceilf can act, leaving
    // the endpoint 14 short of the bound. Integer ceil must give 16079.
    profile.b0[FD_ROLL] = 28508;
    profile.b0ThrottleScaleMax = 48;
    EXPECT_EQ(16079u, adrcZ3LogScale(&profile, 385, 400));

    // Second-review counterexample: the runtime clamp is float32, and 976 * 40721 * 49 rounds UP
    // 48 above the exact product - an exact-bound divisor (59433) left the endpoint 41 short of
    // what the firmware actually clamps to. The covering divisor is 59434, and minimal.
    profile.b0[FD_ROLL] = 40721;
    profile.b0ThrottleScaleMax = 49;
    EXPECT_EQ(59434u, adrcZ3LogScale(&profile, 976, 400));

    // Third-review boundary: 151 * 217 * 16 = 524272 exactly, representable exactly in float32,
    // exactly the legacy /16 endpoint. A blanket safety pad wrongly pushed this to /17, costing
    // 6.25 % resolution for nothing; the minimal covering divisor is the legacy 16.
    profile.b0[FD_ROLL] = 217;
    profile.b0[FD_PITCH] = 100;
    profile.b0[FD_YAW] = 100;
    profile.b0ThrottleScaleMax = 16;
    EXPECT_EQ(16u, adrcZ3LogScale(&profile, 151, 151));
}

TEST_F(AdrcUnittest, Z3LogScaleEndpointAlwaysCoversTheBound)
{
    // The one invariant the divisor exists for: scale * 32767 >= pidSumLimit * b0 * scaleMax for
    // the worst axis, over a deterministic sweep of the valid input space (a float32
    // implementation failed this on ~0.06 % of it).
    uint32_t seed = 0x029A5EED;
    for (int i = 0; i < 20000; i++) {
        seed = seed * 1664525u + 1013904223u;               // LCG, deterministic
        const uint16_t limit = 100 + (seed >> 8) % 901;     // 100..1000
        seed = seed * 1664525u + 1013904223u;
        const uint16_t b0 = 100 + (seed >> 8) % 65436;      // 100..65535
        seed = seed * 1664525u + 1013904223u;
        const uint8_t scaleMax = 1 + (seed >> 8) % 50;      // 1..50

        profile.b0[FD_ROLL] = b0;
        profile.b0[FD_PITCH] = 100;
        profile.b0[FD_YAW] = 100;
        profile.b0ThrottleScaleMax = scaleMax;
        const uint64_t bound = (uint64_t)limit * b0 * scaleMax;
        const uint64_t endpoint = (uint64_t)adrcZ3LogScale(&profile, limit, limit) * 32767u;
        ASSERT_GE(endpoint, bound) << "limit=" << limit << " b0=" << b0
                                   << " scaleMax=" << (int)scaleMax;
        // ... and against the clamp as the firmware computes it: float32, in either association
        // -ffast-math might choose (each product here is integer-valued and exactly convertible).
        const float clampA = (float)limit * ((float)b0 * (float)scaleMax);
        const float clampB = ((float)limit * (float)b0) * (float)scaleMax;
        const uint64_t cover = std::max({bound, (uint64_t)clampA, (uint64_t)clampB});
        ASSERT_GE(endpoint, cover) << "limit=" << limit << " b0=" << b0
                                   << " scaleMax=" << (int)scaleMax;
        // ... and it must be the SMALLEST such divisor (above the legacy floor): one step down
        // must fail to cover. This is the other half of the contract, which a coverage-only
        // assertion cannot see.
        const uint32_t divisor = adrcZ3LogScale(&profile, limit, limit);
        if (divisor > 16u) {
            ASSERT_LT((uint64_t)(divisor - 1) * 32767u, cover)
                << "limit=" << limit << " b0=" << b0 << " scaleMax=" << (int)scaleMax;
        }
    }
}

TEST_F(AdrcUnittest, Z3LogScaleReachesTheRuntimeThroughItsOwnInit)
{
    // adrcInitConfig() alone must leave the b9-era divisor (isolated-module callers, incl. these
    // tests), and adrcInitZ3LogScale() - the pidInitConfig() path - must overwrite it.
    adrcInitConfig(&profile, &runtime, 0.000125f);
    EXPECT_FLOAT_EQ(16.0f, runtime.z3LogScale);

    adrcInitZ3LogScale(&runtime, &profile, 500, 400);
    EXPECT_FLOAT_EQ(92.0f, runtime.z3LogScale);
}

TEST_F(AdrcUnittest, ObservabilityCachesTheCollectivesConsumedThisIteration)
{
    simulatedThrottle = 0.35f;
    simulatedCommandedThrottle = 0.10f;
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);

    EXPECT_FLOAT_EQ(0.35f, runtime.observedAppliedCollective);
    EXPECT_FLOAT_EQ(0.10f, runtime.observedCommandedCollective);
    EXPECT_EQ(ADRC_LIFTOFF_CAUSE_NONE, runtime.liftoffCause);
    EXPECT_EQ(ADRC_STATE_THROTTLE_AT_IDLE, adrcStateFlags(&runtime));
}

TEST_F(AdrcUnittest, ObservabilityMarksOnlyAxesWhoseZ3GrowthWasActuallyInhibited)
{
    simulatedThrottle = simulatedCommandedThrottle = 0.0f;
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);

    adrcApplyControl(&runtime, FD_ROLL, 120.0f, 0.0f, TEST_DT, 500.0f);
    adrcApplyControl(&runtime, FD_PITCH, 0.0f, 0.0f, TEST_DT, 500.0f);

    EXPECT_EQ(1u << FD_ROLL, runtime.z3GrowthInhibitMask);
    EXPECT_EQ(ADRC_STATE_THROTTLE_AT_IDLE | ADRC_STATE_Z3_INHIBITED_ROLL,
        adrcStateFlags(&runtime));

    // ADRC-026b keys the inhibit on the closed gate, not on the idle-stick flag. Preserve that
    // distinction in the log: above the throttle floor but below liftoff, a suppressed z3 update
    // must still be reported.
    simulatedThrottle = simulatedCommandedThrottle = 0.25f;
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    ASSERT_FALSE(runtime.throttleAtIdle);
    ASSERT_FALSE(runtime.liftoff);
    adrcApplyControl(&runtime, FD_YAW, 120.0f, 0.0f, TEST_DT, 500.0f);
    EXPECT_EQ(1u << FD_YAW, runtime.z3GrowthInhibitMask);
    EXPECT_EQ(ADRC_STATE_Z3_INHIBITED_YAW, adrcStateFlags(&runtime));

    // The next PID iteration starts with a fresh per-axis event mask.
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    EXPECT_EQ(0u, runtime.z3GrowthInhibitMask);
}

TEST_F(AdrcUnittest, ObservabilityRecordsGateCauseAndResetEpoch)
{
    const uint32_t initialResetCount = runtime.gateResetCount;

    simulatedThrottle = 0.60f;
    simulatedCommandedThrottle = 0.50f;
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    ASSERT_TRUE(runtime.liftoff);
    EXPECT_EQ(ADRC_LIFTOFF_CAUSE_COMMANDED_COLLECTIVE, runtime.liftoffCause);
    EXPECT_EQ(ADRC_STATE_LIFTOFF
            | (ADRC_LIFTOFF_CAUSE_COMMANDED_COLLECTIVE << ADRC_STATE_LIFTOFF_CAUSE_SHIFT),
        adrcStateFlags(&runtime));

    adrcResetGate(&runtime);
    EXPECT_EQ(initialResetCount + 1, runtime.gateResetCount);
    EXPECT_EQ(ADRC_LIFTOFF_CAUSE_NONE, runtime.liftoffCause);

    simulatedThrottle = 0.10f;
    simulatedCommandedThrottle = 0.25f;
    gyro.gyroADCf[FD_ROLL] = 25.0f;
    for (int i = 0; i < 4; i++) {
        adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    }
    ASSERT_TRUE(runtime.liftoff);
    EXPECT_EQ(ADRC_LIFTOFF_CAUSE_GYRO, runtime.liftoffCause);

    adrcResetGate(&runtime);
    resetGyro();
    profile.liftoffThrottlePercent = 30;
    simulatedThrottle = 0.32f;
    simulatedCommandedThrottle = 0.20f;
    for (int i = 0; i < 40; i++) {
        adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    }
    ASSERT_TRUE(runtime.liftoff);
    EXPECT_EQ(ADRC_LIFTOFF_CAUSE_APPLIED_COLLECTIVE, runtime.liftoffCause);
    EXPECT_EQ(ADRC_STATE_LIFTOFF
            | (ADRC_LIFTOFF_CAUSE_APPLIED_COLLECTIVE << ADRC_STATE_LIFTOFF_CAUSE_SHIFT),
        adrcStateFlags(&runtime));
}

TEST_F(AdrcUnittest, InitConfigCapsCorruptUpperRangeValues)
{
    constexpr float dT = 0.000125f; // 8 kHz
    profile.wc[FD_ROLL] = UINT16_MAX;
    profile.wo[FD_ROLL] = UINT16_MAX;
    profile.sigmaDecay = UINT8_MAX;
    profile.gatedZ3DecayRate = UINT16_MAX;
    profile.tdHz = UINT16_MAX;
    profile.gyroFilterHz = UINT16_MAX;
    adrcInitConfig(&profile, &runtime, dT);

    EXPECT_FLOAT_EQ(300.0f, runtime.coefficient[FD_ROLL].wc);
    EXPECT_FLOAT_EQ(600.0f, runtime.coefficient[FD_ROLL].wo);
    EXPECT_FLOAT_EQ(10.0f, runtime.coefficient[FD_ROLL].decayRate);
    EXPECT_FLOAT_EQ(200.0f, runtime.coefficient[FD_ROLL].gatedDecayRate);
    EXPECT_FLOAT_EQ(pt1FilterGain(LPF_MAX_HZ, dT), runtime.coefficient[FD_ROLL].tdFilterGain);
    EXPECT_FLOAT_EQ(pt2FilterGain(LPF_MAX_HZ, dT), runtime.gyroFilter[FD_ROLL].k);
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
    // behind, unlike the disabled (direct passthrough) case above. The PT1 discretization remains
    // monotonic even at this deliberately slow 125 Hz-equivalent test looptime.
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

TEST_F(AdrcUnittest, TdSweepIsFiniteAndMonotonicAtSupportedLoopRates)
{
    const int loopRatesHz[] = { 200, 1600, 4000, 8000 };

    profile.gyroFilterHz = 0;
    for (const int loopRateHz : loopRatesHz) {
        const float dT = 1.0f / loopRateHz;
        for (int cutoffHz = 1; cutoffHz <= LPF_MAX_HZ; cutoffHz++) {
            SCOPED_TRACE(::testing::Message() << "loopRateHz=" << loopRateHz << ", cutoffHz=" << cutoffHz);
            profile.tdHz = cutoffHz;
            adrcInitConfig(&profile, &runtime, dT);
            gyro.gyroADCf[FD_ROLL] = 0.0f;
            adrcResetState(&runtime, FD_ROLL);

            for (int i = 0; i < 100; i++) {
                const adrcOutput_t out = adrcApplyControl(&runtime, FD_ROLL, 0.0f, 500.0f, dT, 500.0f);
                ASSERT_TRUE(isfinite(runtime.vRef[FD_ROLL]));
                ASSERT_TRUE(isfinite(out.P));
                ASSERT_TRUE(isfinite(out.I));
                ASSERT_TRUE(isfinite(out.D));
                ASSERT_GE(runtime.vRef[FD_ROLL], 0.0f);
                ASSERT_LE(runtime.vRef[FD_ROLL], 500.0f);
            }
        }
    }
}

TEST_F(AdrcUnittest, RepeatedResetSeedsTdReferenceBumplessly)
{
    constexpr float dT = 0.000125f; // 8 kHz
    profile.tdHz = 5;
    profile.gyroFilterHz = 0;
    adrcInitConfig(&profile, &runtime, dT);
    gyro.gyroADCf[FD_ROLL] = 500.0f;

    // pidResetIterm() can repeat every loop during a 3D reversal. The reset reference must start
    // from the measured rate, so a positive setpoint error cannot manufacture a negative P pulse.
    for (int i = 0; i < 100; i++) {
        adrcResetState(&runtime, FD_ROLL);
        const adrcOutput_t out = adrcApplyControl(&runtime, FD_ROLL, 500.0f, 1000.0f, dT, 500.0f);
        ASSERT_TRUE(isfinite(out.P));
        EXPECT_GE(out.P, 0.0f);
        EXPECT_GE(runtime.vRef[FD_ROLL], 500.0f);
        EXPECT_LE(runtime.vRef[FD_ROLL], 1000.0f);
    }
}

TEST_F(AdrcUnittest, EsoSweepConvergesAtSupportedLoopRates)
{
    const int loopRatesHz[] = { 200, 1600, 4000, 8000 };

    profile.gyroFilterHz = 0;
    // Exercise the maximum permitted decay on both paths; zero-decay behavior is covered by
    // Z3IsPureIntegratorWhenDecayDisabled and is less restrictive for Euler stability.
    profile.sigmaDecay = 100;
    profile.gatedZ3DecayRate = 2000;
    for (const int loopRateHz : loopRatesHz) {
        const float dT = 1.0f / loopRateHz;
        for (int wo = 10; wo <= 600; wo++) {
            for (int liftoff = 0; liftoff <= 1; liftoff++) {
                SCOPED_TRACE(::testing::Message() << "loopRateHz=" << loopRateHz << ", wo=" << wo
                    << ", liftoff=" << liftoff);
                profile.wo[FD_ROLL] = wo;
                adrcInitConfig(&profile, &runtime, dT);
                gyro.gyroADCf[FD_ROLL] = 0.0f;
                adrcResetState(&runtime, FD_ROLL);
                runtime.liftoff = liftoff;

                for (int i = 0; i < loopRateHz / 2; i++) {
                    const adrcOutput_t out = adrcApplyControl(&runtime, FD_ROLL, 1000.0f, 1000.0f, dT, 500.0f);
                    ASSERT_TRUE(isfinite(runtime.z1[FD_ROLL]));
                    ASSERT_TRUE(isfinite(runtime.z2[FD_ROLL]));
                    ASSERT_TRUE(isfinite(runtime.z3[FD_ROLL]));
                    ASSERT_TRUE(isfinite(out.P));
                    ASSERT_TRUE(isfinite(out.I));
                    ASSERT_TRUE(isfinite(out.D));
                }

                EXPECT_NEAR(1000.0f, runtime.z1[FD_ROLL], 500.0f);
            }
        }
    }
}

TEST_F(AdrcUnittest, HighFsrGyroDoesNotHitLegacyTwoKdpsStateLimit)
{
    constexpr float dT = 0.000125f; // 8 kHz
    profile.gyroFilterHz = 0;
    adrcInitConfig(&profile, &runtime, dT);
    gyro.gyroADCf[FD_ROLL] = 3900.0f;
    adrcResetState(&runtime, FD_ROLL);

    const adrcOutput_t out = adrcApplyControl(&runtime, FD_ROLL, 3900.0f, 3900.0f, dT, 500.0f);
    EXPECT_GT(runtime.z1[FD_ROLL], 3500.0f);
    EXPECT_TRUE(isfinite(out.P));
    EXPECT_TRUE(isfinite(out.I));
    EXPECT_TRUE(isfinite(out.D));
}

TEST_F(AdrcUnittest, CorruptZeroB0ScaleMaxIsSanitized)
{
    profile.hoverThrottlePercent = 5;
    profile.b0ThrottleScaleMax = 0; // impossible through CLI, possible in stale/corrupt PG data
    simulatedThrottle = 1.0f;
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);

    EXPECT_GE(runtime.b0ThrottleScale, 1.0f);
    const adrcOutput_t out = adrcApplyControl(&runtime, FD_ROLL, 0.0f, 500.0f, TEST_DT, 500.0f);
    EXPECT_TRUE(isfinite(out.P));
    EXPECT_TRUE(isfinite(out.I));
    EXPECT_TRUE(isfinite(out.D));
}

TEST_F(AdrcUnittest, NonFiniteRuntimeStateRecoversToFiniteOutput)
{
    runtime.z1[FD_ROLL] = NAN;
    runtime.z2[FD_ROLL] = INFINITY;
    runtime.z3[FD_ROLL] = -INFINITY;
    runtime.vRef[FD_ROLL] = NAN;
    runtime.lastOutput[FD_ROLL] = INFINITY;
    runtime.gyroFilter[FD_ROLL].state = NAN;
    runtime.b0ThrottleScale = NAN;

    const adrcOutput_t out = adrcApplyControl(&runtime, FD_ROLL, 100.0f, 200.0f, TEST_DT, 500.0f);
    EXPECT_FLOAT_EQ(100.0f, runtime.z1[FD_ROLL]);
    EXPECT_FLOAT_EQ(0.0f, runtime.z2[FD_ROLL]);
    EXPECT_FLOAT_EQ(0.0f, runtime.z3[FD_ROLL]);
    EXPECT_FLOAT_EQ(0.0f, runtime.lastOutput[FD_ROLL]);
    EXPECT_FLOAT_EQ(1.0f, runtime.b0ThrottleScale);
    EXPECT_TRUE(isfinite(runtime.z1[FD_ROLL]));
    EXPECT_TRUE(isfinite(runtime.z2[FD_ROLL]));
    EXPECT_TRUE(isfinite(runtime.z3[FD_ROLL]));
    EXPECT_TRUE(isfinite(runtime.vRef[FD_ROLL]));
    EXPECT_TRUE(isfinite(runtime.lastOutput[FD_ROLL]));
    EXPECT_TRUE(isfinite(runtime.b0ThrottleScale));
    EXPECT_TRUE(isfinite(out.P));
    EXPECT_TRUE(isfinite(out.I));
    EXPECT_TRUE(isfinite(out.D));

    const adrcOutput_t invalidInputOut = adrcApplyControl(&runtime, FD_ROLL, INFINITY, NAN, NAN, NAN);
    EXPECT_TRUE(isfinite(runtime.z1[FD_ROLL]));
    EXPECT_TRUE(isfinite(runtime.z2[FD_ROLL]));
    EXPECT_TRUE(isfinite(runtime.z3[FD_ROLL]));
    EXPECT_TRUE(isfinite(invalidInputOut.P));
    EXPECT_TRUE(isfinite(invalidInputOut.I));
    EXPECT_TRUE(isfinite(invalidInputOut.D));
}

// ADRC-030 (experimental): a lower wc while the liftoff gate is closed, ramping to the flight wc
// once the gate opens. Off by default, in which case the control law must be bit-identical.
TEST_F(AdrcUnittest, GroundWcDisabledByDefaultLeavesGainsUntouched)
{
    ASSERT_FALSE(runtime.liftoff);
    EXPECT_FLOAT_EQ(runtime.coefficient[FD_ROLL].wc, runtime.coefficient[FD_ROLL].groundWc);
    // vRef == setpoint (TD off), z1 == 0: P = wc^2 * setpoint / b0 = 60^2 * 100 / 2000 = 180.
    const adrcOutput_t out = adrcApplyControl(&runtime, FD_ROLL, 0.0f, 100.0f, TEST_DT, 500.0f);
    EXPECT_FLOAT_EQ(180.0f, out.P);
}

TEST_F(AdrcUnittest, GroundWcAppliesWhileGateClosedAndRampsAfterLiftoff)
{
    adrcInitConfig(&profile, &runtime, TEST_DT);
    adrcSetGroundWc(&runtime, 40, 200, 0);
    adrcResetGate(&runtime);

    // Gate closed: wc = 40 -> P = 40^2 * 100 / 2000 = 80.
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    ASSERT_FALSE(runtime.liftoff);
    EXPECT_FLOAT_EQ(80.0f, adrcApplyControl(&runtime, FD_ROLL, 0.0f, 100.0f, TEST_DT, 500.0f).P);

    // Gate opens: 12 loops of 8 ms = 96 ms into a 200 ms ramp -> blend 0.48 -> wc 49.6.
    runtime.liftoff = true;
    for (int i = 0; i < 12; i++) {
        adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    }
    EXPECT_NEAR(49.6f * 49.6f * 100.0f / 2000.0f,
        adrcApplyControl(&runtime, FD_ROLL, 0.0f, 100.0f, TEST_DT, 500.0f).P, 0.5f);

    // Ramp complete: wc = 60 -> P = 180, and it stays there.
    for (int i = 0; i < 40; i++) {
        adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    }
    EXPECT_FLOAT_EQ(1.0f, runtime.wcBlend);
    EXPECT_FLOAT_EQ(180.0f, adrcApplyControl(&runtime, FD_ROLL, 0.0f, 100.0f, TEST_DT, 500.0f).P);

    // Gate reset (disarm-arm) drops straight back to the ground wc.
    adrcResetGate(&runtime);
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    EXPECT_FLOAT_EQ(80.0f, adrcApplyControl(&runtime, FD_ROLL, 0.0f, 100.0f, TEST_DT, 500.0f).P);
}

TEST_F(AdrcUnittest, GroundWcIsCappedAtFlightWcAndRampZeroSwitches)
{
    adrcInitConfig(&profile, &runtime, TEST_DT);
    adrcSetGroundWc(&runtime, 250, 0, 0); // above wc 60: must not raise the ground gain
    adrcResetGate(&runtime);
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    EXPECT_FLOAT_EQ(180.0f, adrcApplyControl(&runtime, FD_ROLL, 0.0f, 100.0f, TEST_DT, 500.0f).P);

    adrcInitConfig(&profile, &runtime, TEST_DT);
    adrcSetGroundWc(&runtime, 40, 0, 0);
    adrcResetGate(&runtime);
    runtime.liftoff = true;
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    EXPECT_FLOAT_EQ(1.0f, runtime.wcBlend);
    EXPECT_FLOAT_EQ(180.0f, adrcApplyControl(&runtime, FD_ROLL, 0.0f, 100.0f, TEST_DT, 500.0f).P);
}

TEST_F(AdrcUnittest, SameTypeReinitPreservesActiveGroundWcRamp)
{
    // An AUX adjustment re-runs pidInitConfig() -> adrcInitConfig() + adrcSetGroundWc() while armed;
    // that must not finish (or restart) a ramp that is in progress.
    adrcInitConfig(&profile, &runtime, TEST_DT);
    adrcSetGroundWc(&runtime, 40, 200, 0);
    adrcResetGate(&runtime);
    runtime.liftoff = true;
    for (int i = 0; i < 12; i++) {
        adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    }
    const float blendBefore = runtime.wcBlend;
    ASSERT_NEAR(0.48f, blendBefore, 0.001f);
    adrcInitConfig(&profile, &runtime, TEST_DT);
    adrcSetGroundWc(&runtime, 40, 200, 0);
    EXPECT_FLOAT_EQ(blendBefore, runtime.wcBlend);
    EXPECT_NEAR(49.6f * 49.6f * 100.0f / 2000.0f,
        adrcApplyControl(&runtime, FD_ROLL, 0.0f, 100.0f, TEST_DT, 500.0f).P, 0.5f);
}

TEST_F(AdrcUnittest, GroundDgainCapsGroundWcPerAxis)
{
    // dgain 1.0 -> cap = b0 / (2 wo) per axis, using the runtime wo (capped against TEST_DT) and b0;
    // with the defaults (b0 2000, wo <= 62.5 at 8 ms) that is 16, well below the requested 40.
    adrcInitConfig(&profile, &runtime, TEST_DT);
    adrcSetGroundWc(&runtime, 40, 0, 10);
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        const adrcCoefficient_t &c = runtime.coefficient[axis];
        EXPECT_NEAR(c.b0 / (2.0f * c.wo), c.groundWc, 1e-3f);
        EXPECT_LT(c.groundWc, 40.0f);
    }
    adrcResetGate(&runtime);
    adrcUpdatePerLoopState(&runtime, &profile, TEST_DT);
    // P = groundWc^2 * 100 / b0 on the ground.
    const adrcCoefficient_t &cr = runtime.coefficient[FD_ROLL];
    EXPECT_NEAR(cr.groundWc * cr.groundWc * 100.0f / cr.b0,
        adrcApplyControl(&runtime, FD_ROLL, 0.0f, 100.0f, TEST_DT, 500.0f).P, 1e-2f);

    // A large dgain leaves the plain ground wc in charge; dgain without ground wc does nothing.
    adrcSetGroundWc(&runtime, 40, 0, 100);
    EXPECT_FLOAT_EQ(40.0f, runtime.coefficient[FD_ROLL].groundWc);
    adrcSetGroundWc(&runtime, 0, 0, 10);
    EXPECT_FLOAT_EQ(60.0f, runtime.coefficient[FD_ROLL].groundWc);
}

// ADRC-031: adrc_b0_scale_min lets the schedule go below 1 under hover; 100 (default) keeps the
// b10.1 "scale only up" behaviour exactly, FIXED is unaffected, and the floor is 20 %.
TEST_F(AdrcUnittest, B0ScaleMinDefaultKeepsScaleOnlyUp)
{
    profile.hoverThrottlePercent = 35;
    profile.b0Law = ADRC_B0_LAW_SQRT;
    adrcInitConfig(&profile, &runtime, TEST_DT);
    adrcSetB0ScaleMin(&runtime, 100);
    simulatedThrottle = 0.10f;
    settleB0ThrottleScale();
    EXPECT_FLOAT_EQ(1.0f, runtime.b0ThrottleScale);
}

TEST_F(AdrcUnittest, B0ScaleMinLetsScheduleGoBelowHoverDownToTheFloor)
{
    profile.hoverThrottlePercent = 35;
    adrcInitConfig(&profile, &runtime, TEST_DT);
    adrcSetB0ScaleMin(&runtime, 50);
    runtime.liftoff = true; // the floor only acts airborne (see the gate test below)
    simulatedThrottle = 0.10f; // ratio 0.2857

    profile.b0Law = ADRC_B0_LAW_SQRT; // sqrt(0.2857) = 0.535 > 0.5 -> unclamped
    settleB0ThrottleScale();
    EXPECT_NEAR(0.5345f, runtime.b0ThrottleScale, 2e-3f);

    profile.b0Law = ADRC_B0_LAW_QUADRATIC; // 0.0816 -> clamped to 0.5
    settleB0ThrottleScale();
    EXPECT_NEAR(0.5f, runtime.b0ThrottleScale, 1e-4f);

    profile.b0Law = ADRC_B0_LAW_FIXED;
    settleB0ThrottleScale();
    EXPECT_FLOAT_EQ(1.0f, runtime.b0ThrottleScale);

    // Above hover nothing changes: LINEAR at ratio 2 -> 2.
    profile.b0Law = ADRC_B0_LAW_LINEAR;
    simulatedThrottle = 0.70f;
    settleB0ThrottleScale();
    EXPECT_NEAR(2.0f, runtime.b0ThrottleScale, 1e-3f);

    // The control law sees the scheduled b0: at scale 0.5 the P output doubles (kp * 100 / (b0 * 0.5)).
    profile.b0Law = ADRC_B0_LAW_QUADRATIC;
    simulatedThrottle = 0.10f;
    settleB0ThrottleScale();
    adrcResetGate(&runtime);
    const float pFull = runtime.coefficient[FD_ROLL].kp * 100.0f / runtime.coefficient[FD_ROLL].b0;
    EXPECT_NEAR(2.0f * pFull, adrcApplyControl(&runtime, FD_ROLL, 0.0f, 100.0f, TEST_DT, 500.0f).P, 1e-2f);

    // Floor: 5 % requested -> 20 % (airborne again after the gate reset above).
    adrcSetB0ScaleMin(&runtime, 5);
    runtime.liftoff = true;
    settleB0ThrottleScale();
    EXPECT_NEAR(0.2f, runtime.b0ThrottleScale, 1e-4f);
}

TEST_F(AdrcUnittest, B0ScaleMinDoesNotApplyWhileGateIsClosed)
{
    // On the ground the stick is below hover, so the floor would divide the P/D b0 by up to 5 and
    // multiply the closed-gate loop gain; the schedule must stay >= 1 until liftoff.
    profile.hoverThrottlePercent = 35;
    profile.b0Law = ADRC_B0_LAW_QUADRATIC;
    adrcInitConfig(&profile, &runtime, TEST_DT);
    adrcSetB0ScaleMin(&runtime, 20);
    adrcResetGate(&runtime);
    simulatedThrottle = 0.10f;
    settleB0ThrottleScale();
    ASSERT_FALSE(runtime.liftoff);
    EXPECT_FLOAT_EQ(1.0f, runtime.b0ThrottleScale);
    runtime.liftoff = true;
    settleB0ThrottleScale();
    EXPECT_NEAR(0.2f, runtime.b0ThrottleScale, 1e-4f);
    adrcResetGate(&runtime);
    settleB0ThrottleScale();
    EXPECT_FLOAT_EQ(1.0f, runtime.b0ThrottleScale);
}

TEST_F(AdrcUnittest, B0ScaleMinBoundaryValueIsNotTreatedAsOff)
{
    // b11-exp5 regression: adrc_b0_scale_min = 20 (the CLI minimum) behaved as 100 on the target
    // because 20 * 0.01f rounds below 0.2f and the old "< floor -> reset to 1" sanity check fired
    // under -ffast-math. The floor must engage for 20 exactly like for 21, and a runtime value just
    // below the floor must be clamped up, not switched off.
    profile.hoverThrottlePercent = 38;
    profile.b0Law = ADRC_B0_LAW_SQRT;
    adrcInitConfig(&profile, &runtime, TEST_DT);
    for (uint8_t pct : {20, 21}) {
        adrcSetB0ScaleMin(&runtime, pct);
        EXPECT_GE(runtime.b0ScaleMin, 0.2f);
        EXPECT_LE(runtime.b0ScaleMin, 0.21f);
        adrcResetGate(&runtime);
        runtime.liftoff = true;
        simulatedThrottle = 0.02f; // idle collective in flight -> raw sqrt(0.02/0.38) = 0.23
        settleB0ThrottleScale();
        adrcApplyControl(&runtime, FD_ROLL, 0.0f, 0.0f, TEST_DT, 500.0f);
        EXPECT_NEAR(0.229f, runtime.b0ThrottleScale, 0.01f) << "pct " << int(pct);
        EXPECT_GE(runtime.b0ScaleMin, 0.2f);
    }
    runtime.b0ScaleMin = 0.19999999f; // what the target computed for 20
    adrcApplyControl(&runtime, FD_ROLL, 0.0f, 0.0f, TEST_DT, 500.0f);
    EXPECT_FLOAT_EQ(0.2f, runtime.b0ScaleMin);
    runtime.b0ScaleMin = NAN;
    adrcApplyControl(&runtime, FD_ROLL, 0.0f, 0.0f, TEST_DT, 500.0f);
    EXPECT_FLOAT_EQ(1.0f, runtime.b0ScaleMin);
}

// ADRC-032: per-axis damping ratio. 100 % leaves the law untouched; 50 % halves D; 0 % removes D; the ground-wc
// path scales with it too.
TEST_F(AdrcUnittest, SatZ3InhibitStopsZ3GrowthOnlyWhileMixerSaturatedAndEnabled)
{
    adrcInitConfig(&profile, &runtime, TEST_DT);
    adrcResetGate(&runtime);
    runtime.liftoff = true;
    // Observer error that drives z3 outward: z1 held at 0, gyro at 200 deg/s, no command.
    auto step = [&]() { runtime.z1[FD_ROLL] = 0.0f; runtime.z2[FD_ROLL] = 0.0f; runtime.z3GrowthInhibitMask = 0;
        adrcApplyControl(&runtime, FD_ROLL, 200.0f, 0.0f, TEST_DT, 500.0f); };

    // Default (flag off): a saturated mixer does not stop z3 from charging.
    adrcSetMixerSaturated(&runtime, true);
    runtime.z3[FD_ROLL] = 0.0f;
    step();
    EXPECT_GT(fabsf(runtime.z3[FD_ROLL]), 0.0f);
    EXPECT_EQ(0u, runtime.z3GrowthInhibitMask);

    // Flag on + saturated: growth suppressed, mask flags it, decay toward zero still runs.
    adrcSetSatZ3Inhibit(&runtime, true);
    runtime.z3[FD_ROLL] = 0.0f;
    step();
    EXPECT_FLOAT_EQ(0.0f, runtime.z3[FD_ROLL]);
    EXPECT_EQ(1u << FD_ROLL, runtime.z3GrowthInhibitMask);
    runtime.z3[FD_ROLL] = 1000.0f; // same sign as the growth direction would push
    const float before = runtime.z3[FD_ROLL];
    step();
    EXPECT_LE(fabsf(runtime.z3[FD_ROLL]), before); // never grows while inhibited

    // Flag on, mixer freed: charging resumes on the next iteration.
    adrcSetMixerSaturated(&runtime, false);
    runtime.z3[FD_ROLL] = 0.0f;
    step();
    EXPECT_GT(fabsf(runtime.z3[FD_ROLL]), 0.0f);
    EXPECT_EQ(0u, runtime.z3GrowthInhibitMask);
}

