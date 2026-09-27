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

#include <math.h>

#include "platform.h"

#ifdef USE_ADRC

#include "common/axis.h"
#include "common/maths.h"

#include "build/debug.h"

#include "sensors/gyro.h"

#include "flight/mixer.h"

#include "adrc.h"

// Generous physical bounds on the ESO's angular-rate estimate z1 [deg/s] and angular-acceleration
// estimate z2 [deg/s^2]. z1 leaves 2x headroom over Betaflight's supported +/-4000 dps gyro FSR;
// a hard 5" snap peaks at roughly 12-25k deg/s^2, so the z2 bound likewise avoids constraining
// normal flight while still preventing unbounded numerical divergence.
#define ADRC_Z1_LIMIT 8000.0f
#define ADRC_Z2_LIMIT 100000.0f

// The forward-Euler ESO has repeated discrete observer poles at 1 - wo*dT before z3 decay is
// included. Keeping wo*dT <= 0.5 leaves the poles non-oscillatory with margin for the independently
// configured decay term. The default wo remains unchanged at every supported loop rate (the slowest
// supported 200 Hz loop permits wo=100); only impossible high-bandwidth/slow-loop combinations are
// reduced at runtime.
#define ADRC_ESO_MAX_WO_DT 0.5f

// Defense-in-depth mirrors the CLI ranges for values that can also arrive through persisted data.
#define ADRC_WC_MIN 5.0f
#define ADRC_WC_MAX 300.0f
#define ADRC_WO_MIN 10.0f
#define ADRC_WO_MAX 600.0f
#define ADRC_B0_MIN 100.0f
#define ADRC_SIGMA_DECAY_MAX 100.0f
#define ADRC_GATED_Z3_DECAY_MAX 2000.0f
#define ADRC_B0_SCALE_MIN_FLOOR_PERCENT 20
#define ADRC_B0_SCALE_MIN_FLOOR 0.2f // ADRC-031: b0 never scheduled below 20 % of the hover value
#define ADRC_B0_SCALE_MAX 50.0f

// Liftoff-gate mechanism (thresholds now live in adrcProfile_t - see adrc.h - ported as tunable
// fields rather than fixed constants; the numbers below are just the danusha2345/ADRC-betaflight
// community-validated defaults, set in adrcResetProfile()). While the craft is ground-constrained
// the plant does not respond to the control output, but the ESO doesn't know that: it misattributes
// the "missing" response to a phantom disturbance and winds z3 up, which then has to unwind
// violently at liftoff. Until liftoff is detected the observer's b0*u feedback term is held at
// zero. Three paths detect it, all in adrcUpdatePerLoopState(): commanded throttle at or above
// liftoffThrottlePercent; any-axis rotation above liftoffGyroDps sustained for liftoffHoldMs; and
// applied collective at or above the same throttle threshold, held for ADRC_LIFTOFF_APPLIED_HOLD_S
// or liftoffHoldMs, whichever is longer. The latter two additionally require the commanded
// throttle to be off idle (ADRC_LIFTOFF_GYRO_THROTTLE_FRACTION of the liftoff threshold), so
// rotation alone does not open the gate - a toss launch thrown at literally zero throttle waits
// for the throttle to come up, which is the ADRC-026 trade-off recorded further down. Detection is
// one-way: nothing in the detector closes the gate once it is open (ADRC-020: an earlier opt-in
// mid-air re-arm heuristic was removed rather than kept - throttle+gyro alone cannot distinguish a
// landing from a calm mid-air float, and adrcUpdateArmTransition() below already covers the
// ground-rep use case it existed for via a fresh epoch on every disarm->arm). That is not the same
// as the gate being permanent: an explicit controller-epoch adrcResetAll() closes it, in flight
// included - see the inhibit comment in adrcApplyControl() for why that stays safe.

// ADRC-026: on the ground under airmode, the gate opened on a craft whose throttle stick had never
// moved, and the ESO then integrated ground-contact dynamics it cannot model until the motors ran
// up to saturation. Which branch let it through is settled by the logs rather than assumed - all
// five wo = 150 arms (`pr15400-b5-wcwo2x2/`) were re-decoded for this fix:
//
//   * The GYRO branch did not fire in any of them. Its hold runs on PID iterations: at these logs'
//     312 us looptime, adrc_liftoff_hold_ms = 25 needs 81 consecutive iterations above
//     adrc_liftoff_gyro_dps. Blackbox saved every second iteration (626 us between samples) and
//     logs the detector's own signal (gyroADC[] = the filtered gyro.gyroADCf); the longest run of
//     saved samples above the threshold before any gate open was 6 (3.8 ms) - at most 13
//     consecutive iterations even crediting every unsaved neighbour, against the 81 required.
//     Consistent with the excitation: a ~28.5 Hz oscillation crosses zero every ~17.5 ms, well
//     inside a 25 ms hold. That is a reading of these logs, not a general theorem - the test takes
//     the max over three axes, so a biased, multi-axis or lower-frequency excitation could still
//     hold it.
//   * The THROTTLE branch explains every open. The collective proxy (mean motor over the logged
//     output range) reached 32.1 / 30.3 / 32.2 % on the opening sample of the three arms that
//     opened, against these logs' adrc_liftoff_throttle = 30, with rcCommand[THROTTLE] at its
//     minimum throughout.
//
// The collective the gate used to read is the mixer's *applied* value, which includes the headroom
// airmode adds so the axis mix fits - thrust nobody commanded. So the gate now reads
// mixerGetAdrcCommandedThrottle(): the same collective sampled before that headroom is added, and
// after the automatic-mode overrides, so ALT_HOLD/GPS_RESCUE still open it with the stick at zero.
// That value is the commanded collective, NOT the stick - throttle angle correction, throttle
// limit/boost, the dyn-idle floor and RPM limiting are all still in it (see mixer.c). The gate's
// question is "was thrust commanded", and those are commands; the distinction that matters here is
// commanded vs. mixer-added, not pilot vs. firmware. All three paths read it - directly for the
// commanded test, and through the idle interlock for the other two, which ask the same question.
// The b0 schedule keeps reading the applied value - it needs the thrust that exists, not the
// thrust that was asked for.
//
// The gyro branch additionally requires a minimum throttle. On this evidence that is defence in
// depth rather than the fix - it closes the toss-launch path against a slower ground excitation
// (below ~20 Hz, or one-sided) that could hold the test, which no log here shows. Its floor is
// derived from liftoffThrottlePercent rather than being its own setting because PG_PID_PROFILE's
// version nibble is 4 bits and already sits at its 15 ceiling, so a new profile field cannot be
// added without wrapping the version to 0 (a real historical value). The fraction must stay
// meaningfully below 1: the gyro branch is an `else if` after the direct throttle check, so a floor
// equal to liftoffThrottlePercent would make the gyro path dead code.
//
// Known trade-off: a toss launch thrown at literally zero throttle no longer opens the gate on
// rotation alone. Once the throttle comes up, the direct commanded test opens it immediately at
// liftoffThrottlePercent; below that the gyro and applied paths can open it, but each has to serve
// its own hold from the moment the throttle leaves idle.
#define ADRC_LIFTOFF_GYRO_THROTTLE_FRACTION 0.5f

// Reading the commanded collective alone (above) closes the ADRC-026 false-open, but it opens the
// opposite failure: thrust the mixer applied without the pilot commanding it can still lift the
// craft, and then nothing opens the gate at all - the observer would fly without its b0*u feedback
// for the rest of the arm cycle. So the applied collective keeps a path of its own, gated on
// duration instead of magnitude: ground oscillation drives the mixer past the threshold only in
// bursts, while a craft that is actually airborne holds it. Measured on the ten 2026-08-06 logs
// (danusha2345/ADRC-betaflight docs/flight-test-analysis): the longest unbroken run above the
// threshold before the pilot first moved the stick was 43.5 ms at a 40% threshold and 68.2 ms at
// 25%, both in the log where the craft never left the ground - against 0.37-16.3 s once flying.
// 250 ms sits an order of magnitude above the ground worst case and still well inside a takeoff.
#define ADRC_LIFTOFF_APPLIED_HOLD_S 0.25f

// Zeroing the b0*u term in z2's update, which is all the liftoff gate above used to do, does
// nothing on its own to stop z3 from winding up while grounded. z3 is a leaky integrator of
// errorEso regardless of that term, and its steady-state gain (beta3/decayRate) is enormous
// (beta3 = wo^3, decayRate is a fraction of 1/s), so even a tiny sustained bias (sensor cal
// residual, filter phase lag) winds it toward its clamp given enough idle time - confirmed on a
// props-off bench test, where sitting armed at idle let yaw z3 wind to ~80% of its clamp before
// any stick input. Two things address that, and they are separate. First, while ungated, use a
// never-slower effective decay (selected from gatedZ3DecayRate, airborne decay, and the 1/s floor)
// so an already non-zero z3 relaxes toward zero instead of holding; it still updates smoothly (no
// reset discontinuity). Second - and this is what actually forbids new growth - the gate state
// feeds the magnitude inhibit described next.
//
// The effective decay bounds where z3 settles, but not how fast it gets there: under the sustained
// observer error of a ground oscillation, beta3 = wo^3 outruns it (at wo = 150 the decay time
// constant is ~50 ms against a per-loop beta3 term three orders larger). While the gate is shut,
// therefore, additionally refuse any update that would grow |z3| - the decay half of the update
// still applies, so z3 relaxes but cannot accumulate. This is the damage-bounding half of the
// ADRC-026 pair: keeping the gate shut removes the runaway's trigger, this bounds the charge z3
// can carry into any open that still happens.
//
// The condition is the gate alone, never the stick. It used to be gate AND idle throttle, which
// left a blind window between the idle floor and the gate opening; see the inhibit itself in
// adrcApplyControl() for the measurements that closed it. Keying on the gate does not reintroduce
// ADRC-020 (suppressing z3 growth at low throttle while airborne, when a genuine zero-throttle
// float is flying and its estimate must stay live), because the gate does not close on a float -
// only an explicit controller-epoch reset closes it, and that zeroes the estimate anyway.

// Throttle-scaled b0 (fix #10a): motor authority scales with RPM, and thrust ~ RPM^2 ~ throttle^2,
// so a b0 tuned at hover is wrong away from hover. Scaled only UP from hover, clamped to
// adrcProfile->b0ThrottleScaleMax - scaling down would make 1/b0 huge and inject extreme output at
// low throttle.
#define ADRC_HOVER_THROTTLE_MIN_FRACTION 0.05f

// The b0 schedule reads a low-passed collective rather than the raw per-loop value, for two
// flight-measured reasons (A/B on identical tune, 2026-07-12): (1) the published collective
// includes the mixer's per-loop constrain, which tracks the loop's own axis activity (airmode
// raises collective to make room for the mix) - fed raw, it modulated the effective gain by tens
// of percent right at the ~25 Hz loop resonance (debug d7 swinging 1.0..2.8 at a steady stick);
// (2) a throttle chop collapsed the scale 3->1 within ~80 ms, faster than the ESO re-adapts, so
// the z3 that had wound up through the inflated b0 over-applied at scale 1 and swung the craft
// against the punch (~90 deg/s uncommanded pitch, the "nose dip rebound"). An ~80 ms time
// constant smooths both paths without delaying punch scaling meaningfully.
#define ADRC_B0_SCALE_THROTTLE_LPF_HZ 2.0f

// z3 is the lumped rate-plant disturbance [deg/s^3], so its numeric range is much larger than z1
// [deg/s] or z2 [deg/s^2]. Scale it down to fit the int16 blackbox debug field (fix #12).
//
// ADRC-029: a fixed divisor of 16 clips at |z3| = 524272, which flown high-b0 tunes exceed in
// ordinary flight (0.05-4.4 % of saved frames on an Air65 at b0 4312-7007), while the controller's
// own anti-windup bound sits at pidSumLimit * b0 * b0ThrottleScale - an order of magnitude higher.
// The divisor is therefore derived per profile in adrcZ3LogScale() - the smallest integer whose
// int16 endpoint covers that bound in every float32 evaluation the runtime can produce - and is
// written into the blackbox header as adrc_z3_log_scale so
// every log carries its own decode key. ADRC_Z3_LOG_SCALE remains as the floor and as the value
// implied for logs whose header lacks the field (b9 and earlier).
#define ADRC_Z3_LOG_SCALE 16.0f
#define ADRC_DEBUG_LIMIT 32767.0f

// Firmware is built with -ffast-math, under which the compiler may assume ordinary isfinite()
// checks are always true. Inspect the IEEE-754 exponent directly so the recovery path remains real
// in release builds as well as unit tests.
static bool adrcIsFinite(float value)
{
    union {
        float f;
        uint32_t u;
    } bits = { .f = value };

    return (bits.u & 0x7F800000U) != 0x7F800000U;
}

static void adrcResetAxisState(adrcRuntime_t *adrcRuntime, int axis, float gyroRate)
{
    const float finiteGyroRate = adrcIsFinite(gyroRate) ? gyroRate : 0.0f;
    pt2Filter_t *gyroFilter = &adrcRuntime->gyroFilter[axis];

    if (!adrcIsFinite(gyroFilter->k) || gyroFilter->k < 0.0f || gyroFilter->k > 1.0f) {
        gyroFilter->k = 1.0f;
    }
    gyroFilter->state = finiteGyroRate;
    gyroFilter->state1 = finiteGyroRate;
    adrcRuntime->z1[axis] = finiteGyroRate;
    adrcRuntime->z2[axis] = 0.0f;
    adrcRuntime->z3[axis] = 0.0f;
    adrcRuntime->vRef[axis] = finiteGyroRate;
    adrcRuntime->lastOutput[axis] = 0.0f;
}

static bool adrcAxisStateIsFinite(const adrcRuntime_t *adrcRuntime, int axis)
{
    const pt2Filter_t *gyroFilter = &adrcRuntime->gyroFilter[axis];

    return adrcIsFinite(gyroFilter->k) && gyroFilter->k >= 0.0f && gyroFilter->k <= 1.0f
        && adrcIsFinite(gyroFilter->state) && adrcIsFinite(gyroFilter->state1)
        && adrcIsFinite(adrcRuntime->z1[axis]) && adrcIsFinite(adrcRuntime->z2[axis])
        && adrcIsFinite(adrcRuntime->z3[axis]) && adrcIsFinite(adrcRuntime->vRef[axis])
        && adrcIsFinite(adrcRuntime->lastOutput[axis]);
}

void adrcResetProfile(adrcProfile_t *adrcProfile)
{
    // wc=60/wo=100/b0=2000 traces to danusha2345/ADRC-betaflight's round-2 real-hardware control-
    // bandwidth sweep on a 5" (ADRC_FIXES.md, "round-2 5\" flight logs"): flown with debug_mode=ADRC
    // and blackbox-analyzed across several wc values, converging on wc=60 as the sweet spot, with
    // the practical ceiling set by gyro noise amplification (kp=wc^2), not control-loop stability.
    // Validated as a package alongside the liftoff gate, throttle-scaled b0 and z3 decay below.
    // Yaw keeps a lower wo (80 vs 100), matching the fork's own roll/pitch-vs-yaw split.
    //
    // The fork's own shorthand for this tune is "60/100/200" - but that "200" is the raw D-field
    // value entered in the Configurator, which their port multiplies by a separate adrc_b0_scale
    // CLI setting (default 10, so actual b0 = 200*10 = 2000) since they repurpose the legacy uint8
    // D field for b0 and need the multiplier to reach past its 255 ceiling. Our adrc_b0 field is
    // already a full uint16 with no such ceiling (see ADRC_FIXES.md fix #9, deliberately not
    // ported), so this default must be the final, already-scaled value: 2000, not 200. The fork's
    // own docs confirm scale=10 was standard across all their 5" testing including this sweep
    // ("a 5\" usually reaches its b0 at the default scale = 10").
    //
    // Still airframe-dependent - b0 in particular is an estimate of this specific craft's motor/
    // prop/weight response and will need re-tuning per airframe.
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        adrcProfile->wc[axis] = 60;
        adrcProfile->wo[axis] = (axis == FD_YAW) ? 80 : 100;
        adrcProfile->b0[axis] = 2000;
    }
    // Classic PID applies a dedicated, separate filter stage (dterm_lpf1/lpf2) specifically to
    // whatever feeds its noise-sensitive D-term, on top of the base gyro filter shared with P/I.
    // ADRC's entire control law - P, I, and D-equivalent together - is derived from a single ESO
    // with no equivalent protection, and kp=wc^2 makes it considerably more sensitive to whatever
    // noise gets through than classic's linear D-gain is. This default is a placeholder in the same
    // spirit as dterm_lpf2's default (150Hz) - untuned for any specific airframe. Not part of the
    // ported source - this project's own addition on top.
    adrcProfile->gyroFilterHz = 150;
    // Throttle % at hover; b0 is scaled by (throttle/hover)^2 above hover (fix #10a).
    adrcProfile->hoverThrottlePercent = 35;
    // z3 leaky-decay rate x0.1 (fix #11): a mild leak (tau ~ 3s) so a transient disturbance bump
    // bleeds off instead of lingering. Set 0 for the classic pure integrator.
    adrcProfile->sigmaDecay = 3;
    // Tracking differentiator, off by default: smooths the setpoint feeding the control law (not
    // the ESO's own gyro-tracking error) instead of feeding it straight through. Ported from
    // SeverinBitterli's independent implementation, not danusha2345's - unvalidated here, left
    // opt-in for testers.
    adrcProfile->tdHz = 0;

    // Liftoff-gate defaults, ported as-is from danusha2345/ADRC-betaflight (ADRC_FIXES.md fix
    // #8/#10) - community-validated on real hardware across several testers/airframes. See adrc.h
    // for what each one does; liftoffThrottlePercent in particular has no built-in relationship to
    // hoverThrottlePercent above - set it a bit above your actual hover throttle, not equal to it.
    adrcProfile->liftoffThrottlePercent = 40;
    adrcProfile->liftoffGyroDps = 20;
    adrcProfile->liftoffHoldMs = 25;
    // No mid-air re-arm: the detector opens the gate once at first liftoff and never closes it
    // itself; disarm is the only ground signal that cannot false-trigger mid-flight (an explicit
    // adrcResetAll() can still close the gate while armed - ADRC-020). An earlier
    // opt-in re-arm heuristic (idle throttle + stillness sustained for a hold) was removed rather
    // than kept: in the first freestyle log flown on this branch (btfl_002-ACRO.bbl, SpeedyBee F7
    // Mini, acro - airmode feature not even enabled) it closed the gate mid-air three times, on
    // ballistic floats reading 1-4 deg/s for over 500 ms with |acc| at 0.1-0.5 g (demonstrably
    // airborne, near free-fall), each time dumping the live z3 estimate (carrying up to ~|100k|)
    // through the fast gated decay and blinding the observer to b0*u. Under airmode the mixer
    // keeps applying u through exactly such floats, so the corruption there is expected to be
    // worse still. Adding a sustained |acc| ~ 1g condition would separate these floats from a
    // genuine landing, but adrcUpdateArmTransition()'s fresh-epoch-per-arm-cycle fix already
    // covers the ground-rep use case the heuristic existed for, so there is no validated use case
    // left to justify keeping the extra params/risk surface.
    // z3 decay rate x0.1 while ungated (grounded) - never slower than sigmaDecay above, so a
    // non-zero z3 relaxes toward zero while the gate is shut regardless of the configured airborne
    // decay. Growth itself is refused by the gate-only inhibit in adrcApplyControl(), not by this.
    adrcProfile->gatedZ3DecayRate = 200;
    // Ceiling on the throttle-scaled b0 multiplier (fix #10a). 3, not the fork's hardcoded 9: the
    // quadratic (throttle/hover)^2 law was only ever community-validated up to ~x3 (hover = 35%
    // craft, scale 9 needs 105% throttle), and the first freestyle log on this branch shows what
    // the extrapolated region does. At hover = 22% a 59%-throttle punch hit scale 7.4, and the
    // failure is two-phase: DURING the punch, an uncommanded 148 deg/s pitch excursion with
    // motors far from saturation (1411/2047) - output per unit error cut 7.4x vs the hover
    // calibration (net under-gain vs the true plant more like x2.5-3, since real gain does grow
    // with throttle) - while z3 wound past the blackbox debug clip (>= 524k) absorbing the
    // unrejected error; then at the throttle chop the scale collapsed 7.4 -> 1 within ~80 ms and
    // the still-wound z3 over-applied, swinging the craft +180 deg/s the opposite way. The
    // schedule yanks the gain out from under the ESO's adaptation faster than z3 re-converges,
    // so the scale transient itself pumps the loop at every throttle pump. Classic TPA's
    // full-throttle authority cut (~35-50%) independently implies true plant-gain growth from
    // hover to full of ~x2-3, not x8 - cap where validation ends. CLI-tunable for experiments.
    adrcProfile->b0ThrottleScaleMax = 3;
    // ADRC-021 A/B selector (see adrcB0Law_e). Quadratic = the law shipped through b10.1; SQRT is the default since b11
    // so a profile reset flies exactly like b4; set sqrt/linear/fixed per PID profile to compare.
    adrcProfile->b0Law = ADRC_B0_LAW_SQRT; // b11 default (D2); QUADRATIC was worst on throttle steps
}

void adrcInitConfig(const adrcProfile_t *adrcProfile, adrcRuntime_t *adrcRuntime, float dT)
{
    const bool validDt = adrcIsFinite(dT) && dT > 0.0f;

    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        adrcCoefficient_t *c = &adrcRuntime->coefficient[axis];
        // Floors mirror the CLI ranges, as defense-in-depth for out-of-range values arriving via
        // PG/EEPROM rather than `set`: wo = 0 would freeze the observer outright (all betas 0,
        // z1 stuck at its arm-time value while P keeps acting on it) and wc = 0 would zero the
        // whole control law; both fail silently, with nothing in the CLI to hint at why.
        c->wc = constrainf(adrcProfile->wc[axis], ADRC_WC_MIN, ADRC_WC_MAX);
        c->wo = constrainf(adrcProfile->wo[axis], ADRC_WO_MIN, ADRC_WO_MAX);
        if (validDt) {
            c->wo = fminf(c->wo, ADRC_ESO_MAX_WO_DT / dT);
        }
        c->b0 = fmaxf(adrcProfile->b0[axis], ADRC_B0_MIN);
        c->kp = c->wc * c->wc;
        c->kd = 2.0f * c->wc;
        c->groundWc = c->wc; // ADRC-030 off until adrcSetGroundWc() lowers it
        c->beta1 = 3.0f * c->wo;
        c->beta2 = 3.0f * c->wo * c->wo;
        c->beta3 = c->wo * c->wo * c->wo;
        c->decayRate = fminf(adrcProfile->sigmaDecay, ADRC_SIGMA_DECAY_MAX) * 0.1f;
        const float tdHz = fminf(adrcProfile->tdHz, LPF_MAX_HZ);
        c->tdFilterGain = (tdHz > 0.0f && validDt) ? pt1FilterGain(tdHz, dT) : 0.0f;
        // Never slower than the airborne decay and never zero. This rate is no longer the thing
        // that prevents grounded windup - the gate-only growth inhibit in adrcApplyControl() does
        // that, at any throttle - but it is what pulls an already non-zero z3 back toward zero
        // while the gate is shut, so a bias picked up before the gate closed (or carried in from a
        // pre-inhibit epoch) does not simply sit there and unwind as an I kick at takeoff. Ground
        // tau is capped at ~1 s.
        const float gatedZ3Decay = fminf(adrcProfile->gatedZ3DecayRate, ADRC_GATED_Z3_DECAY_MAX) * 0.1f;
        c->gatedDecayRate = fmaxf(fmaxf(gatedZ3Decay, c->decayRate), 1.0f);

        // A pt2 gain of 1 makes the filter an exact pass-through, so adrc_gyro_lpf_hz = 0 follows
        // the usual "0 disables the filter" convention - pt2FilterGain(0, dT) would return 0, i.e.
        // an ESO input frozen at zero and a blind controller. The dT guard mirrors
        // pidInitFilters()' targetPidLooptime guard for boot-time calls before the looptime is
        // known (pt2FilterGain(hz, 0) is 0 too); the post-looptime init re-runs with the real dT.
        // Gain-only update, NOT pt2FilterInit(): init zeroes the filter states, and pidInitConfig()
        // can fire while armed for adjustment-range tuning (rc_adjustments.c). Disarmed profile
        // or controller-type switches reset the state explicitly; an in-flight gain update must
        // not make the filter re-converge from zero and feed the ESO a near-zero gyro for a few ms,
        // causing a large
        // false errorEso spike straight into z3 (adrcResetState() seeds the states on every reset
        // path, so they are never stale).
        const float gyroFilterHz = fminf(adrcProfile->gyroFilterHz, LPF_MAX_HZ);
        const float gyroFilterGain = (gyroFilterHz > 0.0f && validDt)
            ? pt2FilterGain(gyroFilterHz, dT) : 1.0f;
        pt2FilterUpdateCutoff(&adrcRuntime->gyroFilter[axis], gyroFilterGain);
    }

    adrcRuntime->b0ThrottleScale = 1.0f;
    adrcRuntime->b0ScaleMin = 1.0f; // ADRC-031 off until adrcSetB0ScaleMin()
    adrcRuntime->satZ3Inhibit = false; // ADRC-033 off until adrcSetSatZ3Inhibit()
    adrcRuntime->mixerSaturated = false;
    adrcRuntime->b0ScaleThrottle = 0.0f;
    // Callers that never learn the pidSum limits (unit-test SetUp paths init the ADRC module in
    // isolation) keep the b9-era divisor; production overwrites this one line below via
    // adrcInitZ3LogScale(), which pidInitConfig() calls with the real limits.
    adrcRuntime->z3LogScale = ADRC_Z3_LOG_SCALE;
    adrcRuntime->wcRampPerS = 0.0f; // ADRC-030: no ramp until adrcSetGroundWc(); wcBlend is gate
                                    // state and is deliberately left alone here - a same-type
                                    // re-init mid-ramp (AUX adjustment) must not finish the ramp
}

uint32_t adrcZ3LogScale(const adrcProfile_t *adrcProfile, uint16_t pidSumLimit, uint16_t pidSumLimitYaw)
{
    // Worst-case |z3| the controller itself allows (ADRC-029): the anti-windup bound in
    // adrcApplyControl() is pidSumLimit * b0, where b0 is the scheduled value - base b0 times a
    // throttle scale of up to b0ThrottleScaleMax. The same floors/clamps the runtime applies are
    // applied here so the two calculations cannot drift apart.
    //
    // The exact bound and the final ceil are computed in uint64_t: every input is an integer
    // (uint16 limits and b0, uint8 scale max), and an all-float32 version of this -
    // ceilf(worst / 32767.0f) - returned a divisor one short of the bound on ~0.06 % of the valid
    // input space, because the quotient rounds before ceilf sees it. Float32 appears below only
    // where it is the point: to model, explicitly and one rounding at a time, the two values the
    // runtime's own float32 clamp evaluation can produce. The largest possible bound,
    // 1000 * 65535 * 50, is ~3.3e9, comfortably inside uint64.
    const uint32_t maxB0Scale = constrain(adrcProfile->b0ThrottleScaleMax, 1, (uint8_t)ADRC_B0_SCALE_MAX);
    uint64_t worstZ3 = 0;
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        const uint32_t b0 = MAX((uint32_t)adrcProfile->b0[axis], (uint32_t)ADRC_B0_MIN);
        const uint32_t sumLimit = (axis == FD_YAW) ? pidSumLimitYaw : pidSumLimit;
        const uint64_t bound = (uint64_t)sumLimit * b0 * maxB0Scale;
        if (bound > worstZ3) {
            worstZ3 = bound;
        }
    }
    // The runtime evaluates its clamp in float32, and a float32 product can land ABOVE the exact
    // integer product. Which value it lands on depends on the association -ffast-math picks for
    // pidSumLimit * b0Base * b0ThrottleScale, but with these integer-valued factors only two
    // results are possible: pairing the two small factors first keeps their product exact
    // (b0 * scale <= 65535 * 50 < 2^24, and limit * scale likewise), so the full product is
    // rounded ONCE - that is (float)worstZ3; pairing limit with b0 first can round twice. Both
    // are computed below through single conversions and a single multiply each, which fast-math
    // cannot reassociate, so this covers every clamp value the firmware can actually produce -
    // without a blanket pad that would cost resolution on profiles where the product is exact
    // (a bound of exactly 524272 must keep the legacy /16, not be pushed to /17).
    const uint64_t worstZ3RoundedOnce = (uint64_t)(float)worstZ3;
    uint64_t clampCover = MAX(worstZ3, worstZ3RoundedOnce);
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        const uint32_t b0 = MAX((uint32_t)adrcProfile->b0[axis], (uint32_t)ADRC_B0_MIN);
        const uint32_t sumLimit = (axis == FD_YAW) ? pidSumLimitYaw : pidSumLimit;
        const float roundedTwice = (float)(sumLimit * b0) * (float)maxB0Scale;
        clampCover = MAX(clampCover, (uint64_t)roundedTwice);
    }
    // Smallest integer divisor whose int16 endpoint covers that clamp, floored at the legacy
    // divisor so the resolution never gets worse than b9's for no reason.
    const uint64_t debugLimit = (uint64_t)ADRC_DEBUG_LIMIT;   // 32767
    const uint64_t needed = (clampCover + debugLimit - 1) / debugLimit;
    if (needed <= (uint64_t)ADRC_Z3_LOG_SCALE) {
        return (uint32_t)ADRC_Z3_LOG_SCALE;
    }
    return (uint32_t)needed;
}

void adrcInitZ3LogScale(adrcRuntime_t *adrcRuntime, const adrcProfile_t *adrcProfile,
    uint16_t pidSumLimit, uint16_t pidSumLimitYaw)
{
    adrcRuntime->z3LogScale = (float)adrcZ3LogScale(adrcProfile, pidSumLimit, pidSumLimitYaw);
}

uint8_t adrcStateFlags(const adrcRuntime_t *adrcRuntime)
{
    uint8_t flags = 0;
    if (adrcRuntime->liftoff) {
        flags |= ADRC_STATE_LIFTOFF;
    }
    if (adrcRuntime->throttleAtIdle) {
        flags |= ADRC_STATE_THROTTLE_AT_IDLE;
    }
    flags |= (adrcRuntime->z3GrowthInhibitMask & 0x07u) << 2;
    flags |= (adrcRuntime->liftoffCause << ADRC_STATE_LIFTOFF_CAUSE_SHIFT)
        & ADRC_STATE_LIFTOFF_CAUSE_MASK;
    return flags;
}

void adrcSetGroundWc(adrcRuntime_t *adrcRuntime, uint8_t groundWc, uint16_t rampMs, uint8_t dGainTenths)
{
    // ADRC-030 (off through b11-exp8, on by default at 10 since b11): the arm-time lift with airmode on is a loop through the
    // grounded airframe whose gain is set by wc (P ~ wc^2/b0, D ~ 2*wc*wo/b0 at the observer's
    // derivative peak); a tester arm at wc 40 / wo 110 did not lift where 99/110 did. This lets a low
    // wc apply only while the liftoff gate is closed, then ramp to the flight value once it opens.
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        adrcCoefficient_t *c = &adrcRuntime->coefficient[axis];
        c->groundWc = (groundWc > 0) ? fminf(groundWc, c->wc) : c->wc;
        if (groundWc > 0 && dGainTenths > 0) {
            // ADRC-030b: bound the closed-gate D gain 2*wc*wo/b0 (see adrc.h)
            c->groundWc = fminf(c->groundWc, dGainTenths * 0.1f * c->b0 / (2.0f * c->wo));
        }
    }
    adrcRuntime->wcRampPerS = (rampMs > 0) ? 1000.0f / rampMs : 0.0f;
}

void adrcResetState(adrcRuntime_t *adrcRuntime, int axis)
{
    const float gyroRate = gyro.gyroADCf[axis];
    // Seed both cascaded pt2 stages so the filtered gyro equals the current gyro immediately.
    // Seeding z1 alone while the filter re-converges from stale (or zeroed) state would
    // manufacture exactly the errorEso kick this reset exists to prevent: at gyro = 1000 deg/s
    // the first filtered sample is ~24 deg/s (150 Hz pt2 @ 8 kHz), so errorEso ~ 976 and the
    // first z3 step is -beta3*errorEso*dT ~ -122 000.
    // vRef is seeded from the same physical state rather than zero. pidResetIterm() also calls this
    // during launch control and every loop of a 3D reversal; a zero TD reference there would create
    // a large command opposite to the current rotation until the tracker caught up.
    adrcResetAxisState(adrcRuntime, axis, gyroRate);
}

void adrcResetGate(adrcRuntime_t *adrcRuntime)
{
    adrcRuntime->liftoff = false;
    adrcRuntime->gyroActiveS = 0.0f;
    adrcRuntime->appliedActiveS = 0.0f;
    // adrcUpdatePerLoopState() recomputes this every loop, and since ADRC-026 nothing in the
    // control path reads the cached copy at all. Seed it to the grounded-and-idle assumption
    // anyway, so a reader added later cannot pair a closed gate with a stale "stick raised".
    adrcRuntime->throttleAtIdle = true;
    adrcRuntime->liftoffCause = ADRC_LIFTOFF_CAUSE_NONE;
    adrcRuntime->z3GrowthInhibitMask = 0;
    adrcRuntime->mixerSaturated = false; // ADRC-033: republished by the next mixTable(); never carry it across a reset
    adrcRuntime->wcBlend = 0.0f;
    adrcRuntime->gateResetCount++;
}

void adrcResetAll(adrcRuntime_t *adrcRuntime)
{
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        adrcResetState(adrcRuntime, axis);
    }
    adrcResetGate(adrcRuntime);
#ifdef USE_YAW_SPIN_RECOVERY
    adrcRuntime->yawSpinActivePreviousLoop = false;
#endif
}

void adrcUpdateArmTransition(adrcRuntime_t *adrcRuntime, bool armed)
{
    // With the stock default pid_at_min_throttle = ON, pidStabilisationEnabled stays true while
    // disarmed, so the !pidStabilisationEnabled reset branch in pidController() never runs and
    // the liftoff gate plus ESO state would survive disarm into the next arm cycle. Measured
    // consequence: after a landing the still-open gate lets z3 wind against outputs the ground
    // (or the disarmed motors) never let act, and that stale disturbance trim then enters the
    // next takeoff. Start every arm cycle with a fresh epoch instead (ADRC-017).
    if (armed && !adrcRuntime->wasArmed) {
        adrcResetAll(adrcRuntime);
    }
    adrcRuntime->wasArmed = armed;
}

#ifdef USE_YAW_SPIN_RECOVERY
bool adrcLatchYawSpinRecovery(adrcRuntime_t *adrcRuntime, bool yawSpinActive)
{
    // Keep the disturbance integrator suppressed for the first loop after yaw-spin detection
    // clears. That loop resumes ordinary P/D control, but must not expose a freshly estimated I
    // term immediately after recovery forced the published I channel to zero.
    const bool activeThisLoop = yawSpinActive || adrcRuntime->yawSpinActivePreviousLoop;
    adrcRuntime->yawSpinActivePreviousLoop = yawSpinActive;
    return activeThisLoop;
}
#endif

void adrcSetAppliedOutput(adrcRuntime_t *adrcRuntime, int axis, float output)
{
    adrcRuntime->lastOutput[axis] = output;
}

void adrcClearDisturbanceEstimate(adrcRuntime_t *adrcRuntime, int axis)
{
    adrcRuntime->z3[axis] = 0.0f;
}

void adrcUpdatePerLoopState(adrcRuntime_t *adrcRuntime, const adrcProfile_t *adrcProfile, float dT)
{
    const bool wasLiftoff = adrcRuntime->liftoff;
    const float gyroPeak = fmaxf(fabsf(gyro.gyroADCf[FD_ROLL]),
        fmaxf(fabsf(gyro.gyroADCf[FD_PITCH]), fabsf(gyro.gyroADCf[FD_YAW])));
    // mixTable() runs after the PID task and publishes both collective values for the next PID
    // iteration. The applied one carries the mixer's airmode headroom and feeds the b0 schedule;
    // the commanded one is sampled before that headroom and feeds the gate (ADRC-026). Both already
    // include the ALT_HOLD/GPS_RESCUE overrides, unlike mixerGetThrottle(), which intentionally
    // remains the pre-override blackbox/TPA value.
    const float rawThrottle = mixerGetAdrcThrottle();
    const float throttle = adrcIsFinite(rawThrottle) ? constrainf(rawThrottle, 0.0f, 1.0f) : 0.0f;
    const float rawCommandedThrottle = mixerGetAdrcCommandedThrottle();
    const float commandedThrottle = adrcIsFinite(rawCommandedThrottle)
        ? constrainf(rawCommandedThrottle, 0.0f, 1.0f) : 0.0f;
    // Blackbox runs after mixTable(), which has already published the NEXT iteration's collective.
    // Cache the values actually consumed here so the log stays aligned with this PID iteration.
    adrcRuntime->observedAppliedCollective = throttle;
    adrcRuntime->observedCommandedCollective = commandedThrottle;
    adrcRuntime->z3GrowthInhibitMask = 0;
    const float finiteDt = (adrcIsFinite(dT) && dT > 0.0f) ? dT : 0.0f;

    const float liftoffThrottle = adrcProfile->liftoffThrottlePercent * 0.01f;
    const float liftoffGyroDps = adrcProfile->liftoffGyroDps;
    const float liftoffHoldS = adrcProfile->liftoffHoldMs * 0.001f;
    const float gyroPathThrottleFloor = liftoffThrottle * ADRC_LIFTOFF_GYRO_THROTTLE_FRACTION;
    const bool throttleAtIdle = commandedThrottle < gyroPathThrottleFloor;
    adrcRuntime->throttleAtIdle = throttleAtIdle;

    // No mid-air re-arm (ADRC-020): nothing below closes the gate once it is open. Disarm
    // (adrcUpdateArmTransition() -> adrcResetAll()) is the only ground signal that cannot
    // false-trigger on a smooth zero-throttle float mid-flight. Controller-epoch resets close the
    // gate too, and can do so while armed - see the inhibit comment in adrcApplyControl().
    if (!adrcRuntime->liftoff) {
        if (commandedThrottle >= liftoffThrottle) {
            adrcRuntime->liftoff = true;
            adrcRuntime->liftoffCause = ADRC_LIFTOFF_CAUSE_COMMANDED_COLLECTIVE;
        } else if (gyroPeak > liftoffGyroDps && !throttleAtIdle) {
            adrcRuntime->gyroActiveS += finiteDt;
            if (adrcRuntime->gyroActiveS >= liftoffHoldS) {
                adrcRuntime->liftoff = true;
                adrcRuntime->liftoffCause = ADRC_LIFTOFF_CAUSE_GYRO;
            }
        } else {
            // Also clears the hold timer whenever throttle drops back below the floor, so time
            // spent rotating at idle cannot be banked and then completed by a later throttle blip.
            adrcRuntime->gyroActiveS = 0.0f;
        }

        // Applied-collective path - see ADRC_LIFTOFF_APPLIED_HOLD_S. Deliberately not an "else if"
        // of the commanded test above: this path must keep accumulating while the commanded value
        // sits below the threshold, which is exactly the case it exists for.
        //
        // It carries the same idle interlock as the gyro path, and for a stronger reason. Duration
        // alone does not separate ground from flight: a held stick, a wedged or tilted craft, and
        // launch control (which forces the commanded collective to zero while forcing airmode on)
        // all produce a one-sided axis demand, and the mixer then holds the applied collective above
        // the threshold indefinitely with the craft on the ground. Requiring the pilot - or an
        // automatic mode, whose throttle is in the commanded value too - to have asked for some
        // thrust is what makes the duration test meaningful. On the ten 2026-08-06 logs this
        // interlock keeps the timer at exactly zero for every pre-takeoff phase.
        //
        // Below the threshold the timer resets outright rather than leaking. A leak at the fill
        // rate turns the hold into an integrator of duty cycle - any pattern above 50% duty latches
        // eventually (55% in 1.8 s, 60% in 0.9 s), which is exactly the bursty signal the ground
        // produces, and it would void the margin the measured burst lengths are supposed to give.
        // A craft hovering right at the threshold therefore does not latch through this path; it
        // opens through the commanded test or the gyro test instead, and the fix for that case is
        // to set liftoffThrottlePercent below the actual hover, not to soften this timer.
        //
        // The hold is the constant or the configured liftoffHoldMs, whichever is longer: a pilot who
        // raises adrc_liftoff_hold_ms to harden the gate after a false open must not find that this
        // path still latches at 250 ms, and the blackbox header would then report a hold that was
        // not in force on the path that opened.
        const float appliedHoldS = fmaxf(ADRC_LIFTOFF_APPLIED_HOLD_S, liftoffHoldS);
        if (!adrcRuntime->liftoff) {
            if (!throttleAtIdle && throttle >= liftoffThrottle) {
                adrcRuntime->appliedActiveS += finiteDt;
                if (adrcRuntime->appliedActiveS >= appliedHoldS) {
                    adrcRuntime->liftoff = true;
                    adrcRuntime->liftoffCause = ADRC_LIFTOFF_CAUSE_APPLIED_COLLECTIVE;
                }
            } else {
                adrcRuntime->appliedActiveS = 0.0f;
            }
        }
    }

    if (!wasLiftoff && adrcRuntime->liftoff) {
        // Start a new actuator-feedback epoch without disturbing the observer state that kept
        // tracking gyro while the gate was closed. The mixer may already have applied airmode
        // output that the ground-constrained plant could not realise; admitting that ground-epoch
        // lastOutput as b0*u in the first open loop creates exactly the discontinuity the gate is
        // meant to prevent. Drop only that stale input. The first open loop's b0*u contribution
        // then matches the closed-path value (zero); the following loop feeds back the first output
        // actually generated during this airborne epoch.
        for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
            adrcRuntime->lastOutput[axis] = 0.0f;
        }
    }

    // ADRC-030: wc sits at groundWc while the gate is closed and ramps to the flight value after it
    // opens; the blend is per-runtime because the gate is shared across axes.
    if (!adrcRuntime->liftoff) {
        adrcRuntime->wcBlend = 0.0f;
    } else if (adrcRuntime->wcRampPerS > 0.0f) {
        adrcRuntime->wcBlend = fminf(1.0f, adrcRuntime->wcBlend + finiteDt * adrcRuntime->wcRampPerS);
    } else {
        adrcRuntime->wcBlend = 1.0f;
    }

    // Motor authority ~ throttle^2, so a hover-tuned b0 is wrong away from hover; scale only UP
    // (clamped) - scaling down would make 1/b0 huge and inject extreme output at low throttle.
    // The schedule reads a low-passed collective, not the raw per-loop value - see the
    // ADRC_B0_SCALE_THROTTLE_LPF_HZ comment up top for the two flight-measured failure modes
    // (26 Hz gain modulation through the mixer constrain; z3 rebound on throttle chops). The
    // liftoff gate above deliberately stays on the raw value: liftoff detection must be prompt.
    const float throttleLpfGain = (finiteDt > 0.0f) ? pt1FilterGain(ADRC_B0_SCALE_THROTTLE_LPF_HZ, finiteDt) : 1.0f;
    if (!adrcIsFinite(adrcRuntime->b0ScaleThrottle)) {
        adrcRuntime->b0ScaleThrottle = throttle;
    }
    adrcRuntime->b0ScaleThrottle += throttleLpfGain * (throttle - adrcRuntime->b0ScaleThrottle);
    const float hover = fmaxf(adrcProfile->hoverThrottlePercent * 0.01f, ADRC_HOVER_THROTTLE_MIN_FRACTION);
    const float throttleRatio = adrcRuntime->b0ScaleThrottle / hover;
    const float maxB0Scale = constrainf(adrcProfile->b0ThrottleScaleMax, 1.0f, ADRC_B0_SCALE_MAX);
    // ADRC-021 A/B: candidate schedule shapes, selectable per PID profile (see adrcB0Law_e).
    // Non-FIXED laws map throttleRatio < 1 below 1, while FIXED maps it to exactly 1; either way,
    // the "scale only UP" clamp stays the sole low-side policy regardless of the selected shape.
    float rawScale;
    switch (adrcProfile->b0Law) {
    case ADRC_B0_LAW_SQRT:
        rawScale = sqrtf(fmaxf(throttleRatio, 0.0f));
        break;
    case ADRC_B0_LAW_LINEAR:
        rawScale = throttleRatio;
        break;
    case ADRC_B0_LAW_FIXED:
        rawScale = 1.0f;
        break;
    case ADRC_B0_LAW_QUADRATIC:
    default:
        rawScale = throttleRatio * throttleRatio;
        break;
    }
    // ADRC-031: the low-side policy is b0ScaleMin (1.0 unless adrc_b0_scale_min lowers it) - but only
    // once airborne. With the gate closed the stick is below hover by definition, so the floor would
    // divide the P/D-path b0 by up to 5 on the ground, multiplying exactly the closed-gate loop gain
    // that ADRC-030/030b bound (and adrcSetGroundWc() caps that gain with the unscaled b0).
    const float minB0Scale = adrcRuntime->liftoff ? adrcRuntime->b0ScaleMin : 1.0f;
    adrcRuntime->b0ThrottleScale = constrainf(rawScale, minB0Scale, maxB0Scale);
}

void adrcSetSatZ3Inhibit(adrcRuntime_t *adrcRuntime, bool enabled)
{
    adrcRuntime->satZ3Inhibit = enabled;
}

void adrcSetMixerSaturated(adrcRuntime_t *adrcRuntime, bool saturated)
{
    adrcRuntime->mixerSaturated = saturated;
}

void adrcSetB0ScaleMin(adrcRuntime_t *adrcRuntime, uint8_t minPercent)
{
    // Integer decision first: the firmware is built with -ffast-math, and a float clamp at exactly
    // the floor (minPercent 20 -> 20 * 0.01f = 0.19999999) followed by a "< floor" sanity check was
    // folded by the compiler into a reset to 1.0 (b11-exp5: adrc_b0_scale_min = 20 behaved as off,
    // 21-100 worked). Never derive "off" from a float comparison against the floor.
    if (minPercent >= 100) {
        adrcRuntime->b0ScaleMin = 1.0f;
    } else if (minPercent <= ADRC_B0_SCALE_MIN_FLOOR_PERCENT) {
        adrcRuntime->b0ScaleMin = ADRC_B0_SCALE_MIN_FLOOR;
    } else {
        adrcRuntime->b0ScaleMin = minPercent * 0.01f;
    }
}

adrcOutput_t adrcApplyControl(adrcRuntime_t *adrcRuntime, int axis, float gyroRate, float currentPidSetpoint,
    float dT, float pidSumLimit)
{
    const adrcCoefficient_t *c = &adrcRuntime->coefficient[axis];
    const float finiteDt = (adrcIsFinite(dT) && dT > 0.0f) ? dT : 0.0f;
    const float finiteGyroRate = adrcIsFinite(gyroRate) ? gyroRate
        : (adrcIsFinite(adrcRuntime->z1[axis]) ? adrcRuntime->z1[axis] : 0.0f);
    const float finiteSetpoint = adrcIsFinite(currentPidSetpoint) ? currentPidSetpoint : finiteGyroRate;

    if (!adrcAxisStateIsFinite(adrcRuntime, axis)) {
        adrcResetAxisState(adrcRuntime, axis, finiteGyroRate);
    }
    // Sanity: a corrupt floor falls back to "off"; a floor merely below the constant (float rounding
    // under -ffast-math, see adrcSetB0ScaleMin) is clamped UP to the constant, never reset to off.
    if (!adrcIsFinite(adrcRuntime->b0ScaleMin) || adrcRuntime->b0ScaleMin > 1.0f
        || adrcRuntime->b0ScaleMin <= 0.0f) {
        adrcRuntime->b0ScaleMin = 1.0f;
    } else if (adrcRuntime->b0ScaleMin < ADRC_B0_SCALE_MIN_FLOOR) {
        adrcRuntime->b0ScaleMin = ADRC_B0_SCALE_MIN_FLOOR;
    }
    if (!adrcIsFinite(adrcRuntime->b0ThrottleScale) || adrcRuntime->b0ThrottleScale < adrcRuntime->b0ScaleMin) {
        adrcRuntime->b0ThrottleScale = adrcRuntime->b0ScaleMin;
    } else if (adrcRuntime->b0ThrottleScale > ADRC_B0_SCALE_MAX) {
        adrcRuntime->b0ThrottleScale = ADRC_B0_SCALE_MAX;
    }

    // Throttle-scheduled plant-gain estimate: apply the selected b0 law above hover (quadratic by
    // default), with the resulting scale cached by adrcUpdatePerLoopState().
    const float b0Base = (adrcIsFinite(c->b0) && c->b0 >= ADRC_B0_MIN) ? c->b0 : ADRC_B0_MIN;
    const float b0 = b0Base * adrcRuntime->b0ThrottleScale;

    // Dedicated low-pass ahead of the ESO. Not part of the ported source (danusha2345's own code
    // feeds the raw gyro reading directly into errorEso); this project's own addition, kept because
    // this airframe's base gyro filter is deliberately loose.
    const float filteredGyroRate = pt2FilterApply(&adrcRuntime->gyroFilter[axis], finiteGyroRate);

    const float errorEso = adrcRuntime->z1[axis] - filteredGyroRate;

    // Liftoff gate (see adrcUpdatePerLoopState()): on the ground the plant does not respond to the
    // command, so the observer only models b0*u after liftoff - otherwise z3 winds toward -b0*u
    // while grounded and has to unwind violently once airborne.
    const float b0u = adrcRuntime->liftoff ? (b0 * adrcRuntime->lastOutput[axis]) : 0.0f;

    // Extended State Observer update (Euler forward integration). z3 is a leaky integrator (rate
    // c->decayRate while airborne, 0 = classic pure integrator; c->gatedDecayRate while ungated,
    // see its comment above) so a transient disturbance bump bleeds off instead of lingering
    // indefinitely. (The ported source also has an optional decay-*scheduling* gain, slowing the
    // leak during observer transients - skipped here since the fork's own docs call it unvalidated:
    // the low-pass it keys on tracks maneuver/prop-wash transients, not a steady load.)
    const float z3DecayRate = adrcRuntime->liftoff ? c->decayRate : c->gatedDecayRate;
    adrcRuntime->z1[axis] += finiteDt * (adrcRuntime->z2[axis] - c->beta1 * errorEso);
    adrcRuntime->z2[axis] += finiteDt * (adrcRuntime->z3[axis] + b0u - c->beta2 * errorEso);

    // Split the z3 step into its decay and observer-error halves so the inhibit below can keep the
    // former while dropping the latter; with the inhibit inactive the two recombine exactly into
    // the original single update.
    const float z3Decayed = adrcRuntime->z3[axis] - finiteDt * z3DecayRate * adrcRuntime->z3[axis];
    const float z3Updated = z3Decayed - finiteDt * c->beta3 * errorEso;
    // ADRC-026: while the gate is shut, admit the observer-error term only when it moves z3 toward
    // zero. Ground excitation then decays away instead of charging the integrator that drives the
    // runaway if the gate does open.
    //
    // This keys on the gate alone. It used to also require an idle stick, which left a blind spot:
    // throttleAtIdle clears at half the liftoff threshold, so any craft still on the ground with
    // the stick past that point charged z3 freely until the gate opened. Measured on a 5" (docs/
    // flight-test-analysis/pr15400-b8-mamba): 0.6 s of that window unloaded and 5.6 s with a 1 kg
    // payload. Raising adrc_liftoff_throttle does not help: on the logged setpoint[3] proxy it
    // shortened the blind interval by roughly 6-7x (that field is rounded and sits ahead of thrust
    // linearisation, so the runtime-domain ratio is not recoverable from it) and did not suppress
    // the growth - peak logged roll/pitch z3 went 1312 -> 1491, and the value carried into gate
    // opening was higher, not lower (411 -> 1054). Only the code fix removes the interval.
    //
    // Dropping the idle condition does not reintroduce ADRC-020 (a mid-air float at zero throttle
    // freezing the observer), but NOT because the gate is permanent - adrcResetAll() closes it
    // while armed on any of the four branches guarding the reset in pidController():
    // !pidStabilisationEnabled, gyroOverflowDetected(), a wing in PASSTHRU_MODE, and Crash Flip.
    // What makes it safe is that each of those zeroes z1/z2/z3 on its way out, so no intact
    // airborne estimate is left for the closed-gate inhibit to freeze; the gate then reopens
    // whenever one of the normal detection paths meets its own conditions again. What ADRC-020 was
    // about - an intact airborne estimate blinded by stick position - cannot happen now, because
    // the stick no longer takes part in the decision at all.
    //
    // ADRC-033 (opt-in): the same inhibit while the mixer clipped on the previous iteration. The
    // observer is fed the pidsum_limit-clipped u, not what the mixer delivered (scaling u by the
    // mixer normalisation over-gains the loop, see pidUpdateAdrcAppliedOutput()), so with both
    // motor ends pinned the undelivered moment is booked as disturbance on every axis at once and
    // z3 charges to its bound in ~100 ms (Petrel75 "yaw washout", addendum 12). Only the growth
    // half is dropped; decay toward zero still runs, so a wound-up z3 unwinds during saturation.
    const bool inhibitZ3Growth = !adrcRuntime->liftoff
        || (adrcRuntime->satZ3Inhibit && adrcRuntime->mixerSaturated);
    const bool z3GrowthInhibited = inhibitZ3Growth && fabsf(z3Updated) > fabsf(z3Decayed);
    if (z3GrowthInhibited) {
        adrcRuntime->z3GrowthInhibitMask |= 1u << axis;
    }
    adrcRuntime->z3[axis] = z3GrowthInhibited ? z3Decayed : z3Updated;

    if (!adrcIsFinite(adrcRuntime->z1[axis]) || !adrcIsFinite(adrcRuntime->z2[axis])
        || !adrcIsFinite(adrcRuntime->z3[axis])) {
        adrcResetAxisState(adrcRuntime, axis, finiteGyroRate);
    }

    // Anti-windup: bound the disturbance estimate (z3) so it cannot wind up under motor/actuator
    // saturation - otherwise recovery from clipping lags while z3 unwinds. |I| = |z3/b0| is thereby
    // capped at pidSumLimit. z3 is the only ESO state that needs an authority-derived bound: it is
    // the only one with integrator memory (pre-decay, a pure integrator of the observer error),
    // while z1/z2 are servo'd back toward the measurement every iteration by their -beta*errorEso
    // terms and cannot accumulate. z1/z2 only get the generous physical divergence bounds above -
    // a tighter, authority-derived bound on z2 (pidSumLimit*b0/kd ~ 8 300 deg/s^2 at the shipped
    // defaults) rails during ordinary snaps/flips, whose real angular acceleration exceeds it
    // severalfold at/below hover, distorting the observer exactly when it must track fastest.
    adrcRuntime->z1[axis] = constrainf(adrcRuntime->z1[axis], -ADRC_Z1_LIMIT, ADRC_Z1_LIMIT);
    adrcRuntime->z2[axis] = constrainf(adrcRuntime->z2[axis], -ADRC_Z2_LIMIT, ADRC_Z2_LIMIT);
    const float finitePidSumLimit = (adrcIsFinite(pidSumLimit) && pidSumLimit > 0.0f) ? pidSumLimit : 0.0f;
    const float maxZ3 = finitePidSumLimit * b0;
    adrcRuntime->z3[axis] = constrainf(adrcRuntime->z3[axis], -maxZ3, maxZ3);

    // Tracking differentiator (opt-in, off by default): smooths the setpoint driving the control
    // law's P term, separate from the ESO's own error term above (errorEso still tracks the filtered
    // gyro directly - the TD only changes what the control law treats as "where we're steering
    // toward", not what the observer treats as "what actually happened"). tdFilterGain is the
    // unconditionally stable PT1 gain omega*dT/(1 + omega*dT), so every positive cutoff/looptime
    // combination stays monotonic. A zero gain bypasses the TD exactly.
    if (c->tdFilterGain > 0.0f) {
        adrcRuntime->vRef[axis] += c->tdFilterGain * (finiteSetpoint - adrcRuntime->vRef[axis]);
    } else {
        adrcRuntime->vRef[axis] = finiteSetpoint;
    }

    // Virtual PD control law; b0 divides out the control-input gain estimate. The terms are NOT
    // clamped individually: P and D are memoryless (nothing to wind up), the z3 clamp above
    // already caps |I| at pidSumLimit, and the mixer applies the final constrainf(Sum,
    // +/-pidSumLimit) either way, so a diverging ESO is equally bounded without them. What
    // per-term clamps would do is change closed-loop authority mid-maneuver - P legitimately
    // exceeds pidSumLimit while an opposing D partially cancels it, and clamping both cuts the
    // net drive severalfold mid-snap - which is exactly the regime the community-validated tunes
    // were flown in without them.
    // ADRC-030: effective wc = groundWc on the ground, ramping to wc after the gate opens. Equals
    // c->wc (so kp == c->kp, kd == c->kd) whenever the feature is off.
    const float wcEff = c->groundWc + (c->wc - c->groundWc) * adrcRuntime->wcBlend;
    const float kp = wcEff * wcEff;
    const float kd = 2.0f * wcEff;
    adrcOutput_t output = {
        .P = (kp * (adrcRuntime->vRef[axis] - adrcRuntime->z1[axis])) / b0,
        .D = (-kd * adrcRuntime->z2[axis]) / b0,
        .I = (-adrcRuntime->z3[axis]) / b0,
    };
    if (!adrcIsFinite(output.P) || !adrcIsFinite(output.I) || !adrcIsFinite(output.D)) {
        adrcResetAxisState(adrcRuntime, axis, finiteGyroRate);
        output.P = 0.0f;
        output.I = 0.0f;
        output.D = 0.0f;
    }

    // Log all three axes simultaneously (the ported source gates on gyro.gyroDebugAxis, one axis
    // only): roll z1/z2/z3 in [0..2], pitch z1/z2/z3 in [3..5], yaw z3 in [6], throttle-scaled b0
    // multiplier x100 sign-tagged by the liftoff latch (positive = airborne, negative = gated) in
    // [7]. z3 is logged divided by adrcRuntime->z3LogScale - profile-derived so the field spans the
    // controller's own z3 bound, written to the blackbox header as adrc_z3_log_scale (ADRC-029);
    // headers without that field imply the legacy divisor 16. debug[] is
    // int16_t and DEBUG_SET does not range-check, so an over-range value would otherwise WRAP into
    // garbage instead of reading as an honest off-scale rail (fix #12).
    if (axis == FD_ROLL) {
        DEBUG_SET(DEBUG_ADRC, 0, lrintf(constrainf(adrcRuntime->z1[axis], -ADRC_DEBUG_LIMIT, ADRC_DEBUG_LIMIT)));  //!< ESO z1 Roll rate estimate [unit:dps]
        DEBUG_SET(DEBUG_ADRC, 1, lrintf(constrainf(adrcRuntime->z2[axis], -ADRC_DEBUG_LIMIT, ADRC_DEBUG_LIMIT)));  //!< ESO z2 Roll acceleration estimate [unit:dps2]
        DEBUG_SET(DEBUG_ADRC, 2, lrintf(constrainf(adrcRuntime->z3[axis] / adrcRuntime->z3LogScale, -ADRC_DEBUG_LIMIT, ADRC_DEBUG_LIMIT)));  //!< ESO z3 Roll disturbance estimate divided by adrc_z3_log_scale
    } else if (axis == FD_PITCH) {
        DEBUG_SET(DEBUG_ADRC, 3, lrintf(constrainf(adrcRuntime->z1[axis], -ADRC_DEBUG_LIMIT, ADRC_DEBUG_LIMIT)));  //!< ESO z1 Pitch rate estimate [unit:dps]
        DEBUG_SET(DEBUG_ADRC, 4, lrintf(constrainf(adrcRuntime->z2[axis], -ADRC_DEBUG_LIMIT, ADRC_DEBUG_LIMIT)));  //!< ESO z2 Pitch acceleration estimate [unit:dps2]
        DEBUG_SET(DEBUG_ADRC, 5, lrintf(constrainf(adrcRuntime->z3[axis] / adrcRuntime->z3LogScale, -ADRC_DEBUG_LIMIT, ADRC_DEBUG_LIMIT)));  //!< ESO z3 Pitch disturbance estimate divided by adrc_z3_log_scale
        DEBUG_SET(DEBUG_ADRC, 7, lrintf((adrcRuntime->liftoff ? 1.0f : -1.0f) * adrcRuntime->b0ThrottleScale * 100.0f));  //!< b0 Throttle Scale, negative while the liftoff gate is closed [unit:0.01]
    } else { // FD_YAW
        DEBUG_SET(DEBUG_ADRC, 6, lrintf(constrainf(adrcRuntime->z3[axis] / adrcRuntime->z3LogScale, -ADRC_DEBUG_LIMIT, ADRC_DEBUG_LIMIT)));  //!< ESO z3 Yaw disturbance estimate divided by adrc_z3_log_scale
    }

    return output;
}

adrcOutput_t adrcApplyControlWithRecovery(adrcRuntime_t *adrcRuntime, int axis, float gyroRate,
    float currentPidSetpoint, float dT, float pidSumLimit, bool yawSpinRecoveryActive, bool crashRecoveryActive)
{
    // Remove a pre-recovery disturbance estimate before the ESO step so stale z3 cannot enter z2
    // once. lastOutput is kept: it is the real command applied during recovery and remains valid
    // b0*u feedback for the observer's acceleration state.
    if (yawSpinRecoveryActive || crashRecoveryActive) {
        adrcRuntime->z3[axis] = 0.0f;
    }

    adrcOutput_t output = adrcApplyControl(adrcRuntime, axis, gyroRate, currentPidSetpoint, dT, pidSumLimit);

    if (yawSpinRecoveryActive) {
        // The ESO step above may estimate a fresh disturbance from the recovery transient.
        // Suppress it through the first exit loop, then resume from a clean zero state. (Crash
        // recovery handles its own I/z3 post-pass across all axes at once - see pidController().)
        adrcRuntime->z3[axis] = 0.0f;
        output.I = 0.0f;
    }

    return output;
}

#endif // USE_ADRC
