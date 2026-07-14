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
#define ADRC_B0_SCALE_MAX 50.0f

// Liftoff-gate mechanism (thresholds now live in adrcProfile_t - see adrc.h - ported as tunable
// fields rather than fixed constants; the numbers below are just the danusha2345/ADRC-betaflight
// community-validated defaults, set in adrcResetProfile()). While the craft is ground-constrained
// the plant does not respond to the control output, but the ESO doesn't know that: it misattributes
// the "missing" response to a phantom disturbance and winds z3 up, which then has to unwind
// violently at liftoff. Until liftoff is detected - throttle above liftoffThrottlePercent, or
// any-axis rotation above liftoffGyroDps sustained for liftoffHoldMs (so a toss launch opens the
// gate almost instantly) - the observer's b0*u feedback term is held at zero. Once open the gate
// stays open until disarm, the only ground signal that cannot false-trigger mid-flight (ADRC-020:
// an earlier opt-in mid-air re-arm heuristic was removed rather than kept - throttle+gyro alone
// cannot distinguish a landing from a calm mid-air float, and adrcUpdateArmTransition() below
// already covers the ground-rep use case it existed for via a fresh epoch on every disarm->arm).

// The liftoff gate above only zeroes the b0*u term in z2's update - it does nothing to stop z3
// itself from winding up while grounded. z3 is a leaky integrator of errorEso regardless of gate
// state, and its steady-state gain (beta3/decayRate) is enormous (beta3 = wo^3, decayRate is a
// fraction of 1/s), so even a tiny sustained bias (sensor cal residual, filter phase lag) winds it
// toward its clamp given enough idle time - confirmed on a props-off bench test, where sitting
// armed at idle let yaw z3 wind to ~80% of its clamp before any stick input. While ungated, use a
// much faster decay (adrcProfile->gatedZ3DecayRate) so z3 relaxes toward zero instead of
// accumulating; it still updates smoothly (no reset discontinuity), just can't hold a wound-up
// value while the craft isn't flying.

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
    // No mid-air re-arm: the gate opens once at first liftoff and stays open until disarm, which
    // is the only ground signal that cannot false-trigger mid-flight (ADRC-020). An earlier
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
    // z3 decay rate x0.1 while ungated (grounded) - always faster than sigmaDecay above so z3
    // can't wind up while idle regardless of its configured airborne decay.
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
        c->beta1 = 3.0f * c->wo;
        c->beta2 = 3.0f * c->wo * c->wo;
        c->beta3 = c->wo * c->wo * c->wo;
        c->decayRate = fminf(adrcProfile->sigmaDecay, ADRC_SIGMA_DECAY_MAX) * 0.1f;
        const float tdHz = fminf(adrcProfile->tdHz, LPF_MAX_HZ);
        c->tdFilterGain = (tdHz > 0.0f && validDt) ? pt1FilterGain(tdHz, dT) : 0.0f;
        // Never slower than the airborne decay and never zero: adrc_gated_z3_decay = 0 would
        // silently reintroduce the grounded z3 windup this rate exists to prevent (a small sensor
        // bias integrates for as long as the craft sits armed at idle, then unwinds as an I kick
        // at takeoff - the exact failure the gated decay was added for). Ground tau is floored at
        // ~1 s.
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
    adrcRuntime->b0ScaleThrottle = 0.0f;
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
    // mixTable() runs after the PID task and publishes the final collective value for the next
    // PID iteration. This includes ALT_HOLD/GPS_RESCUE overrides and mixer constraints, unlike
    // mixerGetThrottle(), which intentionally remains the pre-override blackbox/TPA value.
    const float rawThrottle = mixerGetAdrcThrottle();
    const float throttle = adrcIsFinite(rawThrottle) ? constrainf(rawThrottle, 0.0f, 1.0f) : 0.0f;
    const float finiteDt = (adrcIsFinite(dT) && dT > 0.0f) ? dT : 0.0f;

    const float liftoffThrottle = adrcProfile->liftoffThrottlePercent * 0.01f;
    const float liftoffGyroDps = adrcProfile->liftoffGyroDps;
    const float liftoffHoldS = adrcProfile->liftoffHoldMs * 0.001f;

    // No mid-air re-arm (ADRC-020): once open, the gate stays open for the rest of the arm cycle.
    // Disarm (adrcUpdateArmTransition() -> adrcResetAll()) is the only ground signal that cannot
    // false-trigger on a smooth zero-throttle float mid-flight.
    if (!adrcRuntime->liftoff) {
        if (throttle >= liftoffThrottle) {
            adrcRuntime->liftoff = true;
        } else if (gyroPeak > liftoffGyroDps) {
            adrcRuntime->gyroActiveS += finiteDt;
            if (adrcRuntime->gyroActiveS >= liftoffHoldS) {
                adrcRuntime->liftoff = true;
            }
        } else {
            adrcRuntime->gyroActiveS = 0.0f;
        }
    }

    if (!wasLiftoff && adrcRuntime->liftoff) {
        // Start a new actuator-feedback epoch without disturbing the observer state that kept
        // tracking gyro while the gate was closed. The mixer may already have applied airmode
        // output that the ground-constrained plant could not realise; admitting that ground-epoch
        // lastOutput as b0*u in the first open loop creates exactly the discontinuity the gate is
        // meant to prevent. Drop only that stale input. The first open loop then matches the
        // closed-path observer update, and the following loop feeds back the first output actually
        // generated during this airborne epoch.
        for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
            adrcRuntime->lastOutput[axis] = 0.0f;
        }
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
    adrcRuntime->b0ThrottleScale = constrainf(throttleRatio * throttleRatio, 1.0f, maxB0Scale);
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
    if (!adrcIsFinite(adrcRuntime->b0ThrottleScale) || adrcRuntime->b0ThrottleScale < 1.0f) {
        adrcRuntime->b0ThrottleScale = 1.0f;
    } else if (adrcRuntime->b0ThrottleScale > ADRC_B0_SCALE_MAX) {
        adrcRuntime->b0ThrottleScale = ADRC_B0_SCALE_MAX;
    }

    // Throttle-scaled plant gain: motor authority ~ throttle^2, so a hover-tuned b0 stays calibrated
    // across the throttle range instead of only at hover. See adrcUpdatePerLoopState().
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
    adrcRuntime->z3[axis] += finiteDt * (-c->beta3 * errorEso - z3DecayRate * adrcRuntime->z3[axis]);

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
    // law's P term, separate from the ESO's own error term above (errorEso still tracks the raw
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
    adrcOutput_t output = {
        .P = (c->kp * (adrcRuntime->vRef[axis] - adrcRuntime->z1[axis])) / b0,
        .D = (-c->kd * adrcRuntime->z2[axis]) / b0,
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
    // [7]. z3 is logged /ADRC_Z3_LOG_SCALE so the int16 field does not clip as easily. debug[] is
    // int16_t and DEBUG_SET does not range-check, so an over-range value would otherwise WRAP into
    // garbage instead of reading as an honest off-scale rail (fix #12).
    if (axis == FD_ROLL) {
        DEBUG_SET(DEBUG_ADRC, 0, lrintf(constrainf(adrcRuntime->z1[axis], -ADRC_DEBUG_LIMIT, ADRC_DEBUG_LIMIT)));
        DEBUG_SET(DEBUG_ADRC, 1, lrintf(constrainf(adrcRuntime->z2[axis], -ADRC_DEBUG_LIMIT, ADRC_DEBUG_LIMIT)));
        DEBUG_SET(DEBUG_ADRC, 2, lrintf(constrainf(adrcRuntime->z3[axis] / ADRC_Z3_LOG_SCALE, -ADRC_DEBUG_LIMIT, ADRC_DEBUG_LIMIT)));
    } else if (axis == FD_PITCH) {
        DEBUG_SET(DEBUG_ADRC, 3, lrintf(constrainf(adrcRuntime->z1[axis], -ADRC_DEBUG_LIMIT, ADRC_DEBUG_LIMIT)));
        DEBUG_SET(DEBUG_ADRC, 4, lrintf(constrainf(adrcRuntime->z2[axis], -ADRC_DEBUG_LIMIT, ADRC_DEBUG_LIMIT)));
        DEBUG_SET(DEBUG_ADRC, 5, lrintf(constrainf(adrcRuntime->z3[axis] / ADRC_Z3_LOG_SCALE, -ADRC_DEBUG_LIMIT, ADRC_DEBUG_LIMIT)));
        DEBUG_SET(DEBUG_ADRC, 7, lrintf((adrcRuntime->liftoff ? 1.0f : -1.0f) * adrcRuntime->b0ThrottleScale * 100.0f));
    } else { // FD_YAW
        DEBUG_SET(DEBUG_ADRC, 6, lrintf(constrainf(adrcRuntime->z3[axis] / ADRC_Z3_LOG_SCALE, -ADRC_DEBUG_LIMIT, ADRC_DEBUG_LIMIT)));
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
