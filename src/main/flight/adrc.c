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

// Generous physical bounds on the ESO's rate estimate (z1) and rate-derivative estimate (z2) -
// well beyond any real gyro reading / angular acceleration (a hard 5" snap peaks at roughly
// 12-25k deg/s^2, so 100k leaves ample margin) - just preventing unbounded divergence (e.g. from
// a badly tuned wo) rather than constraining normal flight in any way.
#define ADRC_Z1_LIMIT 2000.0f
#define ADRC_Z2_LIMIT 100000.0f

// Liftoff-gate mechanism (thresholds now live in adrcProfile_t - see adrc.h - ported as tunable
// fields rather than fixed constants; the numbers below are just the danusha2345/ADRC-betaflight
// community-validated defaults, set in adrcResetProfile()). While the craft is ground-constrained
// the plant does not respond to the control output, but the ESO doesn't know that: it misattributes
// the "missing" response to a phantom disturbance and winds z3 up, which then has to unwind
// violently at liftoff. Until liftoff is detected - throttle above liftoffThrottlePercent, or
// any-axis rotation above liftoffGyroDps sustained for liftoffHoldMs (so a toss launch opens the
// gate almost instantly) - the observer's b0*u feedback term is held at zero. Once open the gate
// stays open until disarm; optionally (adrc_liftoff_idle_hold_ms > 0, off by default - see
// adrcResetProfile()) it re-arms after throttle stays below liftoffIdleThrottlePercent *and* the
// craft is still for liftoffIdleHoldMs.

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

// Relationships between the liftoff-gate tunables that per-value CLI ranges cannot enforce,
// sanitized at point of use in adrcUpdatePerLoopState(). These only apply when the opt-in mid-air
// re-arm is enabled at all (adrc_liftoff_idle_hold_ms > 0, see adrcResetProfile()):
// - the re-arm (idle) throttle must sit below the liftoff throttle, or one and the same throttle
//   satisfies both the open and the re-arm condition and the gate chops the ESO's b0*u feedback
//   at loop rate;
// - "stillness" for re-arm must mean actual stillness no matter how high the liftoff-detection
//   gyro threshold is pushed (a craft still rotating through a mid-air throttle chop is not
//   ground-idle), so the re-arm side is capped independently;
// - the re-arm hold must be long enough that a single quiet loop mid-chop cannot close the gate
//   in flight. (These floors cannot fix the heuristic's blind spot - a smooth ballistic float
//   satisfies any throttle+gyro stillness test a genuine landing satisfies, which is why the
//   re-arm is opt-in rather than merely sanitized.)
#define ADRC_REARM_STILL_MAX_DPS 30.0f
#define ADRC_REARM_HOLD_MIN_S 0.1f

// z3, logged in the same units as z1/z2, can exceed the int16 blackbox debug field under strong
// disturbance; scale it down so it stays readable (fix #12).
#define ADRC_Z3_LOG_SCALE 16.0f
#define ADRC_DEBUG_LIMIT 32767.0f

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
    adrcProfile->liftoffIdleThrottlePercent = 5;
    // Mid-air re-arm off by default (0 = the gate stays open from first liftoff until disarm).
    // The ported re-arm heuristic - idle throttle + stillness sustained for this hold - cannot
    // tell a landing from a smooth zero-throttle float: in the first freestyle log flown on this
    // branch (btfl_002-ACRO.bbl, SpeedyBee F7 Mini) it closed the gate mid-air three times, on
    // ballistic floats reading 1-4 deg/s for over 500 ms, each time dumping the live z3 estimate
    // (carrying up to |94k|) through the fast gated decay and blinding the observer to b0*u -
    // exactly the corruption an airmode throttle chop then slams into on the catch. Freestyle
    // *is* throttle chops and catches, so this cannot stay the default. Set > 0 only for bench /
    // repeated ground reps within one arm cycle, where the heuristic's assumptions actually hold.
    adrcProfile->liftoffIdleHoldMs = 0;
    // z3 decay rate x0.1 while ungated (grounded) - always faster than sigmaDecay above so z3
    // can't wind up while idle regardless of its configured airborne decay.
    adrcProfile->gatedZ3DecayRate = 200;
    // Ceiling on the throttle-scaled b0 multiplier (fix #10a).
    adrcProfile->b0ThrottleScaleMax = 9;
}

void adrcInitConfig(const adrcProfile_t *adrcProfile, adrcRuntime_t *adrcRuntime, float dT)
{
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        adrcCoefficient_t *c = &adrcRuntime->coefficient[axis];
        // Floors mirror the CLI ranges, as defense-in-depth for out-of-range values arriving via
        // PG/EEPROM rather than `set`: wo = 0 would freeze the observer outright (all betas 0,
        // z1 stuck at its arm-time value while P keeps acting on it) and wc = 0 would zero the
        // whole control law; both fail silently, with nothing in the CLI to hint at why.
        c->wc = fmaxf(adrcProfile->wc[axis], 5.0f);
        c->wo = fmaxf(adrcProfile->wo[axis], 10.0f);
        c->b0 = fmaxf(adrcProfile->b0[axis], 100.0f);
        c->kp = c->wc * c->wc;
        c->kd = 2.0f * c->wc;
        c->beta1 = 3.0f * c->wo;
        c->beta2 = 3.0f * c->wo * c->wo;
        c->beta3 = c->wo * c->wo * c->wo;
        c->decayRate = adrcProfile->sigmaDecay * 0.1f;
        c->tdGain = (adrcProfile->tdHz > 0) ? (2.0f * M_PIf * adrcProfile->tdHz) : 0.0f;
        // Never slower than the airborne decay and never zero: adrc_gated_z3_decay = 0 would
        // silently reintroduce the grounded z3 windup this rate exists to prevent (a small sensor
        // bias integrates for as long as the craft sits armed at idle, then unwinds as an I kick
        // at takeoff - the exact failure the gated decay was added for). Ground tau is floored at
        // ~1 s.
        c->gatedDecayRate = fmaxf(fmaxf(adrcProfile->gatedZ3DecayRate * 0.1f, c->decayRate), 1.0f);

        // A pt2 gain of 1 makes the filter an exact pass-through, so adrc_gyro_lpf_hz = 0 follows
        // the usual "0 disables the filter" convention - pt2FilterGain(0, dT) would return 0, i.e.
        // an ESO input frozen at zero and a blind controller. The dT guard mirrors
        // pidInitFilters()' targetPidLooptime guard for boot-time calls before the looptime is
        // known (pt2FilterGain(hz, 0) is 0 too); the post-looptime init re-runs with the real dT.
        // Gain-only update, NOT pt2FilterInit(): init zeroes the filter states, and pidInitConfig()
        // fires while armed - adjustment-range tuning (rc_adjustments.c) and pid-profile switches -
        // so re-converging from zero would feed the ESO a near-zero gyro for a few ms, a large
        // false errorEso spike straight into z3 (adrcResetState() seeds the states on every reset
        // path, so they are never stale).
        const float gyroFilterGain = (adrcProfile->gyroFilterHz > 0 && dT > 0)
            ? pt2FilterGain(adrcProfile->gyroFilterHz, dT) : 1.0f;
        pt2FilterUpdateCutoff(&adrcRuntime->gyroFilter[axis], gyroFilterGain);
    }

    adrcRuntime->b0ThrottleScale = 1.0f;
}

void adrcResetState(adrcRuntime_t *adrcRuntime, int axis)
{
    const float gyroRate = gyro.gyroADCf[axis];
    // Seed both cascaded pt2 stages so the filtered gyro equals the current gyro immediately.
    // Seeding z1 alone while the filter re-converges from stale (or zeroed) state would
    // manufacture exactly the errorEso kick this reset exists to prevent: at gyro = 1000 deg/s
    // the first filtered sample is ~24 deg/s (150 Hz pt2 @ 8 kHz), so errorEso ~ 976 and the
    // first z3 step is -beta3*errorEso*dT ~ -122 000.
    adrcRuntime->gyroFilter[axis].state = gyroRate;
    adrcRuntime->gyroFilter[axis].state1 = gyroRate;
    adrcRuntime->z1[axis] = gyroRate;
    adrcRuntime->z2[axis] = 0.0f;
    adrcRuntime->z3[axis] = 0.0f;
    adrcRuntime->vRef[axis] = 0.0f;
    adrcRuntime->lastOutput[axis] = 0.0f;
}

void adrcResetGate(adrcRuntime_t *adrcRuntime)
{
    adrcRuntime->liftoff = false;
    adrcRuntime->gyroActiveS = 0.0f;
    adrcRuntime->idleS = 0.0f;
}

void adrcUpdatePerLoopState(adrcRuntime_t *adrcRuntime, const adrcProfile_t *adrcProfile, float dT)
{
    const float gyroPeak = fmaxf(fabsf(gyro.gyroADCf[FD_ROLL]),
        fmaxf(fabsf(gyro.gyroADCf[FD_PITCH]), fabsf(gyro.gyroADCf[FD_YAW])));
    const float throttle = mixerGetThrottle();

    const float liftoffThrottle = adrcProfile->liftoffThrottlePercent * 0.01f;
    const float liftoffGyroDps = adrcProfile->liftoffGyroDps;
    const float liftoffHoldS = adrcProfile->liftoffHoldMs * 0.001f;
    // 0 disables the mid-air re-arm outright (the default - see adrcResetProfile()): the gate then
    // opens at first liftoff and stays open for the rest of the arm cycle, disarm being the only
    // ground signal that cannot false-trigger on a smooth zero-throttle float.
    const bool rearmEnabled = adrcProfile->liftoffIdleHoldMs > 0;
    // See the ADRC_REARM_* comment up top for why these three are sanitized rather than read raw.
    const float liftoffIdleThrottle = fminf(adrcProfile->liftoffIdleThrottlePercent * 0.01f, liftoffThrottle - 0.01f);
    const float liftoffIdleHoldS = fmaxf(adrcProfile->liftoffIdleHoldMs * 0.001f, ADRC_REARM_HOLD_MIN_S);
    const float rearmStillGyroDps = fminf(liftoffGyroDps, ADRC_REARM_STILL_MAX_DPS);

    if (adrcRuntime->liftoff) {
        // Opt-in re-arm for bench / repeated ground reps within one arm cycle: a sustained return
        // to idle throttle AND stillness. The gyro condition keeps an ordinary mid-air throttle
        // chop (dive, split-S - craft still rotating) from re-gating, but it cannot tell a landing
        // from a smooth ballistic float (throttle 0, gyro a few deg/s for over a second is normal
        // freestyle), and a mid-air re-gate both dumps z3 through the fast gated decay and blinds
        // the ESO to b0*u while airmode keeps flying the craft - hence opt-in, default off.
        if (rearmEnabled && throttle < liftoffIdleThrottle && gyroPeak < rearmStillGyroDps) {
            adrcRuntime->idleS += dT;
            if (adrcRuntime->idleS >= liftoffIdleHoldS) {
                adrcResetGate(adrcRuntime);
            }
        } else {
            adrcRuntime->idleS = 0.0f;
        }
    } else {
        adrcRuntime->idleS = 0.0f;
        if (throttle >= liftoffThrottle) {
            adrcRuntime->liftoff = true;
        } else if (gyroPeak > liftoffGyroDps) {
            adrcRuntime->gyroActiveS += dT;
            if (adrcRuntime->gyroActiveS >= liftoffHoldS) {
                adrcRuntime->liftoff = true;
            }
        } else {
            adrcRuntime->gyroActiveS = 0.0f;
        }
    }

    // Motor authority ~ throttle^2, so a hover-tuned b0 is wrong away from hover; scale only UP
    // (clamped) - scaling down would make 1/b0 huge and inject extreme output at low throttle.
    const float hover = fmaxf(adrcProfile->hoverThrottlePercent * 0.01f, ADRC_HOVER_THROTTLE_MIN_FRACTION);
    const float throttleRatio = constrainf(throttle, 0.0f, 1.0f) / hover;
    adrcRuntime->b0ThrottleScale = constrainf(throttleRatio * throttleRatio, 1.0f, adrcProfile->b0ThrottleScaleMax);
}

adrcOutput_t adrcApplyControl(adrcRuntime_t *adrcRuntime, int axis, float gyroRate, float currentPidSetpoint,
    float dT, float pidSumLimit)
{
    const adrcCoefficient_t *c = &adrcRuntime->coefficient[axis];

    // Throttle-scaled plant gain: motor authority ~ throttle^2, so a hover-tuned b0 stays calibrated
    // across the throttle range instead of only at hover. See adrcUpdatePerLoopState().
    const float b0 = c->b0 * adrcRuntime->b0ThrottleScale;

    // Dedicated low-pass ahead of the ESO. Not part of the ported source (danusha2345's own code
    // feeds the raw gyro reading directly into errorEso); this project's own addition, kept because
    // this airframe's base gyro filter is deliberately loose.
    const float filteredGyroRate = pt2FilterApply(&adrcRuntime->gyroFilter[axis], gyroRate);

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
    adrcRuntime->z1[axis] += dT * (adrcRuntime->z2[axis] - c->beta1 * errorEso);
    adrcRuntime->z2[axis] += dT * (adrcRuntime->z3[axis] + b0u - c->beta2 * errorEso);
    adrcRuntime->z3[axis] += dT * (-c->beta3 * errorEso - z3DecayRate * adrcRuntime->z3[axis]);

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
    const float maxZ3 = pidSumLimit * b0;
    adrcRuntime->z3[axis] = constrainf(adrcRuntime->z3[axis], -maxZ3, maxZ3);

    // Tracking differentiator (opt-in, off by default): smooths the setpoint driving the control
    // law's P term, separate from the ESO's own error term above (errorEso still tracks the raw
    // gyro directly - the TD only changes what the control law treats as "where we're steering
    // toward", not what the observer treats as "what actually happened"). tdGain == 0 bypasses it
    // (vRef tracks currentPidSetpoint exactly, one-loop-delay-free).
    if (c->tdGain > 0.0f) {
        adrcRuntime->vRef[axis] += dT * (c->tdGain * (currentPidSetpoint - adrcRuntime->vRef[axis]));
    } else {
        adrcRuntime->vRef[axis] = currentPidSetpoint;
    }

    // Virtual PD control law; b0 divides out the control-input gain estimate. The terms are NOT
    // clamped individually: P and D are memoryless (nothing to wind up), the z3 clamp above
    // already caps |I| at pidSumLimit, and the mixer applies the final constrainf(Sum,
    // +/-pidSumLimit) either way, so a diverging ESO is equally bounded without them. What
    // per-term clamps would do is change closed-loop authority mid-maneuver - P legitimately
    // exceeds pidSumLimit while an opposing D partially cancels it, and clamping both cuts the
    // net drive severalfold mid-snap - which is exactly the regime the community-validated tunes
    // were flown in without them.
    const adrcOutput_t output = {
        .P = (c->kp * (adrcRuntime->vRef[axis] - adrcRuntime->z1[axis])) / b0,
        .D = (-c->kd * adrcRuntime->z2[axis]) / b0,
        .I = (-adrcRuntime->z3[axis]) / b0,
    };

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

#endif // USE_ADRC
