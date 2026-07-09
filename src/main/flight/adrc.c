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

// Generous physical bound on the ESO's rate estimate (z1) - well beyond any real gyro reading,
// just prevents unbounded divergence (e.g. from a badly tuned wo) rather than constraining normal
// flight in any way.
#define ADRC_Z1_LIMIT 2000.0f

// Liftoff-gate thresholds, ported as-is from danusha2345/ADRC-betaflight (ADRC_FIXES.md fix
// #8/#10) - community-validated on real hardware across several testers/airframes. While the
// craft is ground-constrained the plant does not respond to the control output, but the ESO
// doesn't know that: it misattributes the "missing" response to a phantom disturbance and winds
// z3 up, which then has to unwind violently at liftoff. Until liftoff is detected - throttle above
// ADRC_LIFTOFF_THROTTLE, or any-axis rotation above ADRC_LIFTOFF_GYRO_DPS sustained for
// ADRC_LIFTOFF_HOLD_S (so a toss launch opens the gate almost instantly) - the observer's b0*u
// feedback term is held at zero. The gate re-arms only after throttle stays below
// ADRC_LIFTOFF_IDLE_THROTTLE *and* the craft is still for ADRC_LIFTOFF_IDLE_HOLD_S.
#define ADRC_LIFTOFF_THROTTLE 0.4f
#define ADRC_LIFTOFF_GYRO_DPS 20.0f
#define ADRC_LIFTOFF_HOLD_S 0.025f
#define ADRC_LIFTOFF_IDLE_THROTTLE 0.05f
#define ADRC_LIFTOFF_IDLE_HOLD_S 0.5f

// Throttle-scaled b0 (fix #10a): motor authority scales with RPM, and thrust ~ RPM^2 ~ throttle^2,
// so a b0 tuned at hover is wrong away from hover. Scaled only UP from hover (clamped) - scaling
// down would make 1/b0 huge and inject extreme output at low throttle.
#define ADRC_B0_THR_SCALE_MAX 9.0f
#define ADRC_HOVER_THROTTLE_MIN_FRACTION 0.05f

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
}

void adrcInitConfig(const adrcProfile_t *adrcProfile, adrcRuntime_t *adrcRuntime, float dT)
{
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        adrcCoefficient_t *c = &adrcRuntime->coefficient[axis];
        c->wc = adrcProfile->wc[axis];
        c->wo = adrcProfile->wo[axis];
        c->b0 = (adrcProfile->b0[axis] > 0) ? adrcProfile->b0[axis] : 500.0f;
        c->kp = c->wc * c->wc;
        c->kd = 2.0f * c->wc;
        c->beta1 = 3.0f * c->wo;
        c->beta2 = 3.0f * c->wo * c->wo;
        c->beta3 = c->wo * c->wo * c->wo;
        c->decayRate = adrcProfile->sigmaDecay * 0.1f;

        pt2FilterInit(&adrcRuntime->gyroFilter[axis], pt2FilterGain(adrcProfile->gyroFilterHz, dT));
    }

    adrcRuntime->b0ThrottleScale = 1.0f;
}

void adrcResetState(adrcRuntime_t *adrcRuntime, int axis)
{
    adrcRuntime->z1[axis] = gyro.gyroADCf[axis];
    adrcRuntime->z2[axis] = 0.0f;
    adrcRuntime->z3[axis] = 0.0f;
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

    if (adrcRuntime->liftoff) {
        // Re-arm only after a sustained return to idle throttle AND stillness, so every ground-test
        // rep within one arm cycle gets re-gated. The gyro condition is essential: a mid-air
        // throttle chop (dive, split-S) reads idle on throttle alone while airmode keeps the craft
        // responding - re-arming there would blind the ESO to its own b0*u mid-flight.
        if (throttle < ADRC_LIFTOFF_IDLE_THROTTLE && gyroPeak < ADRC_LIFTOFF_GYRO_DPS) {
            adrcRuntime->idleS += dT;
            if (adrcRuntime->idleS >= ADRC_LIFTOFF_IDLE_HOLD_S) {
                adrcResetGate(adrcRuntime);
            }
        } else {
            adrcRuntime->idleS = 0.0f;
        }
    } else {
        adrcRuntime->idleS = 0.0f;
        if (throttle >= ADRC_LIFTOFF_THROTTLE) {
            adrcRuntime->liftoff = true;
        } else if (gyroPeak > ADRC_LIFTOFF_GYRO_DPS) {
            adrcRuntime->gyroActiveS += dT;
            if (adrcRuntime->gyroActiveS >= ADRC_LIFTOFF_HOLD_S) {
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
    adrcRuntime->b0ThrottleScale = constrainf(throttleRatio * throttleRatio, 1.0f, ADRC_B0_THR_SCALE_MAX);
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
    // c->decayRate, 0 = classic pure integrator) so a transient disturbance bump bleeds off instead
    // of lingering indefinitely. (The ported source also has an optional decay-*scheduling* gain,
    // slowing the leak during observer transients - skipped here since the fork's own docs call it
    // unvalidated: the low-pass it keys on tracks maneuver/prop-wash transients, not a steady load.)
    adrcRuntime->z1[axis] += dT * (adrcRuntime->z2[axis] - c->beta1 * errorEso);
    adrcRuntime->z2[axis] += dT * (adrcRuntime->z3[axis] + b0u - c->beta2 * errorEso);
    adrcRuntime->z3[axis] += dT * (-c->beta3 * errorEso - c->decayRate * adrcRuntime->z3[axis]);

    // Bound the ESO states themselves (not just their effect on P/I/D below), the same way classic
    // PID's iterm accumulator is itself clamped rather than only clamping its contribution to the
    // sum - without this, a saturated output still lets z2/z3 wind up unboundedly in the meantime,
    // which then takes far longer to unwind once the disturbance driving it stops. z2/z3's bounds
    // are derived from the same per-term pidSumLimit used below, so a term can't wind up to more
    // than it could ever contribute anyway.
    adrcRuntime->z1[axis] = constrainf(adrcRuntime->z1[axis], -ADRC_Z1_LIMIT, ADRC_Z1_LIMIT);
    const float maxZ2 = pidSumLimit * b0 / fmaxf(c->kd, 1.0f);
    adrcRuntime->z2[axis] = constrainf(adrcRuntime->z2[axis], -maxZ2, maxZ2);
    const float maxZ3 = pidSumLimit * b0;
    adrcRuntime->z3[axis] = constrainf(adrcRuntime->z3[axis], -maxZ3, maxZ3);

    // Virtual PD control law; b0 divides out the control-input gain estimate. Each term is then
    // clamped to pidSumLimit individually - mirroring how classic PID's iterm is windup-protected -
    // since unlike classic PID, nothing here otherwise stops a single term from claiming the whole
    // mixer's authority every iteration if the ESO is diverging.
    const adrcOutput_t output = {
        .P = constrainf((c->kp * (currentPidSetpoint - adrcRuntime->z1[axis])) / b0, -pidSumLimit, pidSumLimit),
        .D = constrainf((-c->kd * adrcRuntime->z2[axis]) / b0, -pidSumLimit, pidSumLimit),
        .I = constrainf((-adrcRuntime->z3[axis]) / b0, -pidSumLimit, pidSumLimit),
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
