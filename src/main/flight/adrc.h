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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/axis.h"
#include "common/filter.h"

// Experimental: Active Disturbance Rejection Control (ADRC), an opt-in, per-profile alternative
// to classic PID rate control, selected via pidProfile_t.pid_type. A third-order linear ESO
// (Extended State Observer) estimates the plant state (rate, its derivative, and a lumped
// disturbance term) and a virtual PD control law drives the estimated rate to setpoint.
//
// Builds on a proof-of-concept by Godwin Pious (@Boyyt357): https://github.com/Boyyt357/ADRC-betaflight
// Robustness fixes (liftoff gate, throttle-scaled b0, anti-windup, z3 decay) ported from
// danusha2345/ADRC-betaflight: https://github.com/danusha2345/ADRC-betaflight
// Tracking differentiator (adrc_td_hz) ported from an independent implementation by SeverinBitterli:
// https://github.com/SeverinBitterli/betaflight/tree/ADRC-Implementation

typedef enum {
    PID_TYPE_CLASSIC = 0,
#ifdef USE_ADRC
    PID_TYPE_ADRC,
#endif
} pidType_e;

// Candidate laws for the throttle-scheduled b0 (ADRC-021). Temporary fork-side A/B selector:
// flight data from two crafts rejects the quadratic law (measured plant-gain growth to 40-60%
// collective is x1.3-1.7 where the quadratic applies x2.3-3) but does not pick sqrt vs linear -
// that needs a controlled same-craft A/B, one law per PID profile. The winner ships alone; this
// enum does not go upstream.
typedef enum {
    ADRC_B0_LAW_QUADRATIC = 0, // (throttle/hover)^2 - the law shipped through b10.1
    ADRC_B0_LAW_SQRT,          // sqrt(throttle/hover) - default since b11
    ADRC_B0_LAW_LINEAR,        // throttle/hover
    ADRC_B0_LAW_FIXED,         // no throttle scheduling (scale held at 1)
} adrcB0Law_e;

// User-facing tunables, embedded as a single field in pidProfile_t. Defined unconditionally -
// like the pidProfile_t field that embeds it - so the persisted profile layout is identical on
// targets that exclude the ADRC code for flash budget (see common_pre.h / pid.h).
typedef struct adrcProfile_s {
    uint16_t wc[XYZ_AXIS_COUNT]; // controller (virtual PD) bandwidth [rad/s] per axis
    uint16_t wo[XYZ_AXIS_COUNT]; // extended state observer bandwidth [rad/s] per axis
    uint16_t b0[XYZ_AXIS_COUNT]; // control-input gain estimate [deg/s^3 per PID output] per axis
    uint16_t gyroFilterHz;       // low-pass cutoff applied to the ESO's gyro input (not per-axis,
                                 // matching dterm_lpf1/lpf2's single-value convention)
    uint8_t hoverThrottlePercent; // throttle % at hover; b0 is scheduled above hover by the selected
                                   // b0Law (SQRT by default since b11; not per-axis)
    uint8_t sigmaDecay;           // z3 leaky-decay rate x0.1; 0 = classic pure integrator (not
                                   // per-axis)
    uint16_t tdHz;                // tracking-differentiator corner freq on the setpoint feeding the
                                   // control law (not the ESO's own error term); 0 = disabled
                                   // (bypass, setpoint fed straight through). Off by default - not
                                   // part of the danusha2345 port, independently added by a third
                                   // ADRC implementation (SeverinBitterli/betaflight, ADRC-Implementation
                                   // branch); standard ADRC theory component, unvalidated here.

    // Liftoff-gate thresholds (see adrcUpdatePerLoopState() in adrc.c for the state machine these
    // drive). Community-validated defaults from danusha2345/ADRC-betaflight, but craft-dependent -
    // in particular liftoffThrottlePercent has no relationship to hoverThrottlePercent above unless
    // you set one; it answers a different question ("how sure am I this throttle means I'm off the
    // ground", vs. hoverThrottlePercent's "where do I actually hover"). Set it a bit above your
    // actual hover throttle rather than equal to it.
    uint8_t liftoffThrottlePercent;     // commanded throttle % that alone confirms liftoff; also
                                        // the threshold the applied-collective path uses, and half
                                        // of it is the idle floor the other two paths need cleared
                                        // (not per-axis)
    uint8_t liftoffGyroDps;             // sustained rotation (deg/s, any axis) that confirms
                                        // liftoff once the throttle is off idle - the toss-launch
                                        // path, which is why it is not rotation alone (not per-axis)
    uint16_t liftoffHoldMs;             // how long the rotation above must sustain before it counts;
                                        // also the floor on the applied-collective path's hold, so
                                        // raising it hardens both (ADRC-020: the opt-in mid-air
                                        // re-arm heuristic that used to live here was removed rather
                                        // than fixed - throttle+gyro alone cannot distinguish a
                                        // landing from a calm mid-air float, and the arm-epoch fix
                                        // below already covers the ground-rep use case it existed
                                        // for)

    uint16_t gatedZ3DecayRate; // z3 decay rate x0.1 while ungated (grounded) - never slower than
                                // sigmaDecay above (adrcInitConfig() takes the max of the two and a
                                // 1/s floor), so a z3 that is already non-zero when the gate shuts
                                // relaxes toward zero instead of holding. What keeps |z3| from
                                // growing in the first place is the gate-only inhibit in
                                // adrcApplyControl(), not this rate (not per-axis)
    uint8_t b0ThrottleScaleMax; // ceiling on the throttle-scaled b0 multiplier (see
                                // hoverThrottlePercent above); scaling is never applied below 1x
    uint8_t b0Law;              // adrcB0Law_e: which throttle->b0 schedule shape to apply (ADRC-021
                                // A/B selector, not per-axis)
} adrcProfile_t;
// ADRC-030 ground wc and the later tester settings live at the end of pidProfile_t (see pid.h), and reach the
// runtime via adrcSetGroundWc() & co. NOTE: where a field sits does not make a size change safe - pidProfiles is
// one PG *array*, so any change to sizeof(pidProfile_t) moves the element stride and needs a PG version bump
// (see the comment at PG_REGISTER_ARRAY_WITH_RESET_FN in pid.c).

#ifdef USE_ADRC

typedef enum {
    ADRC_LIFTOFF_CAUSE_NONE = 0,
    ADRC_LIFTOFF_CAUSE_COMMANDED_COLLECTIVE,
    ADRC_LIFTOFF_CAUSE_GYRO,
    ADRC_LIFTOFF_CAUSE_APPLIED_COLLECTIVE,
} adrcLiftoffCause_e;

typedef enum {
    ADRC_STATE_LIFTOFF = 1 << 0,
    ADRC_STATE_THROTTLE_AT_IDLE = 1 << 1,
    ADRC_STATE_Z3_INHIBITED_ROLL = 1 << 2,
    ADRC_STATE_Z3_INHIBITED_PITCH = 1 << 3,
    ADRC_STATE_Z3_INHIBITED_YAW = 1 << 4,
    ADRC_STATE_LIFTOFF_CAUSE_SHIFT = 5,
    ADRC_STATE_LIFTOFF_CAUSE_MASK = 3 << ADRC_STATE_LIFTOFF_CAUSE_SHIFT,
} adrcStateFlag_e;

// Precomputed per-axis coefficients derived from adrcProfile_t at profile-load time, so the hot
// loop doesn't redo wc*wc, 3*wo, wo*wo*wo etc every iteration.
typedef struct adrcCoefficient_s {
    float wc;      // controller (virtual PD) bandwidth [rad/s]
    float wo;      // effective observer bandwidth [rad/s], capped against the runtime looptime
    float b0;      // control-input gain estimate [deg/s^3 per PID output]
    float kp;      // = wc*wc (virtual PD control law proportional gain)
    float kd;      // = 2*wc (virtual PD control law derivative gain)
    float groundWc; // wc while the gate is closed; == wc unless adrcSetGroundWc() lowered it (ADRC-030)
    float beta1;   // = 3*wo (ESO observer gain)
    float beta2;   // = 3*wo*wo (ESO observer gain)
    float beta3;   // = wo*wo*wo (ESO observer gain)
    float decayRate; // = adrcProfile->sigmaDecay * 0.1 (z3 leaky-decay rate, shared across axes)
    float tdFilterGain; // stable PT1 gain for tdHz at the runtime looptime; 0 = TD disabled
    float gatedDecayRate; // z3 decay rate while ungated (shared across axes). NOT simply
                           // gatedZ3DecayRate * 0.1: adrcInitConfig() clamps the profile value,
                           // then takes the max against decayRate above and a 1/s floor.
                           // Precomputed so adrcApplyControl() doesn't need the profile pointer
} adrcCoefficient_t;

// Runtime state, embedded as a single field in pidRuntime_t.
typedef struct adrcRuntime_s {
    adrcCoefficient_t coefficient[XYZ_AXIS_COUNT];
    pt2Filter_t gyroFilter[XYZ_AXIS_COUNT]; // low-pass ahead of the ESO; see gyroFilterHz above
    float z1[XYZ_AXIS_COUNT]; // ESO estimate of angular rate [deg/s]
    float z2[XYZ_AXIS_COUNT]; // ESO estimate of angular acceleration [deg/s^2]
    float z3[XYZ_AXIS_COUNT]; // ESO estimate of lumped rate-plant disturbance [deg/s^3]
    float vRef[XYZ_AXIS_COUNT]; // tracking-differentiator-filtered setpoint fed to the control law;
                                 // tracks the raw setpoint directly when tdFilterGain == 0 (disabled)
    float lastOutput[XYZ_AXIS_COUNT]; // control output fed back into the observer next iteration
    bool liftoff;           // craft has left the ground; the detector only ever sets it, and it
                             // is cleared only by adrcResetGate() - through adrcResetAll(), or by
                             // the one direct disarmed call in pidInitConfig() (shared, not
                             // per-axis - gyro activity is checked across all three axes at once)
    float gyroActiveS;      // seconds of sustained gyro activity (liftoff detector)
    float appliedActiveS;   // seconds the applied collective has held above the liftoff threshold
                             // - the gate's third path, see ADRC_LIFTOFF_APPLIED_HOLD_S
    bool throttleAtIdle;    // commanded collective is below the gyro path's throttle floor. The gate
                             // itself uses the local value computed in adrcUpdatePerLoopState(); this
                             // cached copy has no production reader left since the z3 inhibit stopped
                             // keying on the stick (ADRC-026), and is kept for the unit tests and
                             // for anything that wants the per-loop decision after the fact
    float b0ThrottleScale;  // scale selected by b0Law, clamped to [min, max] (SQRT by default since b11) -
                             // updated once per loop, applied per-axis in adrcApplyControl()
    float b0ScaleMin;       // ADRC-031: floor of the b0 schedule below hover, 1.0 = classic "scale only
                             // up"; set from adrc_b0_scale_min via adrcSetB0ScaleMin()
    float b0ScaleThrottle;  // low-passed collective feeding the b0 schedule above (the gate reads
                             // the raw value) - see ADRC_B0_SCALE_THROTTLE_LPF_HZ in adrc.c
    float wcBlend;          // 0 = groundWc, 1 = wc; held at 0 while the gate is closed and ramped
                             // to 1 over wcRampMs after it opens (ADRC-030)
    float wcRampPerS;       // 1 / (adrc_wc_ramp_ms * 0.001); 0 = no ramp (blend jumps to 1)
#ifdef USE_YAW_SPIN_RECOVERY
    bool yawSpinActivePreviousLoop; // holds disturbance I at zero for the first loop after yaw-spin
                                     // recovery clears; see adrcLatchYawSpinRecovery()
#endif
    bool wasArmed;          // previous loop's ARMED state; a rising edge starts a fresh ADRC epoch
                             // (ADRC-017) - see adrcUpdateArmTransition()
    float z3LogScale;       // divisor for the z3 blackbox debug fields, derived from the profile so
                             // the int16 field spans the controller's own z3 anti-windup bound
                             // (ADRC-029); mirrored into the blackbox header as adrc_z3_log_scale
    float observedCommandedCollective; // finite/clamped gate input consumed this PID iteration;
                                       // cached before mixTable() publishes the next iteration's value
    float observedAppliedCollective;   // finite/clamped b0-schedule input consumed this PID iteration
    uint32_t gateResetCount; // increments on every adrcResetGate() call; deltas expose reset epochs
                              // even when Blackbox decimation skips the exact PID iteration
    uint8_t liftoffCause;    // adrcLiftoffCause_e branch that most recently opened the gate
    uint8_t z3GrowthInhibitMask; // axis bits set only when this iteration actually suppresses the
                                 // observer-error half of a z3 update, not merely when eligible
    bool satZ3Inhibit;      // ADRC-033: also inhibit z3 growth while the mixer is saturated (opt-in)
    bool mixerSaturated;    // ADRC-033: mixer clipped (motorMixRange > 1) on the previous iteration;
                             // published by mixTable() through adrcSetMixerSaturated(), same
                             // one-iteration lag as lastOutput
} adrcRuntime_t;

// P/I/D fields are repurposed purely for blackbox/mixer compatibility; they do not carry their
// classic-PID meaning here.
typedef struct adrcOutput_s {
    float P;
    float I;
    float D;
} adrcOutput_t;

void adrcResetProfile(adrcProfile_t *adrcProfile);

void adrcInitConfig(const adrcProfile_t *adrcProfile, adrcRuntime_t *adrcRuntime, float dT);
// ADRC-030: wc used while the liftoff gate is closed (0 = off, i.e. the flight wc) and the ramp back
// to the flight wc after it opens (0 ms = switch). Call after adrcInitConfig(); does not touch the
// live ramp progress (wcBlend), which belongs to the gate: adrcResetGate() restarts it.
// ADRC-030b: dGainTenths (x0.1, 0 = off) additionally caps the ground wc per axis at
// dGain * b0 / (2 * wo), i.e. bounds the closed-gate D-path gain 2*wc*wo/b0 [PID output per deg/s]:
// tap tests ordered every outcome by that gain (>= 4 self-sustaining, 1.6-2.2 settles but rails the
// motors, <= 0.6 settles clean), not by wc alone, because b0 differs per tune.
void adrcSetGroundWc(adrcRuntime_t *adrcRuntime, uint8_t groundWc, uint16_t rampMs, uint8_t dGainTenths);
// ADRC-031 (experimental): lets the throttle->b0 schedule go BELOW 1 under hover, down to minPercent/100
// (100 = off, the b10.1 "scale only up" clamp). Below hover the plant gain is lower than at hover
// (thrust ~ rpm^2), so a b0 pinned at the hover value under-gains the loop there; testers have been
// covering that with thrust_linear (a global gain multiplier) or hover 5. FIXED law is unaffected.
// Applies only while the liftoff gate is open: on the ground the schedule stays >= 1 so the
// closed-gate loop gain bounded by ADRC-030/030b is not multiplied. Call after adrcInitConfig().
void adrcSetB0ScaleMin(adrcRuntime_t *adrcRuntime, uint8_t minPercent);
void adrcSetSatZ3Inhibit(adrcRuntime_t *adrcRuntime, bool enabled);
void adrcSetMixerSaturated(adrcRuntime_t *adrcRuntime, bool saturated);

// The z3 blackbox divisor implied by this profile: the smallest integer whose int16 endpoint
// covers the worst-case z3 anti-windup bound (pidSumLimit * b0 * b0ThrottleScaleMax, per axis)
// in every float32 evaluation the runtime can produce, floored at the legacy 16 (ADRC-029).
// Pure - blackbox header printing calls it directly.
uint32_t adrcZ3LogScale(const adrcProfile_t *adrcProfile, uint16_t pidSumLimit, uint16_t pidSumLimitYaw);

// Stores the divisor above into the runtime; called from pidInitConfig(), which owns both the
// ADRC profile and the pidSum limits. adrcInitConfig() alone leaves the legacy divisor in place.
void adrcInitZ3LogScale(adrcRuntime_t *adrcRuntime, const adrcProfile_t *adrcProfile,
    uint16_t pidSumLimit, uint16_t pidSumLimitYaw);

// Compact Blackbox state for the same PID iteration as the cached collective inputs above.
uint8_t adrcStateFlags(const adrcRuntime_t *adrcRuntime);

// Resets ESO/output state for one axis; call on iterm reset and whenever PID control is
// re-enabled (e.g. on arming) to prevent violent jumps from stale observer state.
void adrcResetState(adrcRuntime_t *adrcRuntime, int axis);

// Resets the liftoff-gate state (shared across axes, not touched by adrcResetState() above).
// Call ONLY from a controller-disabled epoch via adrcResetAll() below, or while disarmed - the one
// direct caller is pidInitConfig() on a pid_type change, which is gated on !ARMED. NOT from
// pidResetIterm()-style resets, which also fire mid-flight (launch control trigger, 3D motor
// reversal) where force-closing the gate would wrongly cut the ESO's b0*u feedback while still
// airborne.
void adrcResetGate(adrcRuntime_t *adrcRuntime);

// Resets everything: all per-axis ESO/output state, the liftoff gate, and the recovery-latch
// flags. Call on the disarm->arm transition (via adrcUpdateArmTransition() below) and from
// controller-disabled epochs - all four branches of the reset in pidController(): stabilisation
// off, gyro overflow, a wing in PASSTHRU_MODE, and Crash Flip. All four can fire while armed, the
// first one included (core.c selects PID_STABILISATION_OFF at low throttle when airmode is
// inactive and pid_at_min_throttle is off), so this can close the gate in flight.
void adrcResetAll(adrcRuntime_t *adrcRuntime);

// Feed the current ARMED state once per loop; a rising edge (disarm->arm) starts a fresh ADRC
// epoch via adrcResetAll(). This must not depend on the pidStabilisationEnabled reset path: with
// the stock default pid_at_min_throttle = ON that path never runs while disarmed (ADRC-017).
void adrcUpdateArmTransition(adrcRuntime_t *adrcRuntime, bool armed);

#ifdef USE_YAW_SPIN_RECOVERY
// Feed the current yaw-spin state once per loop; returns whether recovery handling must stay
// active this loop, which extends one loop past the detector clearing (see adrc.c).
bool adrcLatchYawSpinRecovery(adrcRuntime_t *adrcRuntime, bool yawSpinActive);
#endif

// Stores the control output actually applied to the plant this loop (post mixer normalization /
// saturation), fed back to the observer as b0*u next iteration.
void adrcSetAppliedOutput(adrcRuntime_t *adrcRuntime, int axis, float output);

// Zeroes the lumped disturbance estimate (z3) for one axis; used by crash recovery to keep a
// pre-crash disturbance trim out of the recovery control action.
void adrcClearDisturbanceEstimate(adrcRuntime_t *adrcRuntime, int axis);

// Updates the shared (not per-axis) liftoff-gate latch and throttle-scaled b0 cache. Call once per
// PID loop, before the per-axis loop, so adrcApplyControl() below can read consistent state for
// all three axes in one iteration.
void adrcUpdatePerLoopState(adrcRuntime_t *adrcRuntime, const adrcProfile_t *adrcProfile, float dT);

// pidSumLimit anti-windup-clamps the ESO's disturbance estimate (z3), capping |I| = |z3/b0| at
// pidSumLimit - the ADRC equivalent of classic PID's itermLimit. See adrc.c for why z3 is the
// only state/term that gets an authority-derived bound.
adrcOutput_t adrcApplyControl(adrcRuntime_t *adrcRuntime, int axis, float gyroRate, float currentPidSetpoint,
    float dT, float pidSumLimit);

// adrcApplyControl() plus the yaw-spin/crash recovery disturbance policy (see adrc.c). This is
// the entry point pid.c uses; the recovery flags simply pass through as false when the respective
// feature is compiled out or inactive.
adrcOutput_t adrcApplyControlWithRecovery(adrcRuntime_t *adrcRuntime, int axis, float gyroRate,
    float currentPidSetpoint, float dT, float pidSumLimit, bool yawSpinRecoveryActive, bool crashRecoveryActive);

#endif // USE_ADRC
