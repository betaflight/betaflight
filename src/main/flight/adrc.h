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
// to classic PID rate control, selected via pidProfile_t.pid_type. A second-order linear ESO
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

// User-facing tunables, embedded as a single field in pidProfile_t. Defined unconditionally -
// like the pidProfile_t field that embeds it - so the persisted profile layout is identical on
// targets that exclude the ADRC code for flash budget (see common_pre.h / pid.h).
typedef struct adrcProfile_s {
    uint16_t wc[XYZ_AXIS_COUNT]; // controller (virtual PD) bandwidth [rad/s] per axis
    uint16_t wo[XYZ_AXIS_COUNT]; // extended state observer bandwidth [rad/s] per axis
    uint16_t b0[XYZ_AXIS_COUNT]; // control-input gain estimate [deg/s^3 per PID output] per axis
    uint16_t gyroFilterHz;       // low-pass cutoff applied to the ESO's gyro input (not per-axis,
                                 // matching dterm_lpf1/lpf2's single-value convention)
    uint8_t hoverThrottlePercent; // throttle % at hover; b0 is scaled by (throttle/hover)^2 above
                                   // hover, since motor authority ~ throttle^2 (not per-axis)
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
    uint8_t liftoffThrottlePercent;     // throttle % that alone confirms liftoff (not per-axis)
    uint8_t liftoffGyroDps;             // sustained rotation (deg/s, any axis) that alone confirms
                                        // liftoff - the toss-launch path (not per-axis)
    uint16_t liftoffHoldMs;             // how long the rotation above must sustain before it counts
                                        // (ADRC-020: the opt-in mid-air re-arm heuristic that used
                                        // to live here was removed rather than fixed - throttle+gyro
                                        // alone cannot distinguish a landing from a calm mid-air
                                        // float, and the arm-epoch fix below already covers the
                                        // ground-rep use case it existed for)

    uint16_t gatedZ3DecayRate; // z3 decay rate x0.1 while ungated (grounded) - always faster than
                                // sigmaDecay above so z3 can't wind up while idle regardless of its
                                // configured airborne decay (not per-axis)
    uint8_t b0ThrottleScaleMax; // ceiling on the throttle-scaled b0 multiplier (see
                                // hoverThrottlePercent above); scaling is never applied below 1x
} adrcProfile_t;

#ifdef USE_ADRC

// Precomputed per-axis coefficients derived from adrcProfile_t at profile-load time, so the hot
// loop doesn't redo wc*wc, 3*wo, wo*wo*wo etc every iteration.
typedef struct adrcCoefficient_s {
    float wc;      // controller (virtual PD) bandwidth [rad/s]
    float wo;      // effective observer bandwidth [rad/s], capped against the runtime looptime
    float b0;      // control-input gain estimate [deg/s^3 per PID output]
    float kp;      // = wc*wc (virtual PD control law proportional gain)
    float kd;      // = 2*wc (virtual PD control law derivative gain)
    float beta1;   // = 3*wo (ESO observer gain)
    float beta2;   // = 3*wo*wo (ESO observer gain)
    float beta3;   // = wo*wo*wo (ESO observer gain)
    float decayRate; // = adrcProfile->sigmaDecay * 0.1 (z3 leaky-decay rate, shared across axes)
    float tdFilterGain; // stable PT1 gain for tdHz at the runtime looptime; 0 = TD disabled
    float gatedDecayRate; // = adrcProfile->gatedZ3DecayRate * 0.1 (z3 decay rate while ungated,
                           // shared across axes) - precomputed here so adrcApplyControl() doesn't
                           // need the profile pointer just for this one field
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
    bool liftoff;           // latched once per arm cycle: craft has left the ground (shared, not
                             // per-axis - gyro activity is checked across all three axes at once)
    float gyroActiveS;      // seconds of sustained gyro activity (liftoff detector)
    float b0ThrottleScale;  // (throttle/hover)^2, clamped - shared cache updated once per loop by
                             // adrcUpdatePerLoopState(), applied per-axis in adrcApplyControl()
    float b0ScaleThrottle;  // low-passed collective feeding the b0 schedule above (the gate reads
                             // the raw value) - see ADRC_B0_SCALE_THROTTLE_LPF_HZ in adrc.c
#ifdef USE_YAW_SPIN_RECOVERY
    bool yawSpinActivePreviousLoop; // holds disturbance I at zero for the first loop after yaw-spin
                                     // recovery clears; see adrcLatchYawSpinRecovery()
#endif
    bool wasArmed;          // previous loop's ARMED state; a rising edge starts a fresh ADRC epoch
                             // (ADRC-017) - see adrcUpdateArmTransition()
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

// Resets ESO/output state for one axis; call on iterm reset and whenever PID control is
// re-enabled (e.g. on arming) to prevent violent jumps from stale observer state.
void adrcResetState(adrcRuntime_t *adrcRuntime, int axis);

// Resets the liftoff-gate state (shared across axes, not touched by adrcResetState() above).
// Call ONLY on genuine disarm/overflow - NOT from pidResetIterm()-style resets, which also fire
// mid-flight (launch control trigger, 3D motor reversal) where force-closing the gate would wrongly
// cut the ESO's b0*u feedback while still airborne.
void adrcResetGate(adrcRuntime_t *adrcRuntime);

// Resets everything: all per-axis ESO/output state, the liftoff gate, and the recovery-latch
// flags. Call on the disarm->arm transition (via adrcUpdateArmTransition() below) and from
// controller-disabled epochs (stabilisation off, gyro overflow, Crash Flip).
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
