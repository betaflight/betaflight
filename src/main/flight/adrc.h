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

#ifdef USE_ADRC

// User-facing tunables, embedded as a single field in pidProfile_t.
typedef struct adrcProfile_s {
    uint16_t wc[XYZ_AXIS_COUNT]; // controller (virtual PD) bandwidth per axis
    uint16_t wo[XYZ_AXIS_COUNT]; // extended state observer bandwidth per axis
    uint16_t b0[XYZ_AXIS_COUNT]; // control-input gain estimate per axis
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
} adrcProfile_t;

// Precomputed per-axis coefficients derived from adrcProfile_t at profile-load time, so the hot
// loop doesn't redo wc*wc, 3*wo, wo*wo*wo etc every iteration.
typedef struct adrcCoefficient_s {
    float wc;      // controller (virtual PD) bandwidth
    float wo;      // observer bandwidth
    float b0;      // control-input gain estimate
    float kp;      // = wc*wc (virtual PD control law proportional gain)
    float kd;      // = 2*wc (virtual PD control law derivative gain)
    float beta1;   // = 3*wo (ESO observer gain)
    float beta2;   // = 3*wo*wo (ESO observer gain)
    float beta3;   // = wo*wo*wo (ESO observer gain)
    float decayRate; // = adrcProfile->sigmaDecay * 0.1 (z3 leaky-decay rate, shared across axes)
    float tdGain;    // = 2*pi*tdHz, shared across axes; 0 = tracking differentiator disabled
} adrcCoefficient_t;

// Runtime state, embedded as a single field in pidRuntime_t.
typedef struct adrcRuntime_s {
    adrcCoefficient_t coefficient[XYZ_AXIS_COUNT];
    pt2Filter_t gyroFilter[XYZ_AXIS_COUNT]; // low-pass ahead of the ESO; see gyroFilterHz above
    float z1[XYZ_AXIS_COUNT]; // ESO estimate of rate
    float z2[XYZ_AXIS_COUNT]; // ESO estimate of rate derivative
    float z3[XYZ_AXIS_COUNT]; // ESO estimate of lumped disturbance
    float vRef[XYZ_AXIS_COUNT]; // tracking-differentiator-filtered setpoint fed to the control law;
                                 // tracks the raw setpoint directly when tdGain == 0 (disabled)
    float lastOutput[XYZ_AXIS_COUNT]; // control output fed back into the observer next iteration
    bool liftoff;           // latched once per arm cycle: craft has left the ground (shared, not
                             // per-axis - gyro activity is checked across all three axes at once)
    float gyroActiveS;      // seconds of sustained gyro activity (liftoff detector)
    float idleS;            // seconds throttle has stayed at idle (gate re-arm)
    float b0ThrottleScale;  // (throttle/hover)^2, clamped - shared cache updated once per loop by
                             // adrcUpdatePerLoopState(), applied per-axis in adrcApplyControl()
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

// Updates the shared (not per-axis) liftoff-gate latch and throttle-scaled b0 cache. Call once per
// PID loop, before the per-axis loop, so adrcApplyControl() below can read consistent state for
// all three axes in one iteration.
void adrcUpdatePerLoopState(adrcRuntime_t *adrcRuntime, const adrcProfile_t *adrcProfile, float dT);

// pidSumLimit anti-windup-clamps the ESO's disturbance estimate (z3), capping |I| = |z3/b0| at
// pidSumLimit - the ADRC equivalent of classic PID's itermLimit. See adrc.c for why z3 is the
// only state/term that gets an authority-derived bound.
adrcOutput_t adrcApplyControl(adrcRuntime_t *adrcRuntime, int axis, float gyroRate, float currentPidSetpoint,
    float dT, float pidSumLimit);

#endif // USE_ADRC
