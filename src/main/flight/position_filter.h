/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>

// 4-state constant-acceleration Kalman filter for one axis:
// [position, velocity, acceleration, accelerometer bias].
//
// Position and velocity are accepted as direct measurements from GPS, baro,
// rangefinder and optical flow. The accelerometer is different: it is observed as
// (acceleration + bias), so the part of its reading that comes from attitude error,
// calibration error and thermal drift is estimated as a separate state rather than
// being integrated into velocity as if it were real motion. Without that state a
// small standing offset accumulates without limit whenever aiding is unavailable.
//
// Note that velocity does not take its share of the accelerometer correction from
// the covariance; kalmanUpdateAcceleration integrates the reading directly so the
// accelerometer's authority over velocity does not vary with GPS trust. See the
// comment there, including what that costs in covariance accuracy.
typedef struct positionKalman_s {
    float x[4];
    float P[4][4];
    float Q_jerk;       // continuous jerk spectral density; how freely acceleration may change
    float Q_accelBias;  // accelerometer bias random-walk spectral density; how fast the bias
                        // may be re-learned. Too high and it absorbs real manoeuvres.
} positionKalman_t;

void kalmanInit(positionKalman_t *kf, float initialPos, float initialVel, float initialAccel,
                float initialPosVar, float initialVelVar, float initialAccelVar,
                float qJerk, float qAccelBias);
void kalmanPredict(positionKalman_t *kf, float dt);
void kalmanUpdatePosition(positionKalman_t *kf, float measuredPos, float R);
void kalmanUpdateVelocity(positionKalman_t *kf, float measuredVel, float R);
void kalmanUpdateAcceleration(positionKalman_t *kf, float measuredAccel, float R, float dt);

static inline float kalmanGetPosition(const positionKalman_t *kf) { return kf->x[0]; }
static inline float kalmanGetVelocity(const positionKalman_t *kf) { return kf->x[1]; }
// Bias-corrected acceleration, suitable as a controller feed-forward or damping term.
static inline float kalmanGetAcceleration(const positionKalman_t *kf) { return kf->x[2]; }
// Mostly a measure of attitude estimate error. Useful for diagnostics: it should settle
// soon after arming and stay steady, and reaching the clamp indicates a real sensor fault.
static inline float kalmanGetAccelBias(const positionKalman_t *kf) { return kf->x[3]; }
static inline float kalmanGetPositionVariance(const positionKalman_t *kf) { return kf->P[0][0]; }
static inline float kalmanGetVelocityVariance(const positionKalman_t *kf) { return kf->P[1][1]; }
static inline float kalmanGetAccelerationVariance(const positionKalman_t *kf) { return kf->P[2][2]; }
static inline float kalmanGetAccelBiasVariance(const positionKalman_t *kf) { return kf->P[3][3]; }
