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

// 3-state constant-acceleration Kalman filter for one axis:
// [position, velocity, acceleration]. Process noise models unknown jerk;
// position, velocity, and acceleration are all accepted as measurements.
typedef struct positionKalman_s {
    float x[3];
    float P[3][3];
    float Q_jerk;    // continuous jerk spectral density
} positionKalman_t;

void kalmanInit(positionKalman_t *kf, float initialPos, float initialVel, float initialAccel,
                float initialPosVar, float initialVelVar, float initialAccelVar, float qJerk);
void kalmanPredict(positionKalman_t *kf, float dt);
void kalmanUpdatePosition(positionKalman_t *kf, float measuredPos, float R);
void kalmanUpdateVelocity(positionKalman_t *kf, float measuredVel, float R);
void kalmanUpdateAcceleration(positionKalman_t *kf, float measuredAccel, float R);

static inline float kalmanGetPosition(const positionKalman_t *kf) { return kf->x[0]; }
static inline float kalmanGetVelocity(const positionKalman_t *kf) { return kf->x[1]; }
static inline float kalmanGetAcceleration(const positionKalman_t *kf) { return kf->x[2]; }
static inline float kalmanGetPositionVariance(const positionKalman_t *kf) { return kf->P[0][0]; }
static inline float kalmanGetVelocityVariance(const positionKalman_t *kf) { return kf->P[1][1]; }
static inline float kalmanGetAccelerationVariance(const positionKalman_t *kf) { return kf->P[2][2]; }
