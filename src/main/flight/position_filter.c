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

#include "platform.h"

#include "position_filter.h"

enum {
    KF_POSITION = 0,
    KF_VELOCITY,
    KF_ACCELERATION,
    KF_STATE_COUNT
};

void kalmanInit(positionKalman_t *kf, float initialPos, float initialVel, float initialAccel,
                float initialPosVar, float initialVelVar, float initialAccelVar, float qJerk)
{
    kf->x[0] = initialPos;
    kf->x[1] = initialVel;
    kf->x[2] = initialAccel;

    for (unsigned row = 0; row < KF_STATE_COUNT; row++) {
        for (unsigned column = 0; column < KF_STATE_COUNT; column++) {
            kf->P[row][column] = 0.0f;
        }
    }
    kf->P[KF_POSITION][KF_POSITION] = initialPosVar;
    kf->P[KF_VELOCITY][KF_VELOCITY] = initialVelVar;
    kf->P[KF_ACCELERATION][KF_ACCELERATION] = initialAccelVar;
    kf->Q_jerk = qJerk;
}

// Constant-acceleration prediction with continuous white jerk process noise.
//
//       [1  dt  dt^2/2]
// F  =  [0   1  dt    ]
//       [0   0   1    ]
void kalmanPredict(positionKalman_t *kf, float dt)
{
    const float dt2 = dt * dt;
    const float halfDt2 = 0.5f * dt2;

    kf->x[KF_POSITION] += kf->x[KF_VELOCITY] * dt + halfDt2 * kf->x[KF_ACCELERATION];
    kf->x[KF_VELOCITY] += kf->x[KF_ACCELERATION] * dt;

    const float F[KF_STATE_COUNT][KF_STATE_COUNT] = {
        { 1.0f, dt, halfDt2 },
        { 0.0f, 1.0f, dt },
        { 0.0f, 0.0f, 1.0f },
    };
    float FP[KF_STATE_COUNT][KF_STATE_COUNT] = {{0}};
    float predictedP[KF_STATE_COUNT][KF_STATE_COUNT] = {{0}};

    for (unsigned row = 0; row < KF_STATE_COUNT; row++) {
        for (unsigned column = 0; column < KF_STATE_COUNT; column++) {
            for (unsigned i = 0; i < KF_STATE_COUNT; i++) {
                FP[row][column] += F[row][i] * kf->P[i][column];
            }
        }
    }
    for (unsigned row = 0; row < KF_STATE_COUNT; row++) {
        for (unsigned column = 0; column < KF_STATE_COUNT; column++) {
            for (unsigned i = 0; i < KF_STATE_COUNT; i++) {
                predictedP[row][column] += FP[row][i] * F[column][i];
            }
        }
    }

    const float dt3 = dt2 * dt;
    const float dt4 = dt3 * dt;
    const float dt5 = dt4 * dt;
    const float q = kf->Q_jerk;
    const float processNoise[KF_STATE_COUNT][KF_STATE_COUNT] = {
        { q * dt5 / 20.0f, q * dt4 / 8.0f, q * dt3 / 6.0f },
        { q * dt4 / 8.0f, q * dt3 / 3.0f, q * dt2 / 2.0f },
        { q * dt3 / 6.0f, q * dt2 / 2.0f, q * dt },
    };

    for (unsigned row = 0; row < KF_STATE_COUNT; row++) {
        for (unsigned column = 0; column < KF_STATE_COUNT; column++) {
            kf->P[row][column] = predictedP[row][column] + processNoise[row][column];
        }
    }
}

static void kalmanUpdateScalar(positionKalman_t *kf, unsigned measuredState, float measurement, float R)
{
    const float S = kf->P[measuredState][measuredState] + R;
    if (S < 1e-9f) {
        return;
    }

    float gain[KF_STATE_COUNT];
    float measuredRow[KF_STATE_COUNT];
    for (unsigned i = 0; i < KF_STATE_COUNT; i++) {
        gain[i] = kf->P[i][measuredState] / S;
        measuredRow[i] = kf->P[measuredState][i];
    }

    const float innovation = measurement - kf->x[measuredState];
    for (unsigned i = 0; i < KF_STATE_COUNT; i++) {
        kf->x[i] += gain[i] * innovation;
    }

    for (unsigned row = 0; row < KF_STATE_COUNT; row++) {
        for (unsigned column = 0; column < KF_STATE_COUNT; column++) {
            kf->P[row][column] -= gain[row] * measuredRow[column];
        }
    }
}

void kalmanUpdatePosition(positionKalman_t *kf, float measuredPos, float R)
{
    kalmanUpdateScalar(kf, KF_POSITION, measuredPos, R);
}

void kalmanUpdateVelocity(positionKalman_t *kf, float measuredVel, float R)
{
    kalmanUpdateScalar(kf, KF_VELOCITY, measuredVel, R);
}

void kalmanUpdateAcceleration(positionKalman_t *kf, float measuredAccel, float R)
{
    kalmanUpdateScalar(kf, KF_ACCELERATION, measuredAccel, R);
}
