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

void kalmanInit(positionKalman_t *kf, float initialPos, float initialVel,
                float initialPosVar, float initialVelVar, float qAccel)
{
    kf->x[0] = initialPos;
    kf->x[1] = initialVel;
    kf->P[0][0] = initialPosVar;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = initialVelVar;
    kf->Q_accel = qAccel;
}

// Predict step with acceleration control input.
//
// State transition:  x = F*x + B*u
//   F = [1, dt]    B = [0.5*dt^2]
//       [0,  1]        [dt      ]
//
// Covariance:  P = F*P*F' + B*Q_accel*B'
void kalmanPredict(positionKalman_t *kf, float dt, float accel)
{
    const float dt2 = dt * dt;
    const float halfDt2 = 0.5f * dt2;

    // State prediction
    kf->x[0] += kf->x[1] * dt + halfDt2 * accel;
    kf->x[1] += dt * accel;

    // Covariance prediction: P = F*P*F' + B*Q*B'
    // F*P*F':
    //   P00' = P00 + dt*(P10 + P01) + dt^2*P11
    //   P01' = P01 + dt*P11
    //   P10' = P10 + dt*P11
    //   P11' = P11
    // B*Q*B':
    //   [0.25*dt^4, 0.5*dt^3] * Q_accel
    //   [0.5*dt^3,  dt^2     ]
    const float q = kf->Q_accel;
    const float p00 = kf->P[0][0];
    const float p01 = kf->P[0][1];
    const float p10 = kf->P[1][0];
    const float p11 = kf->P[1][1];

    kf->P[0][0] = p00 + dt * (p10 + p01) + dt2 * p11 + 0.25f * dt2 * dt2 * q;
    kf->P[0][1] = p01 + dt * p11 + halfDt2 * dt * q;
    kf->P[1][0] = p10 + dt * p11 + halfDt2 * dt * q;
    kf->P[1][1] = p11 + dt2 * q;
}

// Scalar measurement update for position: H = [1, 0]
void kalmanUpdatePosition(positionKalman_t *kf, float measuredPos, float R)
{
    const float p00 = kf->P[0][0];
    const float p10 = kf->P[1][0];

    // Innovation covariance: S = H*P*H' + R = P[0][0] + R
    const float S = p00 + R;
    if (S < 1e-9f) {
        return;
    }
    const float Sinv = 1.0f / S;

    // Kalman gain: K = P*H' / S = [P[0][0], P[1][0]]' / S
    const float K0 = p00 * Sinv;
    const float K1 = p10 * Sinv;

    // Innovation
    const float y = measuredPos - kf->x[0];

    // State update
    kf->x[0] += K0 * y;
    kf->x[1] += K1 * y;

    // Covariance update: P = (I - K*H) * P
    // Using Joseph form for numerical stability: P = (I-KH)*P*(I-KH)' + K*R*K'
    // Simplified since H = [1,0]:
    const float I_K0 = 1.0f - K0;
    const float p01 = kf->P[0][1];
    const float p11 = kf->P[1][1];

    kf->P[0][0] = I_K0 * p00 - K0 * 0.0f;   // simplified: (1-K0)*P00
    kf->P[0][1] = I_K0 * p01;
    kf->P[1][0] = p10 - K1 * p00;
    kf->P[1][1] = p11 - K1 * p01;
}

// Scalar measurement update for velocity: H = [0, 1]
void kalmanUpdateVelocity(positionKalman_t *kf, float measuredVel, float R)
{
    const float p01 = kf->P[0][1];
    const float p11 = kf->P[1][1];

    // Innovation covariance: S = H*P*H' + R = P[1][1] + R
    const float S = p11 + R;
    if (S < 1e-9f) {
        return;
    }
    const float Sinv = 1.0f / S;

    // Kalman gain: K = P*H' / S = [P[0][1], P[1][1]]' / S
    const float K0 = p01 * Sinv;
    const float K1 = p11 * Sinv;

    // Innovation
    const float y = measuredVel - kf->x[1];

    // State update
    kf->x[0] += K0 * y;
    kf->x[1] += K1 * y;

    // Covariance update: P = (I - K*H) * P, H = [0,1]
    const float p00 = kf->P[0][0];
    const float p10 = kf->P[1][0];
    const float I_K1 = 1.0f - K1;

    kf->P[0][0] = p00 - K0 * p10;
    kf->P[0][1] = p01 - K0 * p11;
    kf->P[1][0] = I_K1 * p10;
    kf->P[1][1] = I_K1 * p11;
}
