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

// Four measurement updates are provided, differing in how far each innovation is
// allowed to propagate. None of them corrects the acceleration state from a
// position or velocity measurement; acceleration is driven by the IMU alone.
//
//   kalmanUpdatePosition            position     -> position only
//   kalmanUpdatePositionToVelocity  position     -> position + velocity
//   kalmanUpdateVelocityToPosition  velocity     -> velocity + position
//   kalmanUpdateAcceleration        acceleration -> acceleration only
//
// The choice is per sensor, and follows what else that sensor supplies:
//
//   GPS position (XY and altitude)  kalmanUpdatePosition
//   GPS velocity (XY and vertical)  kalmanUpdateVelocityToPosition
//   Baro, rangefinder               kalmanUpdatePositionToVelocity when no velocity from GPS, else kalmanUpdatePosition
//   Optical flow                    kalmanUpdateVelocityToPosition
//   IMU                             kalmanUpdateAcceleration
//
// GPS position stays narrow because GPS measures velocity directly alongside it,
// so the position innovation has no need to drive velocity. Baro and rangefinder
// report height with no vertical velocity, so they take the wider form and let
// the position innovation correct velocity too.
//
// Baro and rangefinder must make that choice together, never one each: they measure
// height against origins that drift relative to each other, so if only one of them
// can write velocity, their standing position disagreement is rectified into an
// unbounded velocity bias. See zPositionInnovationDrivesVelocity().
//
// Optical flow and GPS velocity are
// the only XY velocity sources, and optical flow carries no position measurement
// at all, so kalmanUpdateVelocityToPosition feeds K[KF_POSITION] — the sole path
// correcting XY position when GPS is absent, without which position would be an
// open-loop integral of velocity that drifts without bound.
//
// The two wider updates use the Joseph form; the two narrow ones scale the
// affected covariance terms directly, which is equivalent for their zeroed gains.

// Redefine the origin the position state is measured against. Velocity and
// acceleration are rates, so they are invariant under a change of origin, and the
// covariance describes spreads rather than absolute values, so only the position
// state moves.
//
// This is not a measurement update and must not be confused with one. It is for the
// case where a better absolute reference arrives and the estimator's zero is found to
// have been wrong: pushing that correction through kalmanUpdatePositionToVelocity()
// instead would differentiate it into a phantom velocity, which is precisely what a
// frame shift must not produce. Every other source's offset has to be shifted by the
// same amount, or they will drag the state back.
void kalmanShiftPosition(positionKalman_t *kf, float deltaPosition)
{
    kf->x[KF_POSITION] += deltaPosition;
}

void kalmanUpdatePosition(positionKalman_t *kf, float measuredPosition, float R)
{
    const float S = kf->P[KF_POSITION][KF_POSITION] + R;
    if (S < 1e-9f) {
        return;
    }

    const float gain = kf->P[KF_POSITION][KF_POSITION] / S;
    const float innovation = measuredPosition - kf->x[KF_POSITION];

    kf->x[KF_POSITION] += gain * innovation;

    const float scale = 1.0f - gain;

    kf->P[KF_POSITION][KF_POSITION] *= scale;

    kf->P[KF_POSITION][KF_VELOCITY] *= scale;
    kf->P[KF_VELOCITY][KF_POSITION] = kf->P[KF_POSITION][KF_VELOCITY];

    kf->P[KF_POSITION][KF_ACCELERATION] *= scale;
    kf->P[KF_ACCELERATION][KF_POSITION] = kf->P[KF_POSITION][KF_ACCELERATION];
}

// Joseph form covariance update: P = A P A' + K R K', with A = (I - K H).
//
// Shared by the two updates that correct more than one state, so they cannot drift apart:
// all that differs between them is the A matrix and the K vector, and H is already folded
// into A by the caller.
static void kalmanJosephCovarianceUpdate(positionKalman_t *kf,
                                         const float A[KF_STATE_COUNT][KF_STATE_COUNT],
                                         const float K[KF_STATE_COUNT],
                                         float R)
{
    float AP[KF_STATE_COUNT][KF_STATE_COUNT] = {{0}};
    float newP[KF_STATE_COUNT][KF_STATE_COUNT] = {{0}};

    for (unsigned row = 0; row < KF_STATE_COUNT; row++) {
        for (unsigned column = 0; column < KF_STATE_COUNT; column++) {
            for (unsigned i = 0; i < KF_STATE_COUNT; i++) {
                AP[row][column] += A[row][i] * kf->P[i][column];
            }
        }
    }

    for (unsigned row = 0; row < KF_STATE_COUNT; row++) {
        for (unsigned column = 0; column < KF_STATE_COUNT; column++) {
            for (unsigned i = 0; i < KF_STATE_COUNT; i++) {
                newP[row][column] += AP[row][i] * A[column][i];
            }

            newP[row][column] += K[row] * R * K[column];
        }
    }

    for (unsigned row = 0; row < KF_STATE_COUNT; row++) {
        for (unsigned column = 0; column < KF_STATE_COUNT; column++) {
            kf->P[row][column] = newP[row][column];
        }
    }
}

void kalmanUpdateVelocityToPosition(
    positionKalman_t *kf,
    float measuredVelocity,
    float R)
{
    const float Pvv = kf->P[KF_VELOCITY][KF_VELOCITY];
    const float S = Pvv + R;

    if (S < 1e-9f) {
        return;
    }

    const float innovation =
        measuredVelocity - kf->x[KF_VELOCITY];

    // Velocity innovation corrects velocity and position, but *not* acceleration

    const float K[KF_STATE_COUNT] = {
        [KF_POSITION]     = kf->P[KF_POSITION][KF_VELOCITY] / S,
        [KF_VELOCITY]     = Pvv / S,
        [KF_ACCELERATION] = 0.0f,
    };

    kf->x[KF_POSITION] += K[KF_POSITION] * innovation;
    kf->x[KF_VELOCITY] += K[KF_VELOCITY] * innovation;

    // A = (I - K H) for a velocity measurement, H = [0 1 0]
    const float A[KF_STATE_COUNT][KF_STATE_COUNT] = {
        { 1.0f, -K[KF_POSITION],             0.0f },
        { 0.0f,  1.0f - K[KF_VELOCITY],     0.0f },
        { 0.0f, -K[KF_ACCELERATION],         1.0f },
    };

    kalmanJosephCovarianceUpdate(kf, A, K, R);
}

void kalmanUpdatePositionToVelocity(positionKalman_t *kf, float measuredPosition, float R)
{
    const float Ppp = kf->P[KF_POSITION][KF_POSITION];
    const float S = Ppp + R;

    if (S < 1e-9f) {
        return;
    }

    const float innovation = measuredPosition - kf->x[KF_POSITION];

    // Updates position and velocity from position innovation, but not acceleration.

    const float K[KF_STATE_COUNT] = {
        [KF_POSITION]     = Ppp / S,
        [KF_VELOCITY]     = kf->P[KF_VELOCITY][KF_POSITION] / S,
        [KF_ACCELERATION] = 0.0f,
    };

    kf->x[KF_POSITION] += K[KF_POSITION] * innovation;
    kf->x[KF_VELOCITY] += K[KF_VELOCITY] * innovation;

    // A = (I - K H) for a position measurement, H = [1 0 0]
    const float A[KF_STATE_COUNT][KF_STATE_COUNT] = {
        { 1.0f - K[KF_POSITION], 0.0f, 0.0f },
        {       -K[KF_VELOCITY], 1.0f, 0.0f },
        {   -K[KF_ACCELERATION], 0.0f, 1.0f },
    };

    kalmanJosephCovarianceUpdate(kf, A, K, R);
}

void kalmanUpdateAcceleration(positionKalman_t *kf, float measuredAccel, float R)
{
    const float Paa = kf->P[KF_ACCELERATION][KF_ACCELERATION];
    const float S = Paa + R;

    if (S < 1e-9f) {
        return;
    }

    const float gain = Paa / S;
    const float innovation = measuredAccel - kf->x[KF_ACCELERATION];

    kf->x[KF_ACCELERATION] += gain * innovation;

    const float scale = 1.0f - gain;

    kf->P[KF_ACCELERATION][KF_ACCELERATION] *= scale;

    kf->P[KF_POSITION][KF_ACCELERATION] *= scale;
    kf->P[KF_ACCELERATION][KF_POSITION] = kf->P[KF_POSITION][KF_ACCELERATION];

    kf->P[KF_VELOCITY][KF_ACCELERATION] *= scale;
    kf->P[KF_ACCELERATION][KF_VELOCITY] = kf->P[KF_VELOCITY][KF_ACCELERATION];
}
