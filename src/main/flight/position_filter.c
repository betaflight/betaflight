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

#include "common/maths.h"

#include "position_filter.h"

// KF_ACCELERATION is the true earth-frame acceleration of the craft. KF_ACCEL_BIAS is
// the slowly varying offset between that and what the accelerometer reports, dominated
// in practice by attitude estimate error rather than by the sensor itself: horizontal
// acceleration is obtained by rotating body acceleration into ENU, so one degree of
// attitude error manufactures about 17 cm/s^2 of acceleration that never happened.
// Separating the two keeps that error out of the velocity integration.
enum {
    KF_POSITION = 0,
    KF_VELOCITY,
    KF_ACCELERATION,
    KF_ACCEL_BIAS,
    KF_STATE_COUNT
};

// The prior is deliberately wide relative to the bias values actually seen in flight, so
// that a standing bias is acquired within a few seconds of arming instead of over the
// course of a flight. Steady-state tracking speed is set by Q_accelBias, not by this.
#define INITIAL_ACCEL_BIAS_VAR  2500.0f     // (cm/s^2)^2

// Hard bound on the bias, needed because the bias is only observable while position or
// velocity aiding is present. Without aiding the accelerometer alone cannot distinguish
// bias from acceleration, so the state must not be free to wander somewhere that would
// corrupt velocity once it is trusted again. 150 cm/s^2 is around 8.8 degrees of
// attitude error, far beyond anything legitimate, so clipping here indicates a real
// calibration or alignment fault rather than normal operation.
#define ACCEL_BIAS_LIMIT        150.0f      // cm/s^2

void kalmanInit(positionKalman_t *kf, float initialPos, float initialVel, float initialAccel,
                float initialPosVar, float initialVelVar, float initialAccelVar,
                float qJerk, float qAccelBias)
{
    kf->x[KF_POSITION] = initialPos;
    kf->x[KF_VELOCITY] = initialVel;
    kf->x[KF_ACCELERATION] = initialAccel;
    // Always start from no assumed bias and re-learn it. Carrying a stale value across a
    // reset would be worse than starting from zero, since the attitude error it mostly
    // represents depends on conditions that may have changed.
    kf->x[KF_ACCEL_BIAS] = 0.0f;

    for (unsigned row = 0; row < KF_STATE_COUNT; row++) {
        for (unsigned column = 0; column < KF_STATE_COUNT; column++) {
            kf->P[row][column] = 0.0f;
        }
    }
    kf->P[KF_POSITION][KF_POSITION] = initialPosVar;
    kf->P[KF_VELOCITY][KF_VELOCITY] = initialVelVar;
    kf->P[KF_ACCELERATION][KF_ACCELERATION] = initialAccelVar;
    kf->P[KF_ACCEL_BIAS][KF_ACCEL_BIAS] = INITIAL_ACCEL_BIAS_VAR;
    kf->Q_jerk = qJerk;
    kf->Q_accelBias = qAccelBias;
}

// Constant-acceleration prediction with continuous white jerk process noise.
//
// The bias occupies its own uncoupled row: it is modelled as a random walk that holds
// its value between updates and never drives position or velocity, because it is not a
// property of the craft's motion. It affects the estimate only in
// kalmanUpdateAcceleration, where the accelerometer reading is corrected by it. Its
// correlation with the motion states, which is what lets GPS correct it, is built up by
// the accelerometer update rather than here.
//
//       [1  dt  dt^2/2  0]
// F  =  [0   1  dt      0]
//       [0   0   1      0]
//       [0   0   0      1]
void kalmanPredict(positionKalman_t *kf, float dt)
{
    const float dt2 = dt * dt;
    const float halfDt2 = 0.5f * dt2;

    kf->x[KF_POSITION] += kf->x[KF_VELOCITY] * dt + halfDt2 * kf->x[KF_ACCELERATION];
    kf->x[KF_VELOCITY] += kf->x[KF_ACCELERATION] * dt;

    const float F[KF_STATE_COUNT][KF_STATE_COUNT] = {
        { 1.0f, dt, halfDt2, 0.0f },
        { 0.0f, 1.0f, dt, 0.0f },
        { 0.0f, 0.0f, 1.0f, 0.0f },
        { 0.0f, 0.0f, 0.0f, 1.0f },
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
    const float qb = kf->Q_accelBias;
    const float processNoise[KF_STATE_COUNT][KF_STATE_COUNT] = {
        { q * dt5 / 20.0f, q * dt4 / 8.0f, q * dt3 / 6.0f, 0.0f },
        { q * dt4 / 8.0f, q * dt3 / 3.0f, q * dt2 / 2.0f, 0.0f },
        { q * dt3 / 6.0f, q * dt2 / 2.0f, q * dt, 0.0f },
        { 0.0f, 0.0f, 0.0f, qb * dt },
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

// Fuse an earth-frame accelerometer reading.
//
// Unlike position and velocity, the accelerometer does not observe a single state. It
// reports acceleration plus bias, so H = [0 0 1 1] and this cannot go through
// kalmanUpdateScalar, which assumes H selects one state. Only the sum of the two states
// is visible here; what separates them is the position and velocity aiding. A bias makes
// velocity drift, GPS contradicts that drift, and the resulting correction is pushed into
// the bias because acceleration alone cannot explain a persistent disagreement. This is
// why the bias converges within seconds while aiding is available and simply holds its
// last value when it is not.
//
// The reason for estimating it at all is dead reckoning. Velocity here is integrated from
// the accelerometer, so an uncorrected offset integrates without limit: 30 cm/s^2, about
// 1.8 degrees of attitude error, reaches roughly 10 m/s of velocity error over 15 seconds
// without GPS to pull it back. Removing the estimated bias first bounds that to a few
// tens of cm/s, which is what makes a GPS dropout survivable.
void kalmanUpdateAcceleration(positionKalman_t *kf, float measuredAccel, float R, float dt)
{
    // P*H^T and H*P. These are transposes of each other for a symmetric P, but the
    // covariance update below uses the non-Joseph form, which can let P drift slightly
    // out of symmetry, so both are read directly rather than one being reused.
    float PHt[KF_STATE_COUNT];
    float HP[KF_STATE_COUNT];
    for (unsigned i = 0; i < KF_STATE_COUNT; i++) {
        PHt[i] = kf->P[i][KF_ACCELERATION] + kf->P[i][KF_ACCEL_BIAS];
        HP[i] = kf->P[KF_ACCELERATION][i] + kf->P[KF_ACCEL_BIAS][i];
    }

    // Innovation variance H*P*H^T + R. Because H sums two states, this picks up their
    // cross-covariance as well as both variances.
    const float S = PHt[KF_ACCELERATION] + PHt[KF_ACCEL_BIAS] + R;
    if (S < 1e-9f) {
        return;
    }

    float gain[KF_STATE_COUNT];
    for (unsigned i = 0; i < KF_STATE_COUNT; i++) {
        gain[i] = PHt[i] / S;
    }

    // The prediction being tested is (acceleration + bias), not acceleration alone,
    // so a correctly learned bias produces no innovation and is left undisturbed.
    const float innovation = measuredAccel - (kf->x[KF_ACCELERATION] + kf->x[KF_ACCEL_BIAS]);
    const float debiasedAccel = measuredAccel - kf->x[KF_ACCEL_BIAS];
    const float velocityBeforeUpdate = kf->x[KF_VELOCITY];

    for (unsigned i = 0; i < KF_STATE_COUNT; i++) {
        kf->x[i] += gain[i] * innovation;
    }

    // Velocity is then overwritten to integrate the debiased reading directly, instead of
    // keeping the covariance-weighted share of the innovation it just received above.
    //
    // The purpose is to make the accelerometer's authority over velocity constant. That
    // share is gain[KF_VELOCITY], which is derived from the covariance and therefore moves
    // with GPS trust: measured across realistic conditions it ranged from 18% of a full dt
    // integration with GPS tightly fused, through 58% at normal settings, to 95% with GPS
    // absent. Because GPS trust tracks DOP, the accelerometer's contribution shifted while
    // flying and jumped when the fix dropped, so the estimator's dynamics were not stable
    // enough to tune a position controller against. Integrating directly fixes the
    // contribution at exactly dt regardless of covariance state.
    //
    // The cost is that P no longer describes how velocity was actually propagated. It stays
    // a valid covariance matrix and the GPS weighting it produces is unchanged, but velocity
    // variance is understated, so trustXY and trustZ derived from it read optimistically.
    kf->x[KF_VELOCITY] = velocityBeforeUpdate + debiasedAccel * dt;

    // P = P - K*H*P, applied to the full matrix so the acceleration/bias correlation and
    // its correlation with the motion states are both maintained.
    for (unsigned row = 0; row < KF_STATE_COUNT; row++) {
        for (unsigned column = 0; column < KF_STATE_COUNT; column++) {
            kf->P[row][column] -= gain[row] * HP[column];
        }
    }

    kf->x[KF_ACCEL_BIAS] = constrainf(kf->x[KF_ACCEL_BIAS], -ACCEL_BIAS_LIMIT, ACCEL_BIAS_LIMIT);
}
