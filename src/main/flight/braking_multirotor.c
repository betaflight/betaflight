/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform.h"

#if !defined(USE_WING) && defined(USE_ACC)

#include "common/axis.h"
#include "common/maths.h"
#include "common/vector.h"

#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"

#include "sensors/acceleration.h"
#include "sensors/gyro.h"

#include "braking_multirotor.h"

#define BRAKING_GRAVITY_MSS             9.80665f
#define BRAKING_STATE_TIMEOUT_US        50000
#define BRAKING_VELOCITY_LEAK_PER_SEC   4.8f
#define BRAKING_VELOCITY_SEED_PER_DEG   0.05f
#define BRAKING_VELOCITY_P_GAIN         95.0f
#define BRAKING_ACCEL_D_GAIN            6.0f
#define BRAKING_ANGLE_P_GAIN            16.0f
#define BRAKING_GYRO_D_GAIN             1.6f
#define BRAKING_MAX_VELOCITY_MS         9.0f
#define BRAKING_MAX_SETPOINT_DPS        580.0f

static bool brakingControllerActive = false;
static timeUs_t brakingPreviousUpdateUs = 0;
static vector2_t brakingVelocityEf = { { 0.0f, 0.0f } };
static float brakingSetpoint[RP_AXIS_COUNT] = { 0.0f, 0.0f };

static bool isBrakingControllerAllowed(void)
{
    return FLIGHT_MODE(BRAKING_MODE)
        && ARMING_FLAG(ARMED)
        && sensors(SENSOR_ACC)
        && !isFixedWing()
        && !failsafeIsActive();
}

static void brakingResetState(void)
{
    brakingControllerActive = false;
    brakingPreviousUpdateUs = 0;
    vector2Zero(&brakingVelocityEf);
    brakingSetpoint[FD_ROLL] = 0.0f;
    brakingSetpoint[FD_PITCH] = 0.0f;
}

bool isBrakingActive(void)
{
    return isBrakingControllerAllowed();
}

static void seedVelocityEstimateFromAttitude(void)
{
    const float bodyFrameRotation = DECIDEGREES_TO_RADIANS(attitude.values.yaw - 900);
    const vector2_t seedVelocityBf = {
        .x = attitude.values.pitch * 0.1f * BRAKING_VELOCITY_SEED_PER_DEG,
        .y = -attitude.values.roll * 0.1f * BRAKING_VELOCITY_SEED_PER_DEG,
    };

    vector2Rotate(&brakingVelocityEf, &seedVelocityBf, -bodyFrameRotation);
}

static void constrainVelocityEstimate(void)
{
    const float velocityNorm = vector2Norm(&brakingVelocityEf);
    if (velocityNorm > BRAKING_MAX_VELOCITY_MS && velocityNorm > 0.0f) {
        vector2Scale(&brakingVelocityEf, &brakingVelocityEf, BRAKING_MAX_VELOCITY_MS / velocityNorm);
    }
}

static vector2_t getHorizontalAccelEarthFrame(void)
{
    vector3_t accEarth = acc.accADC;
    matrixVectorMul(&accEarth, &rMat, &accEarth);
    vector3Scale(&accEarth, &accEarth, acc.dev.acc_1G_rec * BRAKING_GRAVITY_MSS);
    accEarth.z -= BRAKING_GRAVITY_MSS;

    return (vector2_t) { .x = accEarth.x, .y = accEarth.y };
}

static void updateVelocityEstimate(const vector2_t *accelEf, float dt)
{
    brakingVelocityEf.x += accelEf->x * dt;
    brakingVelocityEf.y += accelEf->y * dt;

    // Keep this estimate intentionally short-lived. The controller needs
    // aggressive reaction to recent horizontal motion, not a navigation-grade
    // long-horizon velocity state that drifts forever.
    const float velocityLeak = MAX(0.0f, 1.0f - BRAKING_VELOCITY_LEAK_PER_SEC * dt);
    vector2Scale(&brakingVelocityEf, &brakingVelocityEf, velocityLeak);
    constrainVelocityEstimate();
}

static void rotateHorizontalStateToBodyFrame(const vector2_t *velocityEf, const vector2_t *accelEf, vector2_t *velocityBf, vector2_t *accelBf)
{
    // attitude.values.yaw increases clockwise from north. Convert to the same
    // earth-to-body rotation convention used by Betaflight's multirotor autopilot.
    const float bodyFrameRotation = DECIDEGREES_TO_RADIANS(attitude.values.yaw - 900);

    vector2Rotate(velocityBf, velocityEf, bodyFrameRotation);
    vector2Rotate(accelBf, accelEf, bodyFrameRotation);
}

static float calculateBrakingAxisSetpoint(float estimatedVelocity, float measuredAcceleration, float attitudeDegrees, float gyroRate)
{
    return (-estimatedVelocity * BRAKING_VELOCITY_P_GAIN)
        + (-measuredAcceleration * BRAKING_ACCEL_D_GAIN)
        + (-attitudeDegrees * BRAKING_ANGLE_P_GAIN)
        + (-gyroRate * BRAKING_GYRO_D_GAIN);
}

void brakingUpdate(timeUs_t currentTimeUs)
{
    if (!isBrakingControllerAllowed()) {
        brakingResetState();
        return;
    }

    if (!brakingControllerActive) {
        brakingControllerActive = true;
        brakingPreviousUpdateUs = currentTimeUs;
        seedVelocityEstimateFromAttitude();
    }

    const timeDelta_t deltaTimeUs = cmpTimeUs(currentTimeUs, brakingPreviousUpdateUs);
    brakingPreviousUpdateUs = currentTimeUs;

    if ((deltaTimeUs <= 0) || (deltaTimeUs > BRAKING_STATE_TIMEOUT_US)) {
        seedVelocityEstimateFromAttitude();
        brakingSetpoint[FD_ROLL] = 0.0f;
        brakingSetpoint[FD_PITCH] = 0.0f;
        return;
    }

    const float dt = deltaTimeUs * 1e-6f;

    const vector2_t accelEf = getHorizontalAccelEarthFrame();
    vector2_t velocityBf;
    vector2_t accelBf;

    updateVelocityEstimate(&accelEf, dt);
    rotateHorizontalStateToBodyFrame(&brakingVelocityEf, &accelEf, &velocityBf, &accelBf);

    const float rollDegrees = attitude.values.roll * 0.1f;
    const float pitchDegrees = attitude.values.pitch * 0.1f;

    const float rollSetpoint = calculateBrakingAxisSetpoint(velocityBf.y, accelBf.y, rollDegrees, gyro.gyroADCf[FD_ROLL]);
    const float pitchSetpoint = calculateBrakingAxisSetpoint(velocityBf.x, accelBf.x, pitchDegrees, gyro.gyroADCf[FD_PITCH]);

    brakingSetpoint[FD_ROLL] = constrainf(rollSetpoint, -BRAKING_MAX_SETPOINT_DPS, BRAKING_MAX_SETPOINT_DPS);
    brakingSetpoint[FD_PITCH] = constrainf(pitchSetpoint, -BRAKING_MAX_SETPOINT_DPS, BRAKING_MAX_SETPOINT_DPS);
}

float getBrakingSetpoint(int axis)
{
    return axis < FD_YAW ? brakingSetpoint[axis] : 0.0f;
}

#endif
