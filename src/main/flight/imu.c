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

// Inertial Measurement Unit (IMU)

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "flight/gps_rescue.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "fc/rc.h"

#include "io/gps.h"

#include "scheduler/scheduler.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
#include <stdio.h>
#include <pthread.h>

static pthread_mutex_t imuUpdateLock;

#if defined(SIMULATOR_IMU_SYNC)
static uint32_t imuDeltaT = 0;
static bool imuUpdated = false;
#endif

#define IMU_LOCK pthread_mutex_lock(&imuUpdateLock)
#define IMU_UNLOCK pthread_mutex_unlock(&imuUpdateLock)

#else

#define IMU_LOCK
#define IMU_UNLOCK

#endif

// the limit (in degrees/second) beyond which we stop integrating
// omega_I. At larger spin rates the DCM PI controller can get 'dizzy'
// which results in false gyro drift. See
// https://drive.google.com/file/d/0ByvTkVQo3tqXQUVCVUNyZEgtRGs/view?usp=sharing&resourcekey=0-Mo4254cxdWWx2Y4mGN78Zw

#define SPIN_RATE_LIMIT 20

#define GPS_COG_MIN_GROUNDSPEED 100        // 1.0m/s - min groundspeed for GPS Heading reinitialisation etc

bool canUseGPSHeading = true;

static imuRuntimeConfig_t imuRuntimeConfig;

static matrix33_t rMat;

#if defined(UNIT_TEST)
void imuSetRotMatrix(const matrix33_t* inMatrix) { rMat = *inMatrix; }
#endif

#if defined(USE_ACC)
// Do not use accelerometer if the norm of the reading differs more than this from 1g [g]
static const float IMU_ACC_COVARIANCE_CALC_ACC_NORM_LIMIT = 0.2f;
// Do not use accelerometer if the gyro norm is greater than this [deg/s]
static const float IMU_ACC_COVARIANCE_CALC_GYRO_NORM_LIMIT = 50.0f;
// How fast the gyro covaraince will increase with higher rates
static const float IMU_GYRO_COVARIANCE_CALC_RATE_SCALING = 1.0f / 10.0f;
// 500 is the guestimated gyro drift in deg/s when the gyro is saturated
static const float IMU_GYRO_PSD_SATURATED = sq(DEGREES_TO_RADIANS(500.0f));

STATIC_UNIT_TESTED bool attitudeIsEstablished = false;
static const float IMU_ESTIMATE_COVARIANCE_MAXIMUM = sq(DEGREES_TO_RADIANS(180.0f));
#endif

// quaternion of sensor frame relative to earth frame
static imuAhrsState_t ahrsState = {
    .nominal = { .attitude = QUATERNION_INITIALIZE, .integralErr = {.x = 0.0f, .y = 0.0f, .z = 0.0f}},
    .error = { .rp = {.x = 0.0f, .y = 0.0f, .z = 0.0f}, .heading = 0.0f },
    .rpEstimateCovariance = IMU_ESTIMATE_COVARIANCE_MAXIMUM,
    .rpCovariance = 0.0f,
    .headingGain = 0.0f
};

STATIC_UNIT_TESTED void imuResetAhrsState(imuAhrsState_t* state)
{
    imuAhrsState_t intState = {
        .nominal = { .attitude = QUATERNION_INITIALIZE, .integralErr = { .x = 0.0f, .y = 0.0f, .z = 0.0f } },
        .error = { .rp = {.x = 0.0f, .y = 0.0f, .z = 0.0f}, .heading = 0.0f },
        .rpEstimateCovariance = IMU_ESTIMATE_COVARIANCE_MAXIMUM,
        .rpCovariance = 0.0f,
        .headingGain = 0.0f
    };
    *state = intState;
}

#if defined(UNIT_TEST)
void imuSetQuaternion(const quaternion* inQuat) { ahrsState.nominal.attitude = *inQuat; }
#endif

// headfree quaternions
static quaternion headfree = QUATERNION_INITIALIZE;
static quaternion offset = QUATERNION_INITIALIZE;

// absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
attitudeEulerAngles_t attitude = EULER_INITIALIZE;

PG_REGISTER_WITH_RESET_TEMPLATE(imuConfig_t, imuConfig, PG_IMU_CONFIG, 3);

#ifdef USE_RACE_PRO
#define DEFAULT_SMALL_ANGLE 180
#else
#define DEFAULT_SMALL_ANGLE 25
#endif

PG_RESET_TEMPLATE(imuConfig_t, imuConfig,
    .imu_dcm_kp = 2500,      // 1.0 * 10000
    .imu_dcm_ki = 0,         // 0.003 * 10000
    .small_angle = DEFAULT_SMALL_ANGLE,
    .imu_process_denom = 2,
    .mag_declination = 0,
    .gyro_noise_asd = 5,           // 0.5 (deg/s)/sqrt(s)
    .acc_noise_std = 50,           // 5.0 deg
);

static void imuResetEstimateCovariance(imuAhrsState_t* ahrsState)
{
    ahrsState->rpEstimateCovariance = IMU_ESTIMATE_COVARIANCE_MAXIMUM;
}

static void imuQuaternionComputeProducts(quaternionProducts *quatProd, const quaternion *quat)
{
    quatProd->ww = quat->w * quat->w;
    quatProd->wx = quat->w * quat->x;
    quatProd->wy = quat->w * quat->y;
    quatProd->wz = quat->w * quat->z;
    quatProd->xx = quat->x * quat->x;
    quatProd->xy = quat->x * quat->y;
    quatProd->xz = quat->x * quat->z;
    quatProd->yy = quat->y * quat->y;
    quatProd->yz = quat->y * quat->z;
    quatProd->zz = quat->z * quat->z;
}

STATIC_UNIT_TESTED void imuComputeRotationMatrix(matrix33_t* outMat, const quaternion* q)
{
    quaternionProducts qP;
    imuQuaternionComputeProducts(&qP, q);

    outMat->m[0][0] = 1.0f - 2.0f * qP.yy - 2.0f * qP.zz;
    outMat->m[0][1] = 2.0f * (qP.xy + -qP.wz);
    outMat->m[0][2] = 2.0f * (qP.xz - -qP.wy);

    outMat->m[1][0] = 2.0f * (qP.xy - -qP.wz);
    outMat->m[1][1] = 1.0f - 2.0f * qP.xx - 2.0f * qP.zz;
    outMat->m[1][2] = 2.0f * (qP.yz + -qP.wx);

    outMat->m[2][0] = 2.0f * (qP.xz + -qP.wy);
    outMat->m[2][1] = 2.0f * (qP.yz - -qP.wx);
    outMat->m[2][2] = 1.0f - 2.0f * qP.xx - 2.0f * qP.yy;

#if defined(SIMULATOR_BUILD) && !defined(USE_IMU_CALC) && !defined(SET_IMU_FROM_EULER)
    outMat->m[1][0] = -2.0f * (qP.xy - -qP.wz);
    outMat->m[2][0] = -2.0f * (qP.xz + -qP.wy);
#endif
}

static float calculateThrottleAngleScale(uint16_t throttle_correction_angle)
{
    return (1800.0f / M_PIf) * (900.0f / throttle_correction_angle);
}

void imuConfigure(uint16_t throttle_correction_angle, uint8_t throttle_correction_value)
{
    // current default for imu_dcm_kp is 2500; our 'normal' or baseline value for imuDcmKp is 0.25
    imuRuntimeConfig.imuDcmKp = imuConfig()->imu_dcm_kp / 10000.0f;
    imuRuntimeConfig.imuDcmKi = imuConfig()->imu_dcm_ki / 10000.0f;
    // magnetic declination has negative sign (positive clockwise when seen from top)
    const float imuMagneticDeclinationRad = DEGREES_TO_RADIANS(imuConfig()->mag_declination / 10.0f);
    imuRuntimeConfig.north_ef.x = cos_approx(imuMagneticDeclinationRad);
    imuRuntimeConfig.north_ef.y = -sin_approx(imuMagneticDeclinationRad);

    imuRuntimeConfig.gyroNoisePsd = sq(DEGREES_TO_RADIANS(imuConfig()->gyro_noise_asd * 0.1f));
    imuRuntimeConfig.accCovariance = sq(DEGREES_TO_RADIANS(imuConfig()->acc_noise_std * 0.1f));

    imuResetAhrsState(&ahrsState);

    imuRuntimeConfig.smallAngleCosZ = cos_approx(degreesToRadians(imuConfig()->small_angle));

    imuRuntimeConfig.throttleAngleScale = calculateThrottleAngleScale(throttle_correction_angle);

    imuRuntimeConfig.throttleAngleValue = throttle_correction_value;
}

void imuInit(void)
{
#ifdef USE_GPS
    canUseGPSHeading = true;
#else
    canUseGPSHeading = false;
#endif

    imuComputeRotationMatrix(&rMat, &ahrsState.nominal.attitude);

#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
    if (pthread_mutex_init(&imuUpdateLock, NULL) != 0) {
        printf("Create imuUpdateLock error!\n");
    }
#endif
}

#if defined(USE_ACC)
static float invSqrt(float x)
{
    return 1.0f / sqrtf(x);
}

static void imuResetErrorState(imuAhrsState_t* state)
{
    vector3Zero(&state->error.rp);
    state->error.heading = 0.0;
    state->rpCovariance = 0.0f;
    state->headingGain = 0.0f;
}


// Increase Roll/Pitch estimate covariance based on time spent integrating
// Update the estimate covariance according to P_k = (1 - K_k) * P_k-1 + w(t)
// w(t) is the gyro noise and is assumed to be normaldistributed with zero mean.
// A much higher covariance is used when the gyro is saturated, but the noise is still modeled
// as being normal distributed.
// estimateCovariance covariance of the estimate
// accGain kalman gain for the accelerometer
// imuDt total time delta since last ahrs update
// durationSaturated duration that the gyro has been saturated since last update
// gyroCovariance covariance of the gyro under normal (non saturated) circumstances
static void imuUpdateRPEstimateCovariance(imuAhrsState_t* state, const float imuDt, float durationSaturated, const float gyroCovariance, const float accGain)
{
    if (durationSaturated > imuDt) { durationSaturated = imuDt; }
    const float normalDuration = imuDt - durationSaturated;
    const float accumulatedCovariance = gyroCovariance * normalDuration + IMU_GYRO_PSD_SATURATED * durationSaturated;
    const float updatedCovariance = (1.0f - accGain) * (state->rpEstimateCovariance) + accumulatedCovariance;
    state->rpEstimateCovariance = constrainf(updatedCovariance, 0.0f, IMU_ESTIMATE_COVARIANCE_MAXIMUM);
}


// Calculate gyro power spectrum density
static float imuCalcGyroPsd(const float basePsd, const float gyroNorm)
{
    return basePsd + basePsd * RADIANS_TO_DEGREES(gyroNorm) * IMU_GYRO_COVARIANCE_CALC_RATE_SCALING;
}

/// Approximate the accelerometer covariance based on  the accelerometer vector norm
/// and gyro rates.
/// will return 0.0 if the measurement is considered unusable
/// @arg baseAccCovariance best case scenario accelerometer covariance
/// @arg accNorm norm of the accelerometer vector in g
/// @arg gyroNorm norm of the gyrp rate vector in degrees per second
static float imuAccCovariance(const float baseAccCovariance, const float accNorm, const float gyroNorm)
{   // return 0 if the norm of the accelerometer vector differs more than this from 1.0g (ca 9.8 m/s)
    const float accLimit = IMU_ACC_COVARIANCE_CALC_ACC_NORM_LIMIT;
    // [deg/s] return 0 if the norm of the gyro rates are above this value
    const float gyroLimit = IMU_ACC_COVARIANCE_CALC_GYRO_NORM_LIMIT;
    // Use acceleromter, but increase the covariance by how much the
    // gyro and acc vector norms differs from the ideal
    const float accTrust = tent(accNorm - 1.0f, accLimit) * tent(RADIANS_TO_DEGREES(gyroNorm), gyroLimit);

    const float epsilon = 0.01f;
    return accTrust > epsilon ? baseAccCovariance / accTrust : 0.0f;
}

// Calculate Kalman gain
static float imuCalcKalmanGain(const float estimateCovariance, const float measurementCovariance)
{
    const float inovationCovariance = estimateCovariance + measurementCovariance;
    if (inovationCovariance > 0.0f && measurementCovariance > 0.0f) {
        return estimateCovariance / inovationCovariance;
    } else {
        return 0.0f;
    }
}

static bool imuIsMahalanobisOutlier(const float estimateCovariance, const float measurementCovariance, const float diff, const float treshold)
{
    const float inovationCovariance = estimateCovariance + measurementCovariance;
    return (sq(diff) >= treshold * inovationCovariance) && treshold != 0.0f;
}

STATIC_UNIT_TESTED void imuCalcAccError(imuAhrsState_t* state, const imuRuntimeConfig_t* config, const float dt, const vector3_t* gyro, vector3_t acc_bf, const matrix33_t* rotMat)
{
    UNUSED(dt);

    const float gyroNorm = vector3Norm(gyro);

    // Use measured acceleration vector
    const float accNorm = vector3Norm(&acc_bf);
    vector3Normalize(&acc_bf, &acc_bf);

    // estimated roll pitch vector in body frame
    const vector3_t estRP_bf = {.x = rotMat->m[2][0], .y = rotMat->m[2][1], .z = rotMat->m[2][2]};

    const float accAngleError = acos_approx(vector3Dot(&acc_bf, &estRP_bf));

    // chech if the acc measurement should be considered an outlier
#if defined(USE_GPS_RESCUE)
    const bool gpsRescueActive = FLIGHT_MODE(GPS_RESCUE_MODE);
#else
    const bool gpsRescueActive = false;
#endif
    const float outlierThreshold = gpsRescueActive || !(ARMING_FLAG(ARMED)) ? 0.0f : 2.0f;
    const bool notOutlier = !imuIsMahalanobisOutlier(state->rpEstimateCovariance, config->accCovariance, accAngleError, outlierThreshold);

    const float accCovariance = notOutlier ? imuAccCovariance(config->accCovariance, accNorm, gyroNorm) : 0.0f;

    if (accCovariance > 0.0f) {
        // Difference is the cross product between estimated direction and measured direction of gravity
        vector3_t accDiff;
        vector3Cross(&accDiff, &acc_bf, &estRP_bf);
        const float accDiffNormSq = vector3NormSq(&accDiff);
        if (accDiffNormSq > 0.001f) {  // scale larger errors with the actual magnitude of the angle
            vector3Scale(&accDiff, &accDiff, accAngleError / sqrtf(accDiffNormSq));
        }
        vector3Add(&(state->error.rp), &(state->error.rp), &accDiff);
        state->rpCovariance = accCovariance;
        DEBUG_SET(DEBUG_IMU_GAIN, 2, lrintf(RADIANS_TO_DEGREES(accAngleError) * 10.0f));
    } else {
        DEBUG_SET(DEBUG_IMU_GAIN, 2, 0);
    }
    DEBUG_SET(DEBUG_IMU_GAIN, 3, lrintf(10.0f * RADIANS_TO_DEGREES(sqrtf(accCovariance))));
    DEBUG_SET(DEBUG_IMU_GAIN, 4, lrintf(accNorm * 100.0f));
    DEBUG_SET(DEBUG_IMU_GAIN, 5, lrintf(RADIANS_TO_DEGREES(gyroNorm)));
    DEBUG_SET(DEBUG_IMU_GAIN, 7, lrintf(RADIANS_TO_DEGREES(accAngleError) * 10.0f));
}


// Fuse the error state into the nominal state and integrate the attitude from the gyro rates
// state - state of the AHRS filter
// conifg - imu runtime configurations
// dt - time since last update [s]
// gyro - gyro reading, [rad/s]
// durationSaturated - duration that the gyro has been saturated since last update [s]
// rotMat - rotation matrix 
STATIC_UNIT_TESTED void imuAhrsUpdate(imuAhrsState_t* state,
                                const imuRuntimeConfig_t* config, const float dt,
                                const vector3_t* gyro,
                                const float durationSaturated,
                                const matrix33_t* rotMat)
{
    // Calculate general spin rate (rad/s)
    const float gyroRate = vector3Norm(gyro);

    // Heading error
    vector3_t headingError = { .x = rotMat->m[Z][X], .y = rotMat->m[Z][Y], .z = rotMat->m[Z][Z] };
    vector3Scale(&headingError, &headingError, state->error.heading);
    vector3_t headingChange;
    vector3Scale(&headingChange, &headingError, state->headingGain);

    // Roll/Pitch error
    // limit rp gain to avoid converging to fast at large errors and high estimate covariance
    // large errors and gains will converge slower than expected due to approximatons in the integration scheme
    const float rpGain = constrainf(imuCalcKalmanGain(state->rpEstimateCovariance, state->rpCovariance), 0.0f, 0.5f);
    DEBUG_SET(DEBUG_IMU_GAIN, 0, lrintf(1000.0f * rpGain));
    const float gyroPsd = imuCalcGyroPsd(config->gyroNoisePsd, gyroRate);
    imuUpdateRPEstimateCovariance(state, dt, durationSaturated, gyroPsd, rpGain);
    DEBUG_SET(DEBUG_IMU_GAIN, 1, lrintf(10.0f * RADIANS_TO_DEGREES(sqrtf(state->rpEstimateCovariance))));

    vector3_t rpChange = state->error.rp;
    vector3Scale(&rpChange, &rpChange, rpGain);

    vector3_t totalChange;
    vector3Add(&totalChange, &rpChange, &headingChange);

    // Compute and apply integral feedback if enabled
    if (config->imuDcmKi > 0.0f) {
        // Stop integrating if spinning beyond the certain limit
        if (gyroRate < DEGREES_TO_RADIANS(SPIN_RATE_LIMIT)) {
            vector3_t iChange;
            vector3Add(&iChange, &state->error.rp, &headingError);
            vector3Scale(&iChange, &iChange, config->imuDcmKi * dt);  // integral error scaled by Ki
            vector3Add(&state->nominal.integralErr, &state->nominal.integralErr, &iChange);
        }
    } else {
        vector3Zero(&state->nominal.integralErr);  // prevent integral windup
    }

    // Add errors and integrate rate of change of quaternion
    vector3_t g;
    vector3Add(&g, gyro, &state->nominal.integralErr);
    vector3Scale(&g, &g, dt);
    vector3Add(&g, &g, &totalChange);

    vector3Scale(&g, &g, 0.5f);

    const quaternion buffer = state->nominal.attitude;

    quaternion* q = &state->nominal.attitude;

    q->w += (-buffer.x * g.x - buffer.y * g.y - buffer.z * g.z);
    q->x += (+buffer.w * g.x + buffer.y * g.z - buffer.z * g.y);
    q->y += (+buffer.w * g.y - buffer.x * g.z + buffer.z * g.x);
    q->z += (+buffer.w * g.z + buffer.x * g.y - buffer.y * g.x);

    // Normalise quaternion
    float recipNorm = invSqrt(sq(q->w) + sq(q->x) + sq(q->y) + sq(q->z));
    q->w *= recipNorm;
    q->x *= recipNorm;
    q->y *= recipNorm;
    q->z *= recipNorm;

    imuResetErrorState(state);

    attitudeIsEstablished = true;
}

STATIC_UNIT_TESTED void imuUpdateEulerAngles(const matrix33_t* rotMat)
{

    if (FLIGHT_MODE(HEADFREE_MODE)) {
        quaternionProducts buffer;
        imuQuaternionComputeProducts(&buffer, &headfree);

        attitude.values.roll = lrintf(atan2_approx((+2.0f * (buffer.wx + buffer.yz)), (+1.0f - 2.0f * (buffer.xx + buffer.yy))) * (1800.0f / M_PIf));
        attitude.values.pitch = lrintf(((0.5f * M_PIf) - acos_approx(+2.0f * (buffer.wy - buffer.xz))) * (1800.0f / M_PIf));
        attitude.values.yaw = lrintf((-atan2_approx((+2.0f * (buffer.wz + buffer.xy)), (+1.0f - 2.0f * (buffer.yy + buffer.zz))) * (1800.0f / M_PIf)));
    } else {
        attitude.values.roll = lrintf(atan2_approx(rotMat->m[2][1], rotMat->m[2][2]) * (1800.0f / M_PIf));
        attitude.values.pitch = lrintf(((0.5f * M_PIf) - acos_approx(-rotMat->m[2][0])) * (1800.0f / M_PIf));
        attitude.values.yaw = lrintf((-atan2_approx(rotMat->m[1][0], rotMat->m[0][0]) * (1800.0f / M_PIf)));
    }

    if (attitude.values.yaw < 0) {
        attitude.values.yaw += 3600;
    }
}

#ifdef USE_GPS

// IMU groundspeed gain heuristic.
static float imuCalcGroundspeedGain(float* stickSuppression, float dt)
{
    float groundSpeedGain;
    if (FLIGHT_MODE(GPS_RESCUE_MODE)) {
        // GPS_Rescue adjusts groundspeedGain during a rescue in a range 0 - 4.5,
        //   depending on GPS Rescue state and groundspeed relative to speed to home.
        groundSpeedGain = gpsRescueGetImuYawCogGain();
    } else {
        // 0.0 - 10.0, heuristic based on GPS speed and stick state
        // in normal flight, IMU should:
        // - heavily average GPS heading values at low speed, since they are random, almost
        // - respond more quickly at higher speeds.
        // GPS typically returns quite good heading estimates at or above 0.5- 1.0 m/s, quite solid by 2m/s
        // groundSpeedGain will be 0 at 0.0m/s, rising slowly towards 1.0 at 1.0 m/s, and reaching max of 10.0 at 10m/s
        const float speedRatio = (float)gpsSol.groundSpeed / GPS_COG_MIN_GROUNDSPEED;
        float speedBasedGain = speedRatio > 1.0f ? fminf(speedRatio, 10.0f) : sq(speedRatio);

        const bool isWing = isFixedWing();  // different weighting for airplane aerodynamic

        // 2. suppress heading correction during and after yaw inputs, down to zero at 100% yaw
        const float yawStickDeflectionInv = 1.0f - getRcDeflectionAbs(FD_YAW);
        float stickDeflectionFactor = power5(yawStickDeflectionInv);
        // negative peak detector with decay over a 2.5s time constant, to sustain the suppression
        const float k = 0.4f * dt; // k = 0.004 at 100Hz, dt in seconds, 2.5s time constant
        const float stickSuppressionNew = *stickSuppression + k * (stickDeflectionFactor - *stickSuppression);
        *stickSuppression = fminf(stickSuppressionNew, stickDeflectionFactor);

        // 3. suppress heading correction unless roll is centered, from 1.0 to zero if Roll is more than 12 degrees from flat
        // this is to prevent adaptation to GPS while flying sideways, or with a significant sideways element
        const float absRollAngle = fabsf(attitude.values.roll * .1f);  // degrees
        float rollMax = isWing ? 25.0f : 12.0f; // 25 degrees for wing, 12 degrees for quad
        // note: these value are 'educated guesses' - for quads it must be very tight
        // for wings, which can't fly sideways, it can be wider
        const float rollSuppression = (absRollAngle < rollMax) ? (rollMax - absRollAngle) / rollMax : 0.0f;

        // 4. attenuate heading correction by pitch angle, will be zero if flat or negative (ie flying tail first)
        // allow faster adaptation for quads at higher pitch angles; returns 1.0 at 45 degrees
        // but not if a wing, because they typically are flat when flying.
        // need to test if anything special is needed for pitch with wings, for now do nothing.
        float pitchSuppression = 1.0f;
        if (!isWing) {
            const float pitchAngle = attitude.values.pitch * .1f; // degrees, negative is backwards
            pitchSuppression = pitchAngle / 45.0f; // 1.0 at 45 degrees, 2.0 at 90 degrees
            pitchSuppression = (pitchSuppression >= 0) ? pitchSuppression : 0.0f; // zero if flat or pitched backwards
        }

        // NOTE : these suppressions make sense with normal pilot inputs and normal flight
        // They are not used in GPS Rescue, and probably should be bypassed in position hold, etc,

        groundSpeedGain = speedBasedGain * (*stickSuppression) * rollSuppression * pitchSuppression;
    }
    DEBUG_SET(DEBUG_ATTITUDE, 2, lrintf(groundSpeedGain * 100.0f));
    return groundSpeedGain;
}

// *** Calculate heading error derived from IMU heading vs GPS heading ***
// assumes that quad/plane is flying forward (gain factors attenuate situations when this is not true)
// courseOverGround - in rad, 0 = north, clockwise
// return value rotation around earth Z axis, pointing in directipon of smaller error, [rad/s]
STATIC_UNIT_TESTED void imuCalcCourseErr(imuAhrsState_t* state, const imuRuntimeConfig_t* config, const float dt, float courseOverGround, const matrix33_t* rotMat)
{
    // Compute COG heading unit vector in earth frame (ef) from scalar GPS CourseOverGround
    // Earth frame X is pointing north and sin/cos argument is anticlockwise. (|cog_ef| == 1.0)
    const vector2_t cog_ef = {.x = cos_approx(-courseOverGround), .y = sin_approx(-courseOverGround)};

    // Compute and normalise craft Earth frame heading vector from body X axis
    vector2_t heading_ef = {.x = rotMat->m[X][X], .y = rotMat->m[Y][X]};
    vector2Normalize(&heading_ef, &heading_ef); // XY only, normalised to magnitude 1.0

    // cross (vector product) = |heading| * |cog| * sin(angle) = 1 * 1 * sin(angle)
    // cross value reflects error angle of quad X axis vs GPS groundspeed
    // cross sign depends on the rotation change from input to output vectors by right hand rule
    // operand order is arranged such that rotation is in direction of zero error
    // abs value of cross is zero when parallel, max of 1.0 at 90 degree error
    // cross value is used for ez_ef when error is less than 90 degrees
    // decreasing cross value after 90 degrees, and zero cross value at 180 degree error,
    //   would prevent error correction, so the maximum angular error value of 1.0 is used for this interval
    const float cross = vector2Cross(&heading_ef, &cog_ef);

    // dot product = |heading| * |cog| * cos(angle) = 1 * 1 * cos(angle)
    // max when aligned, zero at 90 degrees of vector difference, negative for error > 90 degrees
    const float dot = vector2Dot(&heading_ef, &cog_ef);

    // error around Z axis in Earth frame
    // for error angles less than 90 degrees (dot > 0), use cross value to compute ez_ef
    //   over 90 deg, use sign(cross)
    // when well aligned, return ~ 0 (sin(error)), for error >= 90 degrees, return +-1.0
    const float scalarErr = ((dot > 0) ? cross : (cross < 0 ? -1.0f : 1.0f) );
    state->headingGain = config->imuDcmKp * dt;
    DEBUG_SET(DEBUG_ATTITUDE, 3, (scalarErr * 100));
    state->error.heading = scalarErr;
}

#endif

#if defined(USE_MAG) && defined(USE_GPS_RESCUE)
// refactored from old debug code, to be removed/optimized eventually
static void imuDebug_GPS_RESCUE_HEADING(void)
{
    // Encapsulate additional operations in a block so that it is only executed when the according debug mode is used
    // Only re-calculate magYaw when there is a new Mag data reading, to avoid spikes
    if (debugMode == DEBUG_GPS_RESCUE_HEADING && mag.isNewMagADCFlag) {
        
        vector3_t mag_bf = mag.magADC;
        vector3_t mag_ef;
        matrixVectorMul(&mag_ef, &rMat, &mag_bf); // BF->EF true north

        matrix33_t rMatZTrans;
        yawToRotationMatrixZ(&rMatZTrans, -atan2_approx(rMat.m[1][0], rMat.m[0][0]));

        vector3_t mag_ef_yawed;
        matrixVectorMul(&mag_ef_yawed, &rMatZTrans, &mag_ef); // EF->EF yawed
        
        // Magnetic yaw is the angle between true north and the X axis of the body frame
        int16_t magYaw = lrintf((atan2_approx(mag_ef_yawed.y, mag_ef_yawed.x) * (1800.0f / M_PIf)));
        if (magYaw < 0) {
            magYaw += 3600;
        }
        DEBUG_SET(DEBUG_GPS_RESCUE_HEADING, 4, magYaw); // mag heading in degrees * 10
        // reset new mag data flag to false to initiate monitoring for new Mag data.
        // note that if the debug doesn't run, this reset will not occur, and we won't waste cycles on the comparison
        mag.isNewMagADCFlag = false;
    }
}
#endif // defined(USE_MAG) && defined(USE_GPS_RESCUE)

#ifdef USE_MAG
// Calculate heading error derived from magnetometer
// return value rotation around earth Z axis, pointing in directipon of smaller error, [rad/s]
STATIC_UNIT_TESTED void imuCalcMagErr(imuAhrsState_t* state, const imuRuntimeConfig_t* config, const float dt, const vector3_t* mag_bf, const matrix33_t* rotMat)
{
    // Use measured magnetic field vector
    float magNormSquared = vector3NormSq(mag_bf);

    if (magNormSquared > 0.01f) {
        // project magnetometer reading into Earth frame
        vector3_t mag_ef;
        matrixVectorMul(&mag_ef, rotMat, mag_bf); // BF->EF true north
        // Normalise magnetometer measurement
        vector3Scale(&mag_ef, &mag_ef, 1.0f / sqrtf(magNormSquared));

        // For magnetometer correction we make an assumption that magnetic field is perpendicular to gravity (ignore Z-component in EF).
        // This way magnetic field will only affect heading and wont mess roll/pitch angles
        vector2_t mag2d_ef = {.x = mag_ef.x, .y = mag_ef.y};
        // mag2d_ef - measured mag field vector in EF (2D ground plane projection)

        // magnetometer error is cross product between estimated magnetic north and measured magnetic north (calculated in EF)
        // increase gain on large misalignment
        const float dot = vector2Dot(&mag2d_ef, &config->north_ef);
        const float cross = vector2Cross(&mag2d_ef, &config->north_ef);
        state->error.heading = (dot > 0) ? cross : (cross < 0 ? -1.0f : 1.0f) * vector2Norm(&mag2d_ef);
        state->headingGain = config->imuDcmKp * dt;
    }
}

#endif

#if defined(USE_GPS)
static void imuComputeQuaternionFromRPY(quaternion* q, int16_t initialRoll, int16_t initialPitch, int16_t initialYaw)
{
    if (initialRoll > 1800) {
        initialRoll -= 3600;
    }

    if (initialPitch > 1800) {
        initialPitch -= 3600;
    }

    if (initialYaw > 1800) {
        initialYaw -= 3600;
    }

    const float cosRoll = cos_approx(DECIDEGREES_TO_RADIANS(initialRoll) * 0.5f);
    const float sinRoll = sin_approx(DECIDEGREES_TO_RADIANS(initialRoll) * 0.5f);

    const float cosPitch = cos_approx(DECIDEGREES_TO_RADIANS(initialPitch) * 0.5f);
    const float sinPitch = sin_approx(DECIDEGREES_TO_RADIANS(initialPitch) * 0.5f);

    const float cosYaw = cos_approx(DECIDEGREES_TO_RADIANS(-initialYaw) * 0.5f);
    const float sinYaw = sin_approx(DECIDEGREES_TO_RADIANS(-initialYaw) * 0.5f);

    q->w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
    q->x = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
    q->y = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
    q->z = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
}
#endif

#if defined(SIMULATOR_BUILD) && !defined(USE_IMU_CALC)
static void imuCalculateEstimatedAttitude(timeUs_t currentTimeUs)
{
    // unused static functions
    UNUSED(imuAhrsUpdate);
    UNUSED(canUseGPSHeading);
    UNUSED(imuCalcAccError);
    UNUSED(imuCalcMagErr);
    UNUSED(imuCalcKalmanGain);
    UNUSED(imuUpdateRPEstimateCovariance);
    UNUSED(imuIsMahalanobisOutlier);
    UNUSED(IMU_ACC_COVARIANCE_CALC_ACC_NORM_LIMIT);
    UNUSED(IMU_ACC_COVARIANCE_CALC_GYRO_NORM_LIMIT);
    UNUSED(IMU_GYRO_COVARIANCE_CALC_RATE_SCALING);
    UNUSED(IMU_ESTIMATE_COVARIANCE_MAXIMUM);
    UNUSED(imuCalcGyroPsd);

    UNUSED(currentTimeUs);
}
#else

static void imuCalculateEstimatedAttitude(timeUs_t currentTimeUs)
{
#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_IMU_SYNC)
    // Simulator-based timing
    //  printf("[imu]deltaT = %u, imuDeltaT = %u, currentTimeUs = %u, micros64_real = %lu\n", deltaT, imuDeltaT, currentTimeUs, micros64_real());
    const timeDelta_t deltaT = imuDeltaT;
#else
    static timeUs_t previousIMUUpdateTime = 0;
    static float stickSuppression = 0.0f;
    const timeDelta_t deltaT = currentTimeUs - previousIMUUpdateTime;
    previousIMUUpdateTime = currentTimeUs;
    if (deltaT > 100000) { // do not update attitude if the time delta is over 0.1s, to prevent weirdnes at startup
#if defined(USE_ACC)
        imuResetEstimateCovariance(&ahrsState);
#endif
        return;
    }
    DEBUG_SET(DEBUG_IMU_GAIN, 6, deltaT);

    const float dt = deltaT * 1e-6f;

    // *** magnetometer based error estimate ***
    bool useMag = false;   // mag will suppress GPS correction

    // TODO Only use mag and COG if rp attitude has been established
#ifdef USE_MAG
    if (sensors(SENSOR_MAG)
        && compassIsHealthy()
#ifdef USE_GPS_RESCUE
        && !gpsRescueDisableMag()
#endif
        ) {
        useMag = true;
        imuCalcMagErr(&ahrsState, &imuRuntimeConfig, dt, &mag.magADC, &rMat);
    }
#endif

#if defined(USE_MAG) && defined(USE_GPS_RESCUE)
    // fill in GPS rescue debug value (leftover from code refactoring)
    imuDebug_GPS_RESCUE_HEADING();
#endif

#if defined(USE_GPS)
    if (!useMag
        && sensors(SENSOR_GPS)
        && STATE(GPS_FIX) && gpsSol.numSat > GPS_MIN_SAT_COUNT) {
        static bool gpsHeadingInitialized = false;  // TODO - remove
        if (gpsHeadingInitialized) {
            imuCalcCourseErr(&ahrsState, &imuRuntimeConfig, dt, gpsSol.groundSpeed, &rMat);
            ahrsState.headingGain *= imuCalcGroundspeedGain(&stickSuppression, dt);
        } else if (gpsSol.groundSpeed > GPS_COG_MIN_GROUNDSPEED) {
            // Reset the reference and reinitialize quaternion factors when GPS groundspeed > GPS_COG_MIN_GROUNDSPEED
            imuComputeQuaternionFromRPY(&ahrsState.nominal.attitude, attitude.values.roll, attitude.values.pitch, gpsSol.groundCourse);
            imuComputeRotationMatrix(&rMat, &ahrsState.nominal.attitude);
            imuUpdateEulerAngles(&rMat);
            gpsHeadingInitialized = true;
        }
    }
#else
    UNUSED(useMag);
#endif

    vector3_t acc_bf;
    vector3Scale(&acc_bf, &acc.accADC, acc.dev.acc_1G_rec);

    const vector3_t gyroAverage = {
        .x = DEGREES_TO_RADIANS(gyroGetFilteredDownsampled(X)),
        .y = DEGREES_TO_RADIANS(gyroGetFilteredDownsampled(Y)),
        .z = DEGREES_TO_RADIANS(gyroGetFilteredDownsampled(Z)),
    };

    imuCalcAccError(&ahrsState, &imuRuntimeConfig, dt, &gyroAverage, acc_bf, &rMat);

    imuAhrsUpdate(&ahrsState, &imuRuntimeConfig, dt,
                  &gyroAverage,
                  gyroGetDurationSpentSaturated(), &rMat);

    // Pre-compute rotation matrix from quaternion
    imuComputeRotationMatrix(&rMat, &ahrsState.nominal.attitude);
    imuUpdateEulerAngles(&rMat);

    attitudeIsEstablished = true;  // TODO conditional on RP covariance

#endif
}

#endif

static int calculateThrottleAngleCorrection(const imuRuntimeConfig_t* config)
{
    /*
    * Use 0 as the throttle angle correction if we are inverted, vertical or with a
    * small angle < 0.86 deg
    * TODO: Define this small angle in config.
    */
    if (getCosTiltAngle() <= 0.015f) {
        return 0;
    }
    int angle = lrintf(acos_approx(getCosTiltAngle()) * config->throttleAngleScale);
    if (angle > 900)
        angle = 900;
    return lrintf(config->throttleAngleValue * sin_approx(angle / (900.0f * M_PIf / 2.0f)));
}

void imuUpdateAttitude(timeUs_t currentTimeUs)
{
    if (sensors(SENSOR_ACC) && acc.isAccelUpdatedAtLeastOnce && gyroIsCalibrationComplete()) {
        IMU_LOCK;
#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_IMU_SYNC)
        if (imuUpdated == false) {
            IMU_UNLOCK;
            return;
        }
        imuUpdated = false;
#endif
        imuCalculateEstimatedAttitude(currentTimeUs);
        IMU_UNLOCK;

        // Update the throttle correction for angle and supply it to the mixer
        int throttleAngleCorrection = 0;
        if (imuRuntimeConfig.throttleAngleValue
            && (FLIGHT_MODE(ANGLE_MODE | HORIZON_MODE)) 
            && ARMING_FLAG(ARMED)) {
            throttleAngleCorrection = calculateThrottleAngleCorrection(&imuRuntimeConfig);
        }
        mixerSetThrottleAngleCorrection(throttleAngleCorrection);

    } else {
        if (!sensors(SENSOR_ACC) || !acc.isAccelUpdatedAtLeastOnce) {
            acc.accADC.x = 0;
            acc.accADC.y = 0;
            acc.accADC.z = 0;
        }
        schedulerIgnoreTaskStateTime();
        imuResetEstimateCovariance(&ahrsState);
    }

    DEBUG_SET(DEBUG_ATTITUDE, 0, attitude.values.roll);
    DEBUG_SET(DEBUG_ATTITUDE, 1, attitude.values.pitch);
}
#endif // USE_ACC

// Angle in between the nose axis of the craft and the horizontal plane in ground reference.
// Positive angle - nose down, negative angle - nose up.
float getSinPitchAngle(void)
{
    return -rMat.m[2][0];
}

float getCosTiltAngle(void)
{
    return rMat.m[2][2];
}

void imuGetQuaternion(quaternion *quat)
{
    *quat = ahrsState.nominal.attitude;
}

void imuGetState(imuAhrsState_t* state)
{
    *state = ahrsState;
}

void imuGetRotMatrix(matrix33_t* m)
{
    *m = rMat;
}

#ifdef SIMULATOR_BUILD
void imuSetAttitudeRPY(float roll, float pitch, float yaw)
{
    IMU_LOCK;

    attitude.values.roll = roll * 10;
    attitude.values.pitch = pitch * 10;
    attitude.values.yaw = yaw * 10;

    IMU_UNLOCK;
}

void imuSetAttitudeQuat(float w, float x, float y, float z)
{
    IMU_LOCK;

    ahrsState.nominal.attitude.w = w;
    ahrsState.nominal.attitude.x = x;
    ahrsState.nominal.attitude.y = y;
    ahrsState.nominal.attitude.z = z;

    imuComputeRotationMatrix(&rMat, &ahrsState.nominal.attitude);

    attitudeIsEstablished = true;

    imuUpdateEulerAngles(&rMat);

    IMU_UNLOCK;
}
#endif
#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_IMU_SYNC)
void imuSetHasNewData(uint32_t dt)
{
    IMU_LOCK;

    imuUpdated = true;
    imuDeltaT = dt;

    IMU_UNLOCK;
}
#endif

bool imuQuaternionHeadfreeOffsetSet(void)
{
    if ((abs(attitude.values.roll) < 450)  && (abs(attitude.values.pitch) < 450)) {
        const quaternion* q = &ahrsState.nominal.attitude;
        const float wz = q->w * q->z;
        const float xy = q->x * q->y;
        const float yy = sq(q->y);
        const float zz = sq(q->z);
        const float yaw = -atan2_approx((+2.0f * (wz + xy)), (+1.0f - 2.0f * (yy + zz)));

        offset.w = cos_approx(yaw/2);
        offset.x = 0;
        offset.y = 0;
        offset.z = sin_approx(yaw/2);

        return true;
    } else {
        return false;
    }
}

void imuQuaternionMultiplication(quaternion *result, const quaternion *q1, const quaternion *q2)
{
    const float A = (q1->w + q1->x) * (q2->w + q2->x);
    const float B = (q1->z - q1->y) * (q2->y - q2->z);
    const float C = (q1->w - q1->x) * (q2->y + q2->z);
    const float D = (q1->y + q1->z) * (q2->w - q2->x);
    const float E = (q1->x + q1->z) * (q2->x + q2->y);
    const float F = (q1->x - q1->z) * (q2->x - q2->y);
    const float G = (q1->w + q1->y) * (q2->w - q2->z);
    const float H = (q1->w - q1->y) * (q2->w + q2->z);

    result->w = B + (- E - F + G + H) / 2.0f;
    result->x = A - (+ E + F + G + H) / 2.0f;
    result->y = C + (+ E - F + G - H) / 2.0f;
    result->z = D + (+ E - F - G + H) / 2.0f;
}

void imuQuaternionHeadfreeTransformVectorEarthToBody(vector3_t *v)
{
    quaternionProducts buffer;
    const quaternion* q = &ahrsState.nominal.attitude;

    imuQuaternionMultiplication(&headfree, &offset, q);
    imuQuaternionComputeProducts(&buffer, &headfree);

    const float x = (buffer.ww + buffer.xx - buffer.yy - buffer.zz) * v->x + 2.0f * (buffer.xy + buffer.wz) * v->y + 2.0f * (buffer.xz - buffer.wy) * v->z;
    const float y = 2.0f * (buffer.xy - buffer.wz) * v->x + (buffer.ww - buffer.xx + buffer.yy - buffer.zz) * v->y + 2.0f * (buffer.yz + buffer.wx) * v->z;
    const float z = 2.0f * (buffer.xz + buffer.wy) * v->x + 2.0f * (buffer.yz - buffer.wx) * v->y + (buffer.ww - buffer.xx - buffer.yy + buffer.zz) * v->z;

    v->x = x;
    v->y = y;
    v->z = z;
}

bool isUpright(void)
{
#ifdef USE_ACC
    return !sensors(SENSOR_ACC) || (attitudeIsEstablished && getCosTiltAngle() > imuRuntimeConfig.smallAngleCosZ);
#else
    return true;
#endif
}
