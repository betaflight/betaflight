/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

// Inertial Measurement Unit (IMU)

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "build/build_config.h"

#include "platform.h"

#include "blackbox/blackbox.h"
#include "build/build_config.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"

#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/time.h"

#include "fc/config.h"
#include "fc/runtime_config.h"

#include "flight/hil.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "io/gps.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"


/**
 * In Cleanflight accelerometer is aligned in the following way:
 *      X-axis = Forward
 *      Y-axis = Left
 *      Z-axis = Up
 * Our INAV uses different convention
 *      X-axis = North/Forward
 *      Y-axis = East/Right
 *      Z-axis = Up
 */

// the limit (in degrees/second) beyond which we stop integrating
// omega_I. At larger spin rates the DCM PI controller can get 'dizzy'
// which results in false gyro drift. See
// http://gentlenav.googlecode.com/files/fastRotations.pdf

#define SPIN_RATE_LIMIT             20
#define MAX_ACC_SQ_NEARNESS         25      // 25% or G^2, accepted acceleration of (0.87 - 1.12G)
#define MAX_GPS_HEADING_ERROR_DEG   60      // Amount of error between GPS CoG and estimated Yaw at witch we stop trusting GPS and fallback to MAG

t_fp_vector imuMeasuredAccelBF;
t_fp_vector imuMeasuredRotationBF;
static float smallAngleCosZ = 0;

static bool isAccelUpdatedAtLeastOnce = false;

STATIC_UNIT_TESTED float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;    // quaternion of sensor frame relative to earth frame
STATIC_UNIT_TESTED float rMat[3][3];

attitudeEulerAngles_t attitude = { { 0, 0, 0 } };     // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800

static imuRuntimeConfig_t imuRuntimeConfig;

static const float gyroScale = (M_PIf / 180.0f);  // gyro output scaled to rad per second

static bool gpsHeadingInitialized = false;

PG_REGISTER_WITH_RESET_TEMPLATE(imuConfig_t, imuConfig, PG_IMU_CONFIG, 0);

PG_RESET_TEMPLATE(imuConfig_t, imuConfig,
    .dcm_kp_acc = 2500,             // 0.25 * 10000
    .dcm_ki_acc = 50,               // 0.005 * 10000
    .dcm_kp_mag = 10000,            // 1.00 * 10000
    .dcm_ki_mag = 0,                // 0.00 * 10000
    .small_angle = 25

);

#ifdef ASYNC_GYRO_PROCESSING
/* Asynchronous update accumulators */
static float imuAccumulatedRate[XYZ_AXIS_COUNT];
static timeUs_t imuAccumulatedRateTimeUs;
static float imuAccumulatedAcc[XYZ_AXIS_COUNT];
static int   imuAccumulatedAccCount;
#endif

#ifdef ASYNC_GYRO_PROCESSING
void imuUpdateGyroscope(timeUs_t gyroUpdateDeltaUs)
{
    const float gyroUpdateDelta = gyroUpdateDeltaUs * 1e-6f;

    for (int axis = 0; axis < 3; axis++) {
        imuAccumulatedRate[axis] += gyro.gyroADCf[axis] * gyroScale * gyroUpdateDelta;
    }

    imuAccumulatedRateTimeUs += gyroUpdateDeltaUs;
}
#endif


STATIC_UNIT_TESTED void imuComputeRotationMatrix(void)
{
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3;

    rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    rMat[0][1] = 2.0f * (q1q2 + -q0q3);
    rMat[0][2] = 2.0f * (q1q3 - -q0q2);

    rMat[1][0] = 2.0f * (q1q2 - -q0q3);
    rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    rMat[1][2] = 2.0f * (q2q3 + -q0q1);

    rMat[2][0] = 2.0f * (q1q3 + -q0q2);
    rMat[2][1] = 2.0f * (q2q3 - -q0q1);
    rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}

void imuConfigure(void)
{
    imuRuntimeConfig.dcm_kp_acc = imuConfig()->dcm_kp_acc / 10000.0f;
    imuRuntimeConfig.dcm_ki_acc = imuConfig()->dcm_ki_acc / 10000.0f;
    imuRuntimeConfig.dcm_kp_mag = imuConfig()->dcm_kp_mag / 10000.0f;
    imuRuntimeConfig.dcm_ki_mag = imuConfig()->dcm_ki_mag / 10000.0f;
    imuRuntimeConfig.small_angle = imuConfig()->small_angle;
}

void imuInit(void)
{
    smallAngleCosZ = cos_approx(degreesToRadians(imuRuntimeConfig.small_angle));

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        imuMeasuredAccelBF.A[axis] = 0;
    }

    imuComputeRotationMatrix();
}

void imuTransformVectorBodyToEarth(t_fp_vector * v)
{
    /* From body frame to earth frame */
    const float x = rMat[0][0] * v->V.X + rMat[0][1] * v->V.Y + rMat[0][2] * v->V.Z;
    const float y = rMat[1][0] * v->V.X + rMat[1][1] * v->V.Y + rMat[1][2] * v->V.Z;
    const float z = rMat[2][0] * v->V.X + rMat[2][1] * v->V.Y + rMat[2][2] * v->V.Z;

    v->V.X = x;
    v->V.Y = -y;
    v->V.Z = z;
}

void imuTransformVectorEarthToBody(t_fp_vector * v)
{
    v->V.Y = -v->V.Y;

    /* From earth frame to body frame */
    const float x = rMat[0][0] * v->V.X + rMat[1][0] * v->V.Y + rMat[2][0] * v->V.Z;
    const float y = rMat[0][1] * v->V.X + rMat[1][1] * v->V.Y + rMat[2][1] * v->V.Z;
    const float z = rMat[0][2] * v->V.X + rMat[1][2] * v->V.Y + rMat[2][2] * v->V.Z;

    v->V.X = x;
    v->V.Y = y;
    v->V.Z = z;
}

static float invSqrt(float x)
{
    return 1.0f / sqrtf(x);
}

#if defined(GPS) || defined(HIL)
STATIC_UNIT_TESTED void imuComputeQuaternionFromRPY(int16_t initialRoll, int16_t initialPitch, int16_t initialYaw)
{
    if (initialRoll > 1800) initialRoll -= 3600;
    if (initialPitch > 1800) initialPitch -= 3600;
    if (initialYaw > 1800) initialYaw -= 3600;

    const float cosRoll = cos_approx(DECIDEGREES_TO_RADIANS(initialRoll) * 0.5f);
    const float sinRoll = sin_approx(DECIDEGREES_TO_RADIANS(initialRoll) * 0.5f);

    const float cosPitch = cos_approx(DECIDEGREES_TO_RADIANS(initialPitch) * 0.5f);
    const float sinPitch = sin_approx(DECIDEGREES_TO_RADIANS(initialPitch) * 0.5f);

    const float cosYaw = cos_approx(DECIDEGREES_TO_RADIANS(-initialYaw) * 0.5f);
    const float sinYaw = sin_approx(DECIDEGREES_TO_RADIANS(-initialYaw) * 0.5f);

    q0 = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
    q1 = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
    q2 = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
    q3 = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;

    imuComputeRotationMatrix();
}
#endif

static bool imuUseFastGains(void)
{
    return !ARMING_FLAG(ARMED) && millis() < 20000;
}

static float imuGetPGainScaleFactor(void)
{
    if (imuUseFastGains()) {
        return 10.0f;
    }
    else {
        return 1.0f;
    }
}

static void imuResetOrientationQuaternion(const float ax, const float ay, const float az)
{
    const float accNorm = sqrtf(ax * ax + ay * ay + az * az);

    q0 = az + accNorm;
    q1 = ay;
    q2 = -ax;
    q3 = 0.0f;

    const float recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

static void imuCheckAndResetOrientationQuaternion(const float ax, const float ay, const float az)
{
    // Check if some calculation in IMU update yield NAN or zero quaternion
    // Reset quaternion from accelerometer - this might be incorrect, but it's better than no attitude at all

    const bool isNan = (isnan(q0) || isnan(q1) || isnan(q2) || isnan(q3));
    const bool isInf = (isinf(q0) || isinf(q1) || isinf(q2) || isinf(q3));
    const bool isZero = (ABS(q0) < 1e-3f && ABS(q1) < 1e-3f && ABS(q2) < 1e-3f && ABS(q3) < 1e-3f);

    if (isNan || isZero || isInf) {
        imuResetOrientationQuaternion(ax, ay, az);

#ifdef BLACKBOX
        if (feature(FEATURE_BLACKBOX)) {
            blackboxLogEvent(FLIGHT_LOG_EVENT_IMU_FAILURE, NULL);
        }
#endif
    }
}

static void imuMahonyAHRSupdate(float dt, float gx, float gy, float gz,
                                bool useAcc, float ax, float ay, float az,
                                bool useMag, float mx, float my, float mz,
                                bool useCOG, float courseOverGround)
{
    static float integralAccX = 0.0f,  integralAccY = 0.0f, integralAccZ = 0.0f;    // integral error terms scaled by Ki
    static float integralMagX = 0.0f,  integralMagY = 0.0f, integralMagZ = 0.0f;    // integral error terms scaled by Ki
    float ex, ey, ez;

    /* Calculate general spin rate (rad/s) */
    const float spin_rate_sq = sq(gx) + sq(gy) + sq(gz);

    /* Step 1: Yaw correction */
    // Use measured magnetic field vector
    if (useMag || useCOG) {
        float kpMag = imuRuntimeConfig.dcm_kp_mag * imuGetPGainScaleFactor();
        const float magMagnitudeSq = mx * mx + my * my + mz * mz;

        if (useMag && magMagnitudeSq > 0.01f) {
            // Normalise magnetometer measurement
            const float magRecipNorm = invSqrt(magMagnitudeSq);
            mx *= magRecipNorm;
            my *= magRecipNorm;
            mz *= magRecipNorm;

            // For magnetometer correction we make an assumption that magnetic field is perpendicular to gravity (ignore Z-component in EF).
            // This way magnetic field will only affect heading and wont mess roll/pitch angles

            // (hx; hy; 0) - measured mag field vector in EF (assuming Z-component is zero)
            // (bx; 0; 0) - reference mag field vector heading due North in EF (assuming Z-component is zero)
            const float hx = rMat[0][0] * mx + rMat[0][1] * my + rMat[0][2] * mz;
            const float hy = rMat[1][0] * mx + rMat[1][1] * my + rMat[1][2] * mz;
            const float bx = sqrtf(hx * hx + hy * hy);

            // magnetometer error is cross product between estimated magnetic north and measured magnetic north (calculated in EF)
            const float ez_ef = -(hy * bx);

            // Rotate mag error vector back to BF and accumulate
            ex = rMat[2][0] * ez_ef;
            ey = rMat[2][1] * ez_ef;
            ez = rMat[2][2] * ez_ef;
        }
        else if (useCOG) {
            // Use raw heading error (from GPS or whatever else)
            while (courseOverGround >  M_PIf) courseOverGround -= (2.0f * M_PIf);
            while (courseOverGround < -M_PIf) courseOverGround += (2.0f * M_PIf);

            // William Premerlani and Paul Bizard, Direction Cosine Matrix IMU - Eqn. 22-23
            // (Rxx; Ryx) - measured (estimated) heading vector (EF)
            // (cos(COG), sin(COG)) - reference heading vector (EF)
            // error is cross product between reference heading and estimated heading (calculated in EF)
            const float ez_ef = - sin_approx(courseOverGround) * rMat[0][0] - cos_approx(courseOverGround) * rMat[1][0];

            ex = rMat[2][0] * ez_ef;
            ey = rMat[2][1] * ez_ef;
            ez = rMat[2][2] * ez_ef;
        }
        else {
            ex = 0;
            ey = 0;
            ez = 0;
        }

        // Compute and apply integral feedback if enabled
        if (imuRuntimeConfig.dcm_ki_mag > 0.0f) {
            // Stop integrating if spinning beyond the certain limit
            if (spin_rate_sq < sq(DEGREES_TO_RADIANS(SPIN_RATE_LIMIT))) {
                integralMagX += imuRuntimeConfig.dcm_ki_mag * ex * dt;    // integral error scaled by Ki
                integralMagY += imuRuntimeConfig.dcm_ki_mag * ey * dt;
                integralMagZ += imuRuntimeConfig.dcm_ki_mag * ez * dt;

                gx += integralMagX;
                gy += integralMagY;
                gz += integralMagZ;
            }
        }

        // Calculate kP gain and apply proportional feedback
        gx += kpMag * ex;
        gy += kpMag * ey;
        gz += kpMag * ez;
    }


    /* Step 2: Roll and pitch correction -  use measured acceleration vector */
    if (useAcc) {
        float kpAcc = imuRuntimeConfig.dcm_kp_acc * imuGetPGainScaleFactor();
        const float accRecipNorm = invSqrt(ax * ax + ay * ay + az * az);

        // Just scale by 1G length - That's our vector adjustment. Rather than
        // using one-over-exact length (which needs a costly square root), we already
        // know the vector is enough "roughly unit length" and since it is only weighted
        // in by a certain amount anyway later, having that exact is meaningless. (c) MasterZap
        ax *= accRecipNorm;
        ay *= accRecipNorm;
        az *= accRecipNorm;

        // Error is sum of cross product between estimated direction and measured direction of gravity
        ex = (ay * rMat[2][2] - az * rMat[2][1]);
        ey = (az * rMat[2][0] - ax * rMat[2][2]);
        ez = (ax * rMat[2][1] - ay * rMat[2][0]);

        // Compute and apply integral feedback if enabled
        if (imuRuntimeConfig.dcm_ki_acc > 0.0f) {
            // Stop integrating if spinning beyond the certain limit
            if (spin_rate_sq < sq(DEGREES_TO_RADIANS(SPIN_RATE_LIMIT))) {
                integralAccX += imuRuntimeConfig.dcm_ki_acc * ex * dt;    // integral error scaled by Ki
                integralAccY += imuRuntimeConfig.dcm_ki_acc * ey * dt;
                integralAccZ += imuRuntimeConfig.dcm_ki_acc * ez * dt;

                gx += integralAccX;
                gy += integralAccY;
                gz += integralAccZ;
            }
        }

        // Calculate kP gain and apply proportional feedback
        gx += kpAcc * ex;
        gy += kpAcc * ey;
        gz += kpAcc * ez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    const float qa = q0;
    const float qb = q1;
    const float qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    const float quatRecipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= quatRecipNorm;
    q1 *= quatRecipNorm;
    q2 *= quatRecipNorm;
    q3 *= quatRecipNorm;

    // Check for invalid quaternion
    imuCheckAndResetOrientationQuaternion(ax, ay, az);

    // Pre-compute rotation matrix from quaternion
    imuComputeRotationMatrix();
}

STATIC_UNIT_TESTED void imuUpdateEulerAngles(void)
{
    /* Compute pitch/roll angles */
    attitude.values.roll = RADIANS_TO_DECIDEGREES(atan2_approx(rMat[2][1], rMat[2][2]));
    attitude.values.pitch = RADIANS_TO_DECIDEGREES((0.5f * M_PIf) - acos_approx(-rMat[2][0]));
    attitude.values.yaw = RADIANS_TO_DECIDEGREES(-atan2_approx(rMat[1][0], rMat[0][0])) + mag.magneticDeclination;

    if (attitude.values.yaw < 0)
        attitude.values.yaw += 3600;

    /* Update small angle state */
    if (rMat[2][2] > smallAngleCosZ) {
        ENABLE_STATE(SMALL_ANGLE);
    } else {
        DISABLE_STATE(SMALL_ANGLE);
    }
}

static bool imuCanUseAccelerometerForCorrection(void)
{
    int32_t axis;
    int32_t accMagnitudeSq = 0;

    for (axis = 0; axis < 3; axis++) {
        accMagnitudeSq += (int32_t)acc.accADC[axis] * acc.accADC[axis];
    }

    // Magnitude^2 in percent of G^2
    const int nearness = ABS(100 - (accMagnitudeSq * 100 / ((int32_t)acc.dev.acc_1G * acc.dev.acc_1G)));

    return (nearness > MAX_ACC_SQ_NEARNESS) ? false : true;
}

static void imuCalculateEstimatedAttitude(float dT)
{
#if defined(MAG)
    const bool canUseMAG = sensors(SENSOR_MAG) && compassIsHealthy();
#else
    const bool canUseMAG = false;
#endif

    const bool useAcc = imuCanUseAccelerometerForCorrection();

    float courseOverGround = 0;
    bool useMag = false;
    bool useCOG = false;

#if defined(GPS)
    if (STATE(FIXED_WING)) {
        bool canUseCOG = sensors(SENSOR_GPS) && STATE(GPS_FIX) && gpsSol.numSat >= 6 && gpsSol.groundSpeed >= 300;

        if (canUseCOG) {
            if (gpsHeadingInitialized) {
                // Use GPS heading if error is acceptable or if it's the only source of heading
                if (ABS(gpsSol.groundCourse - attitude.values.yaw) < DEGREES_TO_DECIDEGREES(MAX_GPS_HEADING_ERROR_DEG) || !canUseMAG) {
                    courseOverGround = DECIDEGREES_TO_RADIANS(gpsSol.groundCourse);
                    useCOG = true;
                }
            }
            else {
                // Re-initialize quaternion from known Roll, Pitch and GPS heading
                imuComputeQuaternionFromRPY(attitude.values.roll, attitude.values.pitch, gpsSol.groundCourse);
                gpsHeadingInitialized = true;

                // Force reset of heading hold target
                resetHeadingHoldTarget(DECIDEGREES_TO_DEGREES(attitude.values.yaw));
            }

            // If we can't use COG and there's MAG available - fallback
            if (!useCOG && canUseMAG) {
                useMag = true;
            }
        }
        else if (canUseMAG) {
            useMag = true;
            gpsHeadingInitialized = true;   // GPS heading initialised from MAG, continue on GPS if possible
        }
    }
    else {
        // Multicopters don't use GPS heading
        if (canUseMAG) {
            useMag = true;
        }
    }
#else
    // In absence of GPS MAG is the only option
    if (canUseMAG) {
        useMag = true;
    }
#endif

    imuMahonyAHRSupdate(dT,     imuMeasuredRotationBF.A[X], imuMeasuredRotationBF.A[Y], imuMeasuredRotationBF.A[Z],
                        useAcc, imuMeasuredAccelBF.A[X], imuMeasuredAccelBF.A[Y], imuMeasuredAccelBF.A[Z],
                        useMag, mag.magADC[X], mag.magADC[Y], mag.magADC[Z],
                        useCOG, courseOverGround);

    imuUpdateEulerAngles();
}

/* Calculate rotation rate in rad/s in body frame */
static void imuUpdateMeasuredRotationRate(void)
{
    int axis;

#ifdef ASYNC_GYRO_PROCESSING
    const float imuAccumulatedRateTime = imuAccumulatedRateTimeUs * 1e-6;
    imuAccumulatedRateTimeUs = 0;

    for (axis = 0; axis < 3; axis++) {
        imuMeasuredRotationBF.A[axis] = imuAccumulatedRate[axis] / imuAccumulatedRateTime;
        imuAccumulatedRate[axis] = 0.0f;
    }
#else
    for (axis = 0; axis < 3; axis++) {
        imuMeasuredRotationBF.A[axis] = gyro.gyroADCf[axis] * gyroScale;
    }
#endif
}

/* Calculate measured acceleration in body frame cm/s/s */
static void imuUpdateMeasuredAcceleration(void)
{
    int axis;

#ifdef ASYNC_GYRO_PROCESSING
    for (axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        imuMeasuredAccelBF.A[axis] = imuAccumulatedAcc[axis] / imuAccumulatedAccCount;
        imuAccumulatedAcc[axis] = 0;
    }
    imuAccumulatedAccCount = 0;;
#else
    /* Convert acceleration to cm/s/s */
    for (axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        imuMeasuredAccelBF.A[axis] = acc.accADC[axis] * (GRAVITY_CMSS / acc.dev.acc_1G);
    }
#endif
}

#ifdef HIL
void imuHILUpdate(void)
{
    /* Set attitude */
    attitude.values.roll = hilToFC.rollAngle;
    attitude.values.pitch = hilToFC.pitchAngle;
    attitude.values.yaw = hilToFC.yawAngle;

    /* Compute rotation quaternion for future use */
    imuComputeQuaternionFromRPY(attitude.values.roll, attitude.values.pitch, attitude.values.yaw);

    /* Fake accADC readings */
    accADC[X] = hilToFC.bodyAccel[X] * (acc.acc_1G / GRAVITY_CMSS);
    accADC[Y] = hilToFC.bodyAccel[Y] * (acc.acc_1G / GRAVITY_CMSS);
    accADC[Z] = hilToFC.bodyAccel[Z] * (acc.acc_1G / GRAVITY_CMSS);
}
#endif

void imuUpdateAccelerometer(void)
{
#ifdef HIL
    if (sensors(SENSOR_ACC) && !hilActive) {
        accUpdate();
        isAccelUpdatedAtLeastOnce = true;
    }
#else
    if (sensors(SENSOR_ACC)) {
        accUpdate();
        isAccelUpdatedAtLeastOnce = true;
    }
#endif

#ifdef ASYNC_GYRO_PROCESSING
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        imuAccumulatedAcc[axis] += acc.accADC[axis] * (GRAVITY_CMSS / acc.dev.acc_1G);
    }
    imuAccumulatedAccCount++;
#endif
}

void imuUpdateAttitude(timeUs_t currentTimeUs)
{
    /* Calculate dT */
    static timeUs_t previousIMUUpdateTimeUs;
    const float dT = (currentTimeUs - previousIMUUpdateTimeUs) * 1e-6;
    previousIMUUpdateTimeUs = currentTimeUs;

    if (sensors(SENSOR_ACC) && isAccelUpdatedAtLeastOnce) {
#ifdef HIL
        if (!hilActive) {
            imuUpdateMeasuredRotationRate();    // Calculate gyro rate in body frame in rad/s
            imuUpdateMeasuredAcceleration();  // Calculate accel in body frame in cm/s/s
            imuCalculateEstimatedAttitude(dT);  // Update attitude estimate
        }
        else {
            imuHILUpdate();
            imuUpdateMeasuredAcceleration();
        }
#else
            imuUpdateMeasuredRotationRate();    // Calculate gyro rate in body frame in rad/s
            imuUpdateMeasuredAcceleration();  // Calculate accel in body frame in cm/s/s
            imuCalculateEstimatedAttitude(dT);  // Update attitude estimate
#endif
    } else {
        acc.accADC[X] = 0;
        acc.accADC[Y] = 0;
        acc.accADC[Z] = 0;
    }
}

bool isImuReady(void)
{
    return sensors(SENSOR_ACC) && gyroIsCalibrationComplete();
}

bool isImuHeadingValid(void)
{
    return (sensors(SENSOR_MAG) && STATE(COMPASS_CALIBRATED)) || (STATE(FIXED_WING) && gpsHeadingInitialized);
}

float calculateCosTiltAngle(void)
{
    return rMat[2][2];
}
