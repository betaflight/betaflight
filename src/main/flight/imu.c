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

#include "common/maths.h"

#include <platform.h>

#include "common/axis.h"
#include "flight/flight.h"

#include "drivers/system.h"

#include "sensors/sensors.h"
#include "drivers/accgyro.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/sonar.h"

#include "config/runtime_config.h"

#include "flight/mixer.h"
#include "flight/imu.h"

extern int16_t debug[4];

int16_t gyroADC[XYZ_AXIS_COUNT], accADC[XYZ_AXIS_COUNT], accSmooth[XYZ_AXIS_COUNT];
int32_t accSum[XYZ_AXIS_COUNT];

uint32_t accTimeSum = 0;        // keep track for integration of acc
int accSumCount = 0;
float accVelScale;

int16_t smallAngle = 0;

int32_t EstAlt;                // in cm
int32_t AltHold;
int32_t setVelocity = 0;
uint8_t velocityControl = 0;
int32_t errorVelocityI = 0;

int32_t vario = 0;                      // variometer in cm/s

float throttleAngleScale;
float fc_acc;

int32_t BaroPID = 0;

float magneticDeclination = 0.0f;       // calculated at startup from config
float gyroScaleRad;

// **************
// gyro+acc IMU
// **************
int16_t gyroData[FLIGHT_DYNAMICS_INDEX_COUNT] = { 0, 0, 0 };
int16_t gyroZero[FLIGHT_DYNAMICS_INDEX_COUNT] = { 0, 0, 0 };

rollAndPitchInclination_t inclination = { { 0, 0 } };     // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
float anglerad[2] = { 0.0f, 0.0f };    // absolute angle inclination in radians

static void getEstimatedAttitude(void);

imuRuntimeConfig_t *imuRuntimeConfig;
pidProfile_t *pidProfile;
barometerConfig_t *barometerConfig;
accDeadband_t *accDeadband;

void configureImu(imuRuntimeConfig_t *initialImuRuntimeConfig, pidProfile_t *initialPidProfile, barometerConfig_t *intialBarometerConfig, accDeadband_t *initialAccDeadband)
{
    imuRuntimeConfig = initialImuRuntimeConfig;
    pidProfile = initialPidProfile;
    barometerConfig = intialBarometerConfig;
    accDeadband = initialAccDeadband;
}

void imuInit()
{
    smallAngle = lrintf(acc_1G * cosf(RAD * imuRuntimeConfig->small_angle));
    accVelScale = 9.80665f / acc_1G / 10000.0f;
    gyroScaleRad = gyro.scale * (M_PI / 180.0f) * 0.000001f;
}

void calculateThrottleAngleScale(uint16_t throttle_correction_angle)
{
    throttleAngleScale = (1800.0f / M_PI) * (900.0f / throttle_correction_angle);
}

void calculateAccZLowPassFilterRCTimeConstant(float accz_lpf_cutoff)
{
    fc_acc = 0.5f / (M_PI * accz_lpf_cutoff); // calculate RC time constant used in the accZ lpf
}

void computeIMU(rollAndPitchTrims_t *accelerometerTrims, uint8_t mixerConfiguration)
{
    static int16_t gyroYawSmooth = 0;

    gyroGetADC();
    if (sensors(SENSOR_ACC)) {
        updateAccelerationReadings(accelerometerTrims);
        getEstimatedAttitude();
    } else {
        accADC[X] = 0;
        accADC[Y] = 0;
        accADC[Z] = 0;
    }

    gyroData[FD_ROLL] = gyroADC[FD_ROLL];
    gyroData[FD_PITCH] = gyroADC[FD_PITCH];

    if (mixerConfiguration == MULTITYPE_TRI) {
        gyroData[FD_YAW] = (gyroYawSmooth * 2 + gyroADC[FD_YAW]) / 3;
        gyroYawSmooth = gyroData[FD_YAW];
    } else {
        gyroData[FD_YAW] = gyroADC[FD_YAW];
    }
}

// **************************************************
// Simplified IMU based on "Complementary Filter"
// Inspired by http://starlino.com/imu_guide.html
//
// adapted by ziss_dm : http://www.multiwii.com/forum/viewtopic.php?f=8&t=198
//
// The following ideas was used in this project:
// 1) Rotation matrix: http://en.wikipedia.org/wiki/Rotation_matrix
//
// Currently Magnetometer uses separate CF which is used only
// for heading approximation.
//
// **************************************************


t_fp_vector EstG;

// Normalize a vector
void normalizeV(struct fp_vector *src, struct fp_vector *dest)
{
    float length;

    length = sqrtf(src->X * src->X + src->Y * src->Y + src->Z * src->Z);
    if (length != 0) {
        dest->X = src->X / length;
        dest->Y = src->Y / length;
        dest->Z = src->Z / length;
    }
}

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void rotateV(struct fp_vector *v, fp_angles_t *delta)
{
    struct fp_vector v_tmp = *v;

    // This does a  "proper" matrix rotation using gyro deltas without small-angle approximation
    float mat[3][3];
    float cosx, sinx, cosy, siny, cosz, sinz;
    float coszcosx, sinzcosx, coszsinx, sinzsinx;

    cosx = cosf(delta->angles.roll);
    sinx = sinf(delta->angles.roll);
    cosy = cosf(delta->angles.pitch);
    siny = sinf(delta->angles.pitch);
    cosz = cosf(delta->angles.yaw);
    sinz = sinf(delta->angles.yaw);

    coszcosx = cosz * cosx;
    sinzcosx = sinz * cosx;
    coszsinx = sinx * cosz;
    sinzsinx = sinx * sinz;

    mat[0][0] = cosz * cosy;
    mat[0][1] = -cosy * sinz;
    mat[0][2] = siny;
    mat[1][0] = sinzcosx + (coszsinx * siny);
    mat[1][1] = coszcosx - (sinzsinx * siny);
    mat[1][2] = -sinx * cosy;
    mat[2][0] = (sinzsinx) - (coszcosx * siny);
    mat[2][1] = (coszsinx) + (sinzcosx * siny);
    mat[2][2] = cosy * cosx;

    v->X = v_tmp.X * mat[0][0] + v_tmp.Y * mat[1][0] + v_tmp.Z * mat[2][0];
    v->Y = v_tmp.X * mat[0][1] + v_tmp.Y * mat[1][1] + v_tmp.Z * mat[2][1];
    v->Z = v_tmp.X * mat[0][2] + v_tmp.Y * mat[1][2] + v_tmp.Z * mat[2][2];
}

int32_t applyDeadband(int32_t value, int32_t deadband)
{
    if (abs(value) < deadband) {
        value = 0;
    } else if (value > 0) {
        value -= deadband;
    } else if (value < 0) {
        value += deadband;
    }
    return value;
}

// rotate acc into Earth frame and calculate acceleration in it
void acc_calc(uint32_t deltaT)
{
    static int32_t accZoffset = 0;
    static float accz_smooth = 0;
    float dT;
    fp_angles_t rpy;
    t_fp_vector accel_ned;

    // deltaT is measured in us ticks
    dT = (float)deltaT * 1e-6f;

    // the accel values have to be rotated into the earth frame
    rpy.angles.roll = -(float)anglerad[AI_ROLL];
    rpy.angles.pitch = -(float)anglerad[AI_PITCH];
    rpy.angles.yaw = -(float)heading * RAD;

    accel_ned.V.X = accSmooth[0];
    accel_ned.V.Y = accSmooth[1];
    accel_ned.V.Z = accSmooth[2];

    rotateV(&accel_ned.V, &rpy);

    if (imuRuntimeConfig->acc_unarmedcal == 1) {
        if (!ARMING_FLAG(ARMED)) {
            accZoffset -= accZoffset / 64;
            accZoffset += accel_ned.V.Z;
        }
        accel_ned.V.Z -= accZoffset / 64;  // compensate for gravitation on z-axis
    } else
        accel_ned.V.Z -= acc_1G;

    accz_smooth = accz_smooth + (dT / (fc_acc + dT)) * (accel_ned.V.Z - accz_smooth); // low pass filter

    // apply Deadband to reduce integration drift and vibration influence
    accSum[X] += applyDeadband(lrintf(accel_ned.V.X), accDeadband->xy);
    accSum[Y] += applyDeadband(lrintf(accel_ned.V.Y), accDeadband->xy);
    accSum[Z] += applyDeadband(lrintf(accz_smooth), accDeadband->z);

    // sum up Values for later integration to get velocity and distance
    accTimeSum += deltaT;
    accSumCount++;
}

void accSum_reset(void)
{
    accSum[0] = 0;
    accSum[1] = 0;
    accSum[2] = 0;
    accSumCount = 0;
    accTimeSum = 0;
}

// baseflight calculation by Luggi09 originates from arducopter
static int16_t calculateHeading(t_fp_vector *vec)
{
    int16_t head;

    float cosineRoll = cosf(anglerad[AI_ROLL]);
    float sineRoll = sinf(anglerad[AI_ROLL]);
    float cosinePitch = cosf(anglerad[AI_PITCH]);
    float sinePitch = sinf(anglerad[AI_PITCH]);
    float Xh = vec->A[X] * cosinePitch + vec->A[Y] * sineRoll * sinePitch + vec->A[Z] * sinePitch * cosineRoll;
    float Yh = vec->A[Y] * cosineRoll - vec->A[Z] * sineRoll;
    float hd = (atan2f(Yh, Xh) * 1800.0f / M_PI + magneticDeclination) / 10.0f;
    head = lrintf(hd);
    if (head < 0)
        head += 360;

    return head;
}

static void getEstimatedAttitude(void)
{
    int32_t axis;
    int32_t accMag = 0;
    static t_fp_vector EstM;
    static t_fp_vector EstN = { .A = { 1.0f, 0.0f, 0.0f } };
    static float accLPF[3];
    static uint32_t previousT;
    uint32_t currentT = micros();
    uint32_t deltaT;
    float scale;
    fp_angles_t deltaGyroAngle;
    deltaT = currentT - previousT;
    scale = deltaT * gyroScaleRad;
    previousT = currentT;

    // Initialization
    for (axis = 0; axis < 3; axis++) {
        deltaGyroAngle.raw[axis] = gyroADC[axis] * scale;
        if (imuRuntimeConfig->acc_lpf_factor > 0) {
            accLPF[axis] = accLPF[axis] * (1.0f - (1.0f / imuRuntimeConfig->acc_lpf_factor)) + accADC[axis] * (1.0f / imuRuntimeConfig->acc_lpf_factor);
            accSmooth[axis] = accLPF[axis];
        } else {
            accSmooth[axis] = accADC[axis];
        }
        accMag += (int32_t)accSmooth[axis] * accSmooth[axis];
    }
    accMag = accMag * 100 / ((int32_t)acc_1G * acc_1G);

    rotateV(&EstG.V, &deltaGyroAngle);

    // Apply complimentary filter (Gyro drift correction)
    // If accel magnitude >1.15G or <0.85G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
    // To do that, we just skip filter, as EstV already rotated by Gyro

    float invGyroComplimentaryFilterFactor = (1.0f / (imuRuntimeConfig->gyro_cmpf_factor + 1.0f));

    if (72 < (uint16_t)accMag && (uint16_t)accMag < 133) {
        for (axis = 0; axis < 3; axis++)
            EstG.A[axis] = (EstG.A[axis] * imuRuntimeConfig->gyro_cmpf_factor + accSmooth[axis]) * invGyroComplimentaryFilterFactor;
    }

    if (EstG.A[Z] > smallAngle) {
        ENABLE_STATE(SMALL_ANGLE);
    } else {
        DISABLE_STATE(SMALL_ANGLE);
    }

    // Attitude of the estimated vector
    anglerad[AI_ROLL] = atan2f(EstG.V.Y, EstG.V.Z);
    anglerad[AI_PITCH] = atan2f(-EstG.V.X, sqrtf(EstG.V.Y * EstG.V.Y + EstG.V.Z * EstG.V.Z));
    inclination.values.rollDeciDegrees = lrintf(anglerad[AI_ROLL] * (1800.0f / M_PI));
    inclination.values.pitchDeciDegrees = lrintf(anglerad[AI_PITCH] * (1800.0f / M_PI));

    if (sensors(SENSOR_MAG)) {
        rotateV(&EstM.V, &deltaGyroAngle);
        // FIXME what does the _M_ mean?
        float invGyroComplimentaryFilter_M_Factor = (1.0f / (imuRuntimeConfig->gyro_cmpfm_factor + 1.0f));
        for (axis = 0; axis < 3; axis++) {
            EstM.A[axis] = (EstM.A[axis] * imuRuntimeConfig->gyro_cmpfm_factor + magADC[axis]) * invGyroComplimentaryFilter_M_Factor;
        }
        heading = calculateHeading(&EstM);
    } else {
        rotateV(&EstN.V, &deltaGyroAngle);
        normalizeV(&EstN.V, &EstN.V);
        heading = calculateHeading(&EstN);
    }

    acc_calc(deltaT); // rotate acc vector into earth frame
}

// correction of throttle in lateral wind,
int16_t calculateThrottleAngleCorrection(uint8_t throttle_correction_value)
{
    float cosZ = EstG.V.Z / sqrtf(EstG.V.X * EstG.V.X + EstG.V.Y * EstG.V.Y + EstG.V.Z * EstG.V.Z);

    if (cosZ <= 0.015f) { // we are inverted, vertical or with a small angle < 0.86 deg
        return 0;
    }
    int angle = lrintf(acosf(cosZ) * throttleAngleScale);
    if (angle > 900)
        angle = 900;
    return lrintf(throttle_correction_value * sinf(angle / (900.0f * M_PI / 2.0f)));
}

#ifdef BARO
// 40hz update rate (20hz LPF on acc)
#define BARO_UPDATE_FREQUENCY_40HZ (1000 * 25)

#define DEGREES_80_IN_DECIDEGREES 800

bool isThrustFacingDownwards(rollAndPitchInclination_t *inclination)
{
    return abs(inclination->values.rollDeciDegrees) < DEGREES_80_IN_DECIDEGREES && abs(inclination->values.pitchDeciDegrees) < DEGREES_80_IN_DECIDEGREES;
}

int16_t calculateTiltAngle(rollAndPitchInclination_t *inclination)
{
	return max(abs(inclination->values.rollDeciDegrees), abs(inclination->values.pitchDeciDegrees));
}

int32_t calculateBaroPid(int32_t vel_tmp, float accZ_tmp, float accZ_old)
{
    uint32_t newBaroPID = 0;
    int32_t error;
    int32_t setVel;

    if (!isThrustFacingDownwards(&inclination)) {
        return newBaroPID;
    }

    // Altitude P-Controller

    if (!velocityControl) {
        error = constrain(AltHold - EstAlt, -500, 500);
        error = applyDeadband(error, 10); // remove small P parameter to reduce noise near zero position
        setVel = constrain((pidProfile->P8[PIDALT] * error / 128), -300, +300); // limit velocity to +/- 3 m/s
    } else {
        setVel = setVelocity;
    }
    // Velocity PID-Controller

    // P
    error = setVel - vel_tmp;
    newBaroPID = constrain((pidProfile->P8[PIDVEL] * error / 32), -300, +300);

    // I
    errorVelocityI += (pidProfile->I8[PIDVEL] * error);
    errorVelocityI = constrain(errorVelocityI, -(8192 * 200), (8192 * 200));
    newBaroPID += errorVelocityI / 8192;     // I in range +/-200

    // D
    newBaroPID -= constrain(pidProfile->D8[PIDVEL] * (accZ_tmp + accZ_old) / 512, -150, 150);

    return newBaroPID;
}

void calculateEstimatedAltitude(uint32_t currentTime)
{
    static uint32_t previousTime;
    uint32_t dTime;
    int32_t baroVel;
    float dt;
    float vel_acc;
    int32_t vel_tmp;
    float accZ_tmp;
    static float accZ_old = 0.0f;
    static float vel = 0.0f;
    static float accAlt = 0.0f;
    static int32_t lastBaroAlt;

    static int32_t baroAlt_offset = 0;
    float sonarTransition;

#ifdef SONAR
    int16_t tiltAngle;
#endif



    dTime = currentTime - previousTime;
    if (dTime < BARO_UPDATE_FREQUENCY_40HZ)
        return;
        
    previousTime = currentTime;

    if (!isBaroCalibrationComplete()) {
        performBaroCalibrationCycle();
        vel = 0;
        accAlt = 0;
    }

    BaroAlt = baroCalculateAltitude();

#ifdef SONAR
    tiltAngle = calculateTiltAngle(&inclination);
    sonarAlt = sonarCalculateAltitude(sonarAlt, tiltAngle);
#endif

    if (sonarAlt > 0 && sonarAlt < 200) {
        baroAlt_offset = BaroAlt - sonarAlt;
        BaroAlt = sonarAlt;
    } else {
        BaroAlt -= baroAlt_offset;
        if (sonarAlt > 0) {
            sonarTransition = (300 - sonarAlt) / 100.0f;
            BaroAlt = sonarAlt * sonarTransition + BaroAlt * (1.0f - sonarTransition);
        }
    }

    dt = accTimeSum * 1e-6f; // delta acc reading time in seconds

    // Integrator - velocity, cm/sec
    accZ_tmp = (float)accSum[2] / (float)accSumCount;
    vel_acc = accZ_tmp * accVelScale * (float)accTimeSum;

    // Integrator - Altitude in cm
    accAlt += (vel_acc * 0.5f) * dt + vel * dt;                                                                 // integrate velocity to get distance (x= a/2 * t^2)
    accAlt = accAlt * barometerConfig->baro_cf_alt + (float)BaroAlt * (1.0f - barometerConfig->baro_cf_alt);    // complementary filter for altitude estimation (baro & acc)
    vel += vel_acc;

#if 0
    debug[1] = accSum[2] / accSumCount; // acceleration
    debug[2] = vel;                     // velocity
    debug[3] = accAlt;                  // height
#endif

    accSum_reset();

    if (!isBaroCalibrationComplete()) {
        return;
    }

    if (sonarAlt > 0 && sonarAlt < 200) {
        // the sonar has the best range
        EstAlt = BaroAlt;
    } else {
        EstAlt = accAlt;
    }

    baroVel = (BaroAlt - lastBaroAlt) * 1000000.0f / dTime;
    lastBaroAlt = BaroAlt;

    baroVel = constrain(baroVel, -1500, 1500);  // constrain baro velocity +/- 1500cm/s
    baroVel = applyDeadband(baroVel, 10);       // to reduce noise near zero

    // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity).
    // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay
    vel = vel * barometerConfig->baro_cf_vel + baroVel * (1.0f - barometerConfig->baro_cf_vel);
    vel_tmp = lrintf(vel);

    // set vario
    vario = applyDeadband(vel_tmp, 5);

    BaroPID = calculateBaroPid(vel_tmp, accZ_tmp, accZ_old);

    accZ_old = accZ_tmp;
}
#endif /* BARO */
