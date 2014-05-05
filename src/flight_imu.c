// Inertial Measurement Unit (IMU)

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "common/maths.h"

#include <platform.h>

#include "common/axis.h"
#include "flight_common.h"

#include "drivers/system_common.h"

#include "sensors_common.h"
#include "drivers/accgyro_common.h"
#include "sensors_gyro.h"
#include "sensors_compass.h"
#include "sensors_acceleration.h"
#include "sensors_barometer.h"

#include "gps_common.h"

#include "gimbal.h"
#include "flight_mixer.h"

// FIXME remove dependency on config.h
#include "boardalignment.h"
#include "battery.h"
#include "escservo.h"
#include "rc_controls.h"
#include "rx_common.h"
#include "telemetry_common.h"
#include "drivers/serial_common.h"
#include "serial_common.h"
#include "failsafe.h"
#include "runtime_config.h"
#include "config.h"
#include "config_profile.h"
#include "config_master.h"


int16_t gyroADC[XYZ_AXIS_COUNT], accADC[XYZ_AXIS_COUNT], accSmooth[XYZ_AXIS_COUNT];
int32_t accSum[XYZ_AXIS_COUNT];

uint32_t accTimeSum = 0;        // keep track for integration of acc
int accSumCount = 0;
float accVelScale;

int16_t smallAngle = 0;

int32_t EstAlt;                // in cm
int32_t AltHold;
int32_t errorAltitudeI = 0;

int32_t vario = 0;                      // variometer in cm/s

int16_t throttleAngleCorrection = 0;    // correction of throttle in lateral wind,
float throttleAngleScale;

int32_t BaroPID = 0;

float magneticDeclination = 0.0f;       // calculated at startup from config


// **************
// gyro+acc IMU
// **************
int16_t gyroData[FLIGHT_DYNAMICS_INDEX_COUNT] = { 0, 0, 0 };
int16_t gyroZero[FLIGHT_DYNAMICS_INDEX_COUNT] = { 0, 0, 0 };

rollAndPitchInclination_t inclination = { { 0, 0 } };     // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
float anglerad[2] = { 0.0f, 0.0f };    // absolute angle inclination in radians

static void getEstimatedAttitude(void);

void imuInit(void)
{
    smallAngle = lrintf(acc_1G * cosf(RAD * 25.0f));
    accVelScale = 9.80665f / acc_1G / 10000.0f;
    throttleAngleScale = (1800.0f / M_PI) * (900.0f / currentProfile.throttle_correction_angle);

#ifdef MAG
    // if mag sensor is enabled, use it
    if (sensors(SENSOR_MAG))
        compassInit();
#endif
}

void computeIMU(void)
{
    uint32_t axis;
    static int16_t gyroYawSmooth = 0;

    gyroGetADC();
    if (sensors(SENSOR_ACC)) {
        updateAccelerationReadings(&currentProfile.accelerometerTrims);
        getEstimatedAttitude();
    } else {
        accADC[X] = 0;
        accADC[Y] = 0;
        accADC[Z] = 0;
    }

    if (masterConfig.mixerConfiguration == MULTITYPE_TRI) {
        gyroData[FD_YAW] = (gyroYawSmooth * 2 + gyroADC[FD_YAW]) / 3;
        gyroYawSmooth = gyroData[FD_YAW];
        gyroData[FD_ROLL] = gyroADC[FD_ROLL];
        gyroData[FD_PITCH] = gyroADC[FD_PITCH];
    } else {
        for (axis = 0; axis < 3; axis++)
            gyroData[axis] = gyroADC[axis];
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
    float coszcosx, coszcosy, sinzcosx, coszsinx, sinzsinx;

    cosx = cosf(delta->angles.roll);
    sinx = sinf(delta->angles.roll);
    cosy = cosf(delta->angles.pitch);
    siny = sinf(delta->angles.pitch);
    cosz = cosf(delta->angles.yaw);
    sinz = sinf(delta->angles.yaw);

    coszcosx = cosz * cosx;
    coszcosy = cosz * cosy;
    sinzcosx = sinz * cosx;
    coszsinx = sinx * cosz;
    sinzsinx = sinx * sinz;

    mat[0][0] = coszcosy;
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

#define F_CUT_ACCZ 10.0f // 10Hz should still be fast enough
static const float fc_acc = 0.5f / (M_PI * F_CUT_ACCZ);

// rotate acc into Earth frame and calculate acceleration in it
void acc_calc(uint32_t deltaT)
{
    static int32_t accZoffset = 0;
    static float accz_smooth;
    fp_angles_t rpy;
    t_fp_vector accel_ned;

    // the accel values have to be rotated into the earth frame
    rpy.angles.roll = -(float)anglerad[AI_ROLL];
    rpy.angles.pitch = -(float)anglerad[AI_PITCH];
    rpy.angles.yaw = -(float)heading * RAD;

    accel_ned.V.X = accSmooth[0];
    accel_ned.V.Y = accSmooth[1];
    accel_ned.V.Z = accSmooth[2];

    rotateV(&accel_ned.V, &rpy);

    if (currentProfile.acc_unarmedcal == 1) {
        if (!f.ARMED) {
            accZoffset -= accZoffset / 64;
            accZoffset += accel_ned.V.Z;
        }
        accel_ned.V.Z -= accZoffset / 64;  // compensate for gravitation on z-axis
    } else
        accel_ned.V.Z -= acc_1G;

    accz_smooth = accz_smooth + (deltaT / (fc_acc + deltaT)) * (accel_ned.V.Z - accz_smooth); // low pass filter

    // apply Deadband to reduce integration drift and vibration influence
    accSum[X] += applyDeadband(lrintf(accel_ned.V.X), currentProfile.accxy_deadband);
    accSum[Y] += applyDeadband(lrintf(accel_ned.V.Y), currentProfile.accxy_deadband);
    accSum[Z] += applyDeadband(lrintf(accz_smooth), currentProfile.accz_deadband);

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
    scale = deltaT * gyro.scale;
    previousT = currentT;

    // Initialization
    for (axis = 0; axis < 3; axis++) {
        deltaGyroAngle.raw[axis] = gyroADC[axis] * scale;
        if (currentProfile.acc_lpf_factor > 0) {
            accLPF[axis] = accLPF[axis] * (1.0f - (1.0f / currentProfile.acc_lpf_factor)) + accADC[axis] * (1.0f / currentProfile.acc_lpf_factor);
            accSmooth[axis] = accLPF[axis];
        } else {
            accSmooth[axis] = accADC[axis];
        }
        accMag += (int32_t)accSmooth[axis] * accSmooth[axis];
    }
    accMag = accMag * 100 / ((int32_t)acc_1G * acc_1G);

    rotateV(&EstG.V, &deltaGyroAngle);
    if (sensors(SENSOR_MAG)) {
        rotateV(&EstM.V, &deltaGyroAngle);
    } else {
        rotateV(&EstN.V, &deltaGyroAngle);
        normalizeV(&EstN.V, &EstN.V);
    }

    // Apply complimentary filter (Gyro drift correction)
    // If accel magnitude >1.15G or <0.85G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
    // To do that, we just skip filter, as EstV already rotated by Gyro

    float invGyroComplimentaryFilterFactor = (1.0f / ((float)masterConfig.gyro_cmpf_factor + 1.0f));

    if (72 < (uint16_t)accMag && (uint16_t)accMag < 133) {
        for (axis = 0; axis < 3; axis++)
            EstG.A[axis] = (EstG.A[axis] * (float)masterConfig.gyro_cmpf_factor + accSmooth[axis]) * invGyroComplimentaryFilterFactor;
    }

    // FIXME what does the _M_ mean?
    float invGyroComplimentaryFilter_M_Factor = (1.0f / ((float)masterConfig.gyro_cmpfm_factor + 1.0f));

    if (sensors(SENSOR_MAG)) {
        for (axis = 0; axis < 3; axis++)
            EstM.A[axis] = (EstM.A[axis] * (float)masterConfig.gyro_cmpfm_factor + magADC[axis]) * invGyroComplimentaryFilter_M_Factor;
    }

    f.SMALL_ANGLE = (EstG.A[Z] > smallAngle);

    // Attitude of the estimated vector
    anglerad[AI_ROLL] = atan2f(EstG.V.Y, EstG.V.Z);
    anglerad[AI_PITCH] = atan2f(-EstG.V.X, sqrtf(EstG.V.Y * EstG.V.Y + EstG.V.Z * EstG.V.Z));
    inclination.angle.rollDeciDegrees = lrintf(anglerad[AI_ROLL] * (1800.0f / M_PI));
    inclination.angle.pitchDeciDegrees = lrintf(anglerad[AI_PITCH] * (1800.0f / M_PI));

    if (sensors(SENSOR_MAG))
        heading = calculateHeading(&EstM);
    else
        heading = calculateHeading(&EstN);

    acc_calc(deltaT); // rotate acc vector into earth frame

    if (currentProfile.throttle_correction_value) {

        float cosZ = EstG.V.Z / sqrtf(EstG.V.X * EstG.V.X + EstG.V.Y * EstG.V.Y + EstG.V.Z * EstG.V.Z);

        if (cosZ <= 0.015f) { // we are inverted, vertical or with a small angle < 0.86 deg
            throttleAngleCorrection = 0;
        } else {
            int angle = lrintf(acosf(cosZ) * throttleAngleScale);
            if (angle > 900)
                angle = 900;
            throttleAngleCorrection = lrintf(currentProfile.throttle_correction_value * sinf(angle / (900.0f * M_PI / 2.0f))) ;
        }

    }
}

#ifdef BARO
#define UPDATE_INTERVAL 25000   // 40hz update rate (20hz LPF on acc)


#define DEGREES_80_IN_DECIDEGREES 800

bool isThrustFacingDownwards(rollAndPitchInclination_t *inclination)
{
    return abs(inclination->angle.rollDeciDegrees) < DEGREES_80_IN_DECIDEGREES && abs(inclination->angle.pitchDeciDegrees) < DEGREES_80_IN_DECIDEGREES;
}

int32_t calculateBaroPid(int32_t vel_tmp, float accZ_tmp, float accZ_old)
{
    uint32_t baroPID = 0;
    int32_t error;
    int32_t setVel;

    if (!isThrustFacingDownwards(&inclination)) {
        return baroPID;
    }

    // Altitude P-Controller

    error = constrain(AltHold - EstAlt, -500, 500);
    error = applyDeadband(error, 10);       // remove small P parametr to reduce noise near zero position
    setVel = constrain((currentProfile.pidProfile.P8[PIDALT] * error / 128), -300, +300); // limit velocity to +/- 3 m/s

    // Velocity PID-Controller

    // P
    error = setVel - vel_tmp;
    baroPID = constrain((currentProfile.pidProfile.P8[PIDVEL] * error / 32), -300, +300);

    // I
    errorAltitudeI += (currentProfile.pidProfile.I8[PIDVEL] * error) / 8;
    errorAltitudeI = constrain(errorAltitudeI, -(1024 * 200), (1024 * 200));
    baroPID += errorAltitudeI / 1024;     // I in range +/-200

    // D
    baroPID -= constrain(currentProfile.pidProfile.D8[PIDVEL] * (accZ_tmp + accZ_old) / 64, -150, 150);

    return baroPID;
}

int getEstimatedAltitude(void)
{
    static uint32_t previousT;
    uint32_t currentT = micros();
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

    dTime = currentT - previousT;
    if (dTime < UPDATE_INTERVAL)
        return 0;
    previousT = currentT;

    if (!isBaroCalibrationComplete()) {
        performBaroCalibrationCycle();
        vel = 0;
        accAlt = 0;
    }
    BaroAlt = baroCalculateAltitude();

    dt = accTimeSum * 1e-6f; // delta acc reading time in seconds

    // Integrator - velocity, cm/sec
    accZ_tmp = (float)accSum[2] / (float)accSumCount;
    vel_acc = accZ_tmp * accVelScale * (float)accTimeSum;

    // Integrator - Altitude in cm
    accAlt += (vel_acc * 0.5f) * dt + vel * dt;                                         // integrate velocity to get distance (x= a/2 * t^2)
    accAlt = accAlt * currentProfile.barometerConfig.baro_cf_alt + (float)BaroAlt * (1.0f - currentProfile.barometerConfig.baro_cf_alt);      // complementary filter for Altitude estimation (baro & acc)
    EstAlt = accAlt;
    vel += vel_acc;

#if 0
    debug[0] = accSum[2] / accSumCount; // acceleration
    debug[1] = vel;                     // velocity
    debug[2] = accAlt;                  // height
#endif

    accSum_reset();

    baroVel = (BaroAlt - lastBaroAlt) * 1000000.0f / dTime;
    lastBaroAlt = BaroAlt;

    baroVel = constrain(baroVel, -300, 300);    // constrain baro velocity +/- 300cm/s
    baroVel = applyDeadband(baroVel, 10);       // to reduce noise near zero

    // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity).
    // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay
    vel = vel * currentProfile.barometerConfig.baro_cf_vel + baroVel * (1 - currentProfile.barometerConfig.baro_cf_vel);
    vel_tmp = lrintf(vel);

    // set vario
    vario = applyDeadband(vel_tmp, 5);

    BaroPID = calculateBaroPid(vel_tmp, accZ_tmp, accZ_old);
    accZ_old = accZ_tmp;

    return 1;
}
#endif /* BARO */
