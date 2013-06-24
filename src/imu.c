#include "board.h"
#include "mw.h"

int16_t gyroADC[3], accADC[3], accSmooth[3], magADC[3];
int16_t acc_25deg = 0;
int32_t baroPressure = 0;
int32_t baroTemperature = 0;
int32_t baroPressureSum = 0;
int32_t BaroAlt = 0;
int16_t sonarAlt;              // to think about the unit
int32_t EstAlt;                // in cm
int16_t BaroPID = 0;
int32_t AltHold;
int16_t errorAltitudeI = 0;
int16_t vario = 0;              // variometer in cm/s
float magneticDeclination = 0.0f; // calculated at startup from config
float accVelScale;

// **************
// gyro+acc IMU
// **************
int16_t gyroData[3] = { 0, 0, 0 };
int16_t gyroZero[3] = { 0, 0, 0 };
int16_t angle[2] = { 0, 0 };     // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800

static void getEstimatedAttitude(void);

void imuInit(void)
{
    acc_25deg = acc_1G * 0.423f;
    accVelScale = 9.80665f / acc_1G / 10000.0f;

#ifdef MAG
    // if mag sensor is enabled, use it
    if (sensors(SENSOR_MAG))
        Mag_init();
#endif
}


void computeIMU(void)
{
    uint32_t axis;
    static int16_t gyroADCprevious[3] = { 0, 0, 0 };
    int16_t gyroADCp[3];
    int16_t gyroADCinter[3];
    static uint32_t timeInterleave = 0;
    static int16_t gyroYawSmooth = 0;

    if (sensors(SENSOR_ACC)) {
        ACC_getADC();
        getEstimatedAttitude();
    }

    Gyro_getADC();

    for (axis = 0; axis < 3; axis++)
        gyroADCp[axis] = gyroADC[axis];
    timeInterleave = micros();
    annexCode();

    if ((micros() - timeInterleave) > 650) {
        annex650_overrun_count++;
    } else {
        while ((micros() - timeInterleave) < 650);  // empirical, interleaving delay between 2 consecutive reads
    }

    Gyro_getADC();
    for (axis = 0; axis < 3; axis++) {
        gyroADCinter[axis] = gyroADC[axis] + gyroADCp[axis];
        // empirical, we take a weighted value of the current and the previous values
        gyroData[axis] = (gyroADCinter[axis] + gyroADCprevious[axis]) / 3;
        gyroADCprevious[axis] = gyroADCinter[axis] / 2;
        if (!sensors(SENSOR_ACC))
            accADC[axis] = 0;
    }

    if (feature(FEATURE_GYRO_SMOOTHING)) {
        static uint8_t Smoothing[3] = { 0, 0, 0 };
        static int16_t gyroSmooth[3] = { 0, 0, 0 };
        if (Smoothing[0] == 0) {
            // initialize
            Smoothing[ROLL] = (mcfg.gyro_smoothing_factor >> 16) & 0xff;
            Smoothing[PITCH] = (mcfg.gyro_smoothing_factor >> 8) & 0xff;
            Smoothing[YAW] = (mcfg.gyro_smoothing_factor) & 0xff;
        }
        for (axis = 0; axis < 3; axis++) {
            gyroData[axis] = (int16_t)(((int32_t)((int32_t)gyroSmooth[axis] * (Smoothing[axis] - 1)) + gyroData[axis] + 1 ) / Smoothing[axis]);
            gyroSmooth[axis] = gyroData[axis];
        }
    } else if (mcfg.mixerConfiguration == MULTITYPE_TRI) {
        gyroData[YAW] = (gyroYawSmooth * 2 + gyroData[YAW]) / 3;
        gyroYawSmooth = gyroData[YAW];
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
// 2) Small-angle approximation: http://en.wikipedia.org/wiki/Small-angle_approximation
// 3) C. Hastings approximation for atan2()
// 4) Optimization tricks: http://www.hackersdelight.org/
//
// Currently Magnetometer uses separate CF which is used only
// for heading approximation.
//
// Modified: 19/04/2011  by ziss_dm
// Version: V1.1
//
// code size deduction and tmp vector intermediate step for vector rotation computation: October 2011 by Alex
// **************************************************

//******  advanced users settings *******************

/* Set the Gyro Weight for Gyro/Magnetometer complementary filter */
/* Increasing this value would reduce and delay Magnetometer influence on the output of the filter*/
/* Default WMC value: n/a*/
#define GYR_CMPFM_FACTOR 200.0f

//****** end of advanced users settings *************

#define INV_GYR_CMPF_FACTOR   (1.0f / ((float)mcfg.gyro_cmpf_factor + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / ((float)mcfg.gyro_cmpfm_factor + 1.0f))

// #define GYRO_SCALE ((1998 * M_PI)/((32767.0f / 4.0f ) * 180.0f * 1000000.0f))     // 32767 / 16.4lsb/dps for MPU3000

typedef struct fp_vector {
    float X;
    float Y;
    float Z;
} t_fp_vector_def;

typedef union {
    float A[3];
    t_fp_vector_def V;
} t_fp_vector;

t_fp_vector EstG;

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void rotateV(struct fp_vector *v, float *delta)
{
    struct fp_vector v_tmp = *v;

    // This does a  "proper" matrix rotation using gyro deltas without small-angle approximation
    float mat[3][3];
    float cosx, sinx, cosy, siny, cosz, sinz;
    float coszcosx, coszcosy, sinzcosx, coszsinx, sinzsinx;

    cosx = cosf(-delta[PITCH]);
    sinx = sinf(-delta[PITCH]);
    cosy = cosf(delta[ROLL]);
    siny = sinf(delta[ROLL]);
    cosz = cosf(delta[YAW]);
    sinz = sinf(delta[YAW]);

    coszcosx = cosz * cosx;
    coszcosy = cosz * cosy;
    sinzcosx = sinz * cosx;
    coszsinx = sinx * cosz;
    sinzsinx = sinx * sinz;

    mat[0][0] = coszcosy;
    mat[0][1] = sinz * cosy;
    mat[0][2] = -siny;
    mat[1][0] = (coszsinx * siny) - sinzcosx;
    mat[1][1] = (sinzsinx * siny) + (coszcosx);
    mat[1][2] = cosy * sinx;
    mat[2][0] = (coszcosx * siny) + (sinzsinx);
    mat[2][1] = (sinzcosx * siny) - (coszsinx);
    mat[2][2] = cosy * cosx;

    v->X = v_tmp.X * mat[0][0] + v_tmp.Y * mat[1][0] + v_tmp.Z * mat[2][0];
    v->Y = v_tmp.X * mat[0][1] + v_tmp.Y * mat[1][1] + v_tmp.Z * mat[2][1];
    v->Z = v_tmp.X * mat[0][2] + v_tmp.Y * mat[1][2] + v_tmp.Z * mat[2][2];
}

static int16_t _atan2f(float y, float x)
{
    // no need for aidsy inaccurate shortcuts on a proper platform
    return (int16_t)(atan2f(y, x) * (180.0f / M_PI * 10.0f));
}

// Use original baseflight angle calculation
// #define BASEFLIGHT_CALC
static float invG;

static void getEstimatedAttitude(void)
{
    uint32_t axis;
    int32_t accMag = 0;
    static t_fp_vector EstM;
    static float accLPF[3];
    static uint32_t previousT;
    uint32_t currentT = micros();
    float scale, deltaGyroAngle[3];
#ifndef BASEFLIGHT_CALC
    float sqGZ;
    float sqGX;
    float sqGY;
    float sqGX_sqGZ;
    float invmagXZ;
#endif

    scale = (currentT - previousT) * gyro.scale;
    previousT = currentT;

    // Initialization
    for (axis = 0; axis < 3; axis++) {
        deltaGyroAngle[axis] = gyroADC[axis] * scale;
        if (cfg.acc_lpf_factor > 0) {
            accLPF[axis] = accLPF[axis] * (1.0f - (1.0f / cfg.acc_lpf_factor)) + accADC[axis] * (1.0f / cfg.acc_lpf_factor);
            accSmooth[axis] = accLPF[axis];
        } else {
            accSmooth[axis] = accADC[axis];
        }
        accMag += (int32_t)accSmooth[axis] * accSmooth[axis];
    }
    accMag = accMag * 100 / ((int32_t)acc_1G * acc_1G);

    rotateV(&EstG.V, deltaGyroAngle);
    if (sensors(SENSOR_MAG))
        rotateV(&EstM.V, deltaGyroAngle);

    if (abs(accSmooth[ROLL]) < acc_25deg && abs(accSmooth[PITCH]) < acc_25deg && accSmooth[YAW] > 0)
        f.SMALL_ANGLES_25 = 1;
    else
        f.SMALL_ANGLES_25 = 0;

    // Apply complimentary filter (Gyro drift correction)
    // If accel magnitude >1.15G or <0.85G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
    // To do that, we just skip filter, as EstV already rotated by Gyro
    if (72 < accMag && accMag < 133) {
        for (axis = 0; axis < 3; axis++)
            EstG.A[axis] = (EstG.A[axis] * (float)mcfg.gyro_cmpf_factor + accSmooth[axis]) * INV_GYR_CMPF_FACTOR;
    }

    if (sensors(SENSOR_MAG)) {
        for (axis = 0; axis < 3; axis++)
            EstM.A[axis] = (EstM.A[axis] * (float)mcfg.gyro_cmpfm_factor + magADC[axis]) * INV_GYR_CMPFM_FACTOR;
    }

    // Attitude of the estimated vector
#ifdef BASEFLIGHT_CALC
    // This hack removes gimbal lock (sorta) on pitch, so rolling around doesn't make pitch jump when roll reaches 90deg
    angle[ROLL] = _atan2f(EstG.V.X, EstG.V.Z);
    angle[PITCH] = -asinf(EstG.V.Y / -sqrtf(EstG.V.X * EstG.V.X + EstG.V.Y * EstG.V.Y + EstG.V.Z * EstG.V.Z)) * (180.0f / M_PI * 10.0f);
#else
    // MW2.2 version
    sqGZ = EstG.V.Z * EstG.V.Z;
    sqGX = EstG.V.X * EstG.V.X;
    sqGY = EstG.V.Y * EstG.V.Y;
    sqGX_sqGZ = sqGX + sqGZ;
    invmagXZ = 1.0f / sqrtf(sqGX_sqGZ);
    invG = 1.0f / sqrtf(sqGX_sqGZ + sqGY);
    angle[ROLL] = _atan2f(EstG.V.X, EstG.V.Z);
    angle[PITCH] = _atan2f(EstG.V.Y, invmagXZ * sqGX_sqGZ);
#endif

#ifdef MAG
    if (sensors(SENSOR_MAG)) {
#ifdef BASEFLIGHT_CALC
        // baseflight calculation
        float rollRAD = (float)angle[ROLL] * RADX10;
        float pitchRAD = -(float)angle[PITCH] * RADX10;
        float magX = EstM.A[1];                         // Swap X/Y
        float magY = EstM.A[0];                         // Swap X/Y
        float magZ = EstM.A[2];
        float cr = cosf(rollRAD);
        float sr = sinf(rollRAD);
        float cp = cosf(pitchRAD);
        float sp = sinf(pitchRAD);
        float Xh = magX * cp + magY * sr * sp + magZ * cr * sp;
        float Yh = magY * cr - magZ * sr;
        float hd = (atan2f(-Yh, Xh) * 1800.0f / M_PI + magneticDeclination) / 10.0f;
        heading = hd;
#else
        // MW 2.2 calculation
        heading = _atan2f(EstM.V.Z * EstG.V.X - EstM.V.X * EstG.V.Z, EstM.V.Y * invG * sqGX_sqGZ - (EstM.V.X * EstG.V.X + EstM.V.Z * EstG.V.Z) * invG * EstG.V.Y);
        heading = heading + magneticDeclination;
        heading = heading / 10;
#endif
        if (heading > 180)
            heading = heading - 360;
        else if (heading < -180)
            heading = heading + 360;
    }
#endif
}

#ifdef BARO
#define UPDATE_INTERVAL 25000   // 40hz update rate (20hz LPF on acc)
#define INIT_DELAY      4000000 // 4 sec initialization delay

int16_t applyDeadband(int16_t value, int16_t deadband)
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

int getEstimatedAltitude(void)
{
    static int32_t baroGroundPressure;
    static uint32_t previousT;
    uint32_t currentT = micros();
    uint32_t dTime;
    int16_t error16;
    int16_t baroVel;
    int16_t accZ;
    static int16_t accZoffset = 0;
    static float vel = 0.0f;
    static int32_t lastBaroAlt;
    int16_t vel_tmp;

    dTime = currentT - previousT;
    if (dTime < UPDATE_INTERVAL)
        return 0;
    previousT = currentT;

    if (calibratingB > 0) {
        baroGroundPressure = baroPressureSum / (cfg.baro_tab_size - 1);
        calibratingB--;
    }

    // pressure relative to ground pressure with temperature compensation (fast!)
    // baroGroundPressure is not supposed to be 0 here
    // see: https://code.google.com/p/ardupilot-mega/source/browse/libraries/AP_Baro/AP_Baro.cpp
    BaroAlt = logf(baroGroundPressure * (cfg.baro_tab_size - 1) / (float)baroPressureSum) * (baroTemperature + 27315) * 29.271267f; // in cemtimeter 
    EstAlt = (EstAlt * 6 + BaroAlt * 2) >> 3;   // additional LPF to reduce baro noise

    //P
    error16 = constrain(AltHold - EstAlt, -300, 300);
    error16 = applyDeadband(error16, 10); // remove small P parametr to reduce noise near zero position
    BaroPID = constrain((cfg.P8[PIDALT] * error16 >> 7), -150, +150);

    //I
    errorAltitudeI += cfg.I8[PIDALT] * error16 >> 6;
    errorAltitudeI = constrain(errorAltitudeI, -30000, 30000);
    BaroPID += errorAltitudeI >> 9;     // I in range +/-60

    // projection of ACC vector to global Z, with 1G subtructed
    // Math: accZ = A * G / |G| - 1G (invG is calculated in getEstimatedAttitude)
    accZ = (accSmooth[ROLL] * EstG.V.X + accSmooth[PITCH] * EstG.V.Y + accSmooth[YAW] * EstG.V.Z) * invG;

    if (!f.ARMED) {
        accZoffset -= accZoffset >> 3;
        accZoffset += accZ;
    }
    accZ -= accZoffset >> 3;
    accZ = applyDeadband(accZ, cfg.accz_deadband);

    // Integrator - velocity, cm/sec
    vel += accZ * accVelScale * dTime;

    baroVel = (EstAlt - lastBaroAlt) * 1000000.0f / dTime;
    lastBaroAlt = EstAlt;

    baroVel = constrain(baroVel, -300, 300);    // constrain baro velocity +/- 300cm/s
    baroVel = applyDeadband(baroVel, 10); // to reduce noise near zero

    // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity). 
    // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay
    vel = vel * 0.985f + baroVel * 0.015f;

    // D
    vel_tmp = vel;
    vel_tmp = applyDeadband(vel_tmp, 5);
    vario = vel_tmp;
    BaroPID -= constrain(cfg.D8[PIDALT] * vel_tmp >> 4, -150, 150);

    return 1;
}
#endif /* BARO */
