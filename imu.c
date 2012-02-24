#include "board.h"
#include "mw.h"

#define M_PI       3.14159265358979323846

int16_t gyroADC[3], accADC[3], accSmooth[3], magADC[3];
int16_t acc_25deg = 0;
int32_t pressure = 0;
int32_t  BaroAlt;
int32_t  EstAlt;             // in cm
int16_t  BaroPID = 0;
int32_t  AltHold;
int16_t  errorAltitudeI = 0;
int32_t  EstVelocity;

// **************
// gyro+acc IMU
// **************
int16_t gyroData[3] = { 0, 0, 0 };
int16_t gyroZero[3] = { 0, 0, 0 };
int16_t accZero[3] = { 0, 0, 0 };
int16_t magZero[3] = { 0, 0, 0 };
int16_t angle[2] = { 0, 0 };     // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
int8_t smallAngle25 = 1;

static void getEstimatedAttitude(void);

void imuInit(void)
{
    acc_25deg = acc_1G * 0.423;

    // if mag sensor is enabled, use it
    if (sensors(SENSOR_MAG))
        Mag_init();
}

void computeIMU(void)
{
    uint8_t axis;
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
        while ((micros() - timeInterleave) < 650);  //empirical, interleaving delay between 2 consecutive reads
    }

    Gyro_getADC();

    for (axis = 0; axis < 3; axis++) {
        gyroADCinter[axis] = gyroADC[axis] + gyroADCp[axis];
        // empirical, we take a weighted value of the current and the previous values
        gyroData[axis] = (gyroADCinter[axis] + gyroADCprevious[axis] + 1) / 3;
        gyroADCprevious[axis] = gyroADCinter[axis] / 2;
        if (!sensors(SENSOR_ACC))
            accADC[axis] = 0;
    }
    
    if (mixerConfiguration == MULTITYPE_TRI) {
        gyroData[YAW] = (gyroYawSmooth * 2 + gyroData[YAW] + 1) / 3;
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
/* Set the Low Pass Filter factor for ACC */
/* Increasing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time*/
/* Comment this if  you do not want filter at all.*/
/* Default WMC value: 8*/
#define ACC_LPF_FACTOR 4

/* Set the Low Pass Filter factor for Magnetometer */
/* Increasing this value would reduce Magnetometer noise (not visible in GUI), but would increase Magnetometer lag time*/
/* Comment this if  you do not want filter at all.*/
/* Default WMC value: n/a*/
//#define MG_LPF_FACTOR 4

/* Set the Gyro Weight for Gyro/Acc complementary filter */
/* Increasing this value would reduce and delay Acc influence on the output of the filter*/
/* Default WMC value: 300*/
#define GYR_CMPF_FACTOR 310.0f

/* Set the Gyro Weight for Gyro/Magnetometer complementary filter */
/* Increasing this value would reduce and delay Magnetometer influence on the output of the filter*/
/* Default WMC value: n/a*/
#define GYR_CMPFM_FACTOR 200.0f

//****** end of advanced users settings *************

#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / (GYR_CMPFM_FACTOR + 1.0f))
#define GYRO_SCALE ((2380 * M_PI)/((32767.0f / 4.0f ) * 180.0f * 1000000.0f))     //should be 2279.44 but 2380 gives better result
  // +-2000/sec deg scale
  //#define GYRO_SCALE ((200.0f * PI)/((32768.0f / 5.0f / 4.0f ) * 180.0f * 1000000.0f) * 1.5f)     
  // +- 200/sec deg scale
  // 1.5 is emperical, not sure what it means
  // should be in rad/sec

typedef struct fp_vector {
    float X;
    float Y;
    float Z;
} t_fp_vector_def;

typedef union {
    float A[3];
    t_fp_vector_def V;
} t_fp_vector;

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void rotateV(struct fp_vector *v, float *delta)
{
    struct fp_vector v_tmp = *v;
    v->Z -= delta[ROLL] * v_tmp.X + delta[PITCH] * v_tmp.Y;
    v->X += delta[ROLL] * v_tmp.Z - delta[YAW] * v_tmp.Y;
    v->Y += delta[PITCH] * v_tmp.Z + delta[YAW] * v_tmp.X;
}

static int16_t _atan2f(float y, float x)
{
    // no need for aidsy inaccurate shortcuts on a proper platform
    return (int16_t)(atan2f(y, x) * (180.0f / M_PI * 10.0f));
}

static void getEstimatedAttitude(void)
{
    uint8_t axis;
    int32_t accMag = 0;
    static t_fp_vector EstG;
    static t_fp_vector EstM;
#if defined(MG_LPF_FACTOR)
    static int16_t mgSmooth[3];
#endif
#if defined(ACC_LPF_FACTOR)
    static int16_t accTemp[3];  //projection of smoothed and normalized magnetic vector on x/y/z axis, as measured by magnetometer
#endif
    static uint32_t previousT;
    uint32_t currentT = micros();
    float scale, deltaGyroAngle[3];

    scale = (currentT - previousT) * GYRO_SCALE;
    previousT = currentT;

    // Initialization
    for (axis = 0; axis < 3; axis++) {
        deltaGyroAngle[axis] = gyroADC[axis] * scale;
#if defined(ACC_LPF_FACTOR)
        accTemp[axis] = (accTemp[axis] - (accTemp[axis] >> ACC_LPF_FACTOR)) + accADC[axis];
        accSmooth[axis] = accTemp[axis] >> ACC_LPF_FACTOR;
#define ACC_VALUE accSmooth[axis]
#else
        accSmooth[axis] = accADC[axis];
#define ACC_VALUE accADC[axis]
#endif
        accMag += (int32_t) ACC_VALUE * ACC_VALUE;
        
        if (sensors(SENSOR_MAG)) {
#if defined(MG_LPF_FACTOR)
            mgSmooth[axis] = (mgSmooth[axis] * (MG_LPF_FACTOR - 1) + magADC[axis]) / MG_LPF_FACTOR; // LPF for Magnetometer values
#define MAG_VALUE mgSmooth[axis]
#else
#define MAG_VALUE magADC[axis]
#endif
        }
    }
    accMag = accMag * 100 / ((int32_t) acc_1G * acc_1G);

    rotateV(&EstG.V, deltaGyroAngle);
    if (sensors(SENSOR_MAG)) {
        rotateV(&EstM.V, deltaGyroAngle);
    }

    if (abs(accSmooth[ROLL]) < acc_25deg && abs(accSmooth[PITCH]) < acc_25deg && accSmooth[YAW] > 0)
        smallAngle25 = 1;
    else
        smallAngle25 = 0;

    // Apply complimentary filter (Gyro drift correction)
    // If accel magnitude >1.4G or <0.6G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
    // To do that, we just skip filter, as EstV already rotated by Gyro
    if ((36 < accMag && accMag < 196) || smallAngle25)
        for (axis = 0; axis < 3; axis++) {
            int16_t acc = ACC_VALUE;
#if !defined(TRUSTED_ACCZ)
            if (smallAngle25 && axis == YAW)
                //We consider ACCZ = acc_1G when the acc on other axis is small.
                //It's a tweak to deal with some configs where ACC_Z tends to a value < acc_1G when high throttle is applied.
                //This tweak applies only when the multi is not in inverted position
                acc = acc_1G;
#endif
            EstG.A[axis] = (EstG.A[axis] * GYR_CMPF_FACTOR + acc) * INV_GYR_CMPF_FACTOR;
        }

    if (sensors(SENSOR_MAG)) {
        for (axis = 0; axis < 3; axis++)
            EstM.A[axis] = (EstM.A[axis] * GYR_CMPFM_FACTOR + MAG_VALUE) * INV_GYR_CMPFM_FACTOR;
    }

    // Attitude of the estimated vector
    angle[ROLL] = _atan2f(EstG.V.X, EstG.V.Z);
    angle[PITCH] = _atan2f(EstG.V.Y, EstG.V.Z);

    if (sensors(SENSOR_MAG)) {
        // Attitude of the cross product vector GxM
        heading = _atan2f(EstG.V.X * EstM.V.Z - EstG.V.Z * EstM.V.X, EstG.V.Z * EstM.V.Y - EstG.V.Y * EstM.V.Z) / 10;
    }
}

float InvSqrt(float x)
{
    union {
        int32_t i;
        float f;
    } conv;
    conv.f = x;
    conv.i = 0x5f3759df - (conv.i >> 1);
    return 0.5f * conv.f * (3.0f - x * conv.f * conv.f);
}

int32_t isq(int32_t x)
{
    return x * x;
}

#define UPDATE_INTERVAL 25000   // 40hz update rate (20hz LPF on acc)
#define INIT_DELAY      4000000 // 4 sec initialization delay
#define BARO_TAB_SIZE   40
#define Kp1 5.5f                // PI observer velocity gain
#define Kp2 10.0f               // PI observer position gain
#define Ki  0.01f               // PI observer integral gain (bias cancellation)
#define dt  (UPDATE_INTERVAL / 1000000.0f)

void getEstimatedAltitude(void)
{
    static uint8_t inited = 0;
    static uint32_t deadLine = INIT_DELAY;
    static int16_t BaroHistTab[BARO_TAB_SIZE];
    static int8_t BaroHistIdx = 0;
    int32_t BaroHigh, BaroLow;
    int32_t temp32;

    if (currentTime < deadLine)
        return;
    deadLine = currentTime + UPDATE_INTERVAL;

    if (!inited) {
        inited = 1;
        EstAlt = BaroAlt;
    }

    //**** Alt. Set Point stabilization PID ****
    //calculate speed for D calculation
    BaroHistTab[BaroHistIdx] = BaroAlt;  
    BaroHigh = 0;
    BaroLow = 0;
    BaroPID = 0;
    for (temp32 = 0; temp32 < BARO_TAB_SIZE / 2; temp32++) {
        BaroHigh += BaroHistTab[(BaroHistIdx - temp32 + BARO_TAB_SIZE) % BARO_TAB_SIZE];  // sum last half samples
        BaroLow += BaroHistTab[(BaroHistIdx + temp32 + BARO_TAB_SIZE) % BARO_TAB_SIZE];  // sum older samples
    }
    BaroHistIdx++;
    if (BaroHistIdx >= BARO_TAB_SIZE)
        BaroHistIdx = 0;

    temp32 = D8[PIDALT] * (BaroHigh - BaroLow) / 400;
    BaroPID -= temp32;
    
    EstAlt = BaroHigh / (BARO_TAB_SIZE / 2);

    temp32 = constrain(AltHold - EstAlt, -1000, 1000);
    if (abs(temp32) < 10 && BaroPID < 10)
        BaroPID = 0;  // remove small D parametr to reduce noise near zoro position
    // P
    BaroPID += P8[PIDALT] * constrain(temp32, (-2) * P8[PIDALT], 2 * P8[PIDALT]) / 100;
    BaroPID = constrain(BaroPID, -150, +150); // sum of P and D should be in range 150
    
    // I
    errorAltitudeI += temp32 * I8[PIDALT] / 50;
    errorAltitudeI = constrain(errorAltitudeI, -30000, 30000);
    temp32 = errorAltitudeI / 500; // I in range +/-60
    BaroPID += temp32;
}
