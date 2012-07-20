#include "board.h"
#include "mw.h"

int16_t gyroADC[3], accADC[3], accSmooth[3], magADC[3];
int16_t acc_25deg = 0;
int32_t  BaroAlt;
int16_t  sonarAlt;           //to think about the unit
int32_t  EstAlt;             // in cm
int16_t  BaroPID = 0;
int32_t  AltHold;
int16_t  errorAltitudeI = 0;
float magneticDeclination = 0.0f; // calculated at startup from config

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

#define GYRO_INTERLEAVE

    if (sensors(SENSOR_ACC)) {
        ACC_getADC();
        getEstimatedAttitude();
    }

    Gyro_getADC();

    for (axis = 0; axis < 3; axis++) {
#ifdef GYRO_INTERLEAVE
        gyroADCp[axis] = gyroADC[axis];
#else
        gyroData[axis] = gyroADC[axis];
#endif
        if (!sensors(SENSOR_ACC))
            accADC[axis] = 0;
    }
    timeInterleave = micros();
    annexCode();
#ifdef GYRO_INTERLEAVE
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
#endif

    if (feature(FEATURE_GYRO_SMOOTHING)) {
        static uint8_t Smoothing[3] = { 0, 0, 0 };
        static int16_t gyroSmooth[3] = { 0, 0, 0 };
        if (Smoothing[0] == 0) {
            // initialize
            Smoothing[ROLL] = (cfg.gyro_smoothing_factor >> 16) & 0xff;
            Smoothing[PITCH] = (cfg.gyro_smoothing_factor >> 8) & 0xff;
            Smoothing[YAW] = (cfg.gyro_smoothing_factor) & 0xff;
        }
        for (axis = 0; axis < 3; axis++) {
            gyroData[axis] = (int16_t)(((int32_t)((int32_t)gyroSmooth[axis] * (Smoothing[axis] - 1)) + gyroData[axis] + 1 ) / Smoothing[axis]);
            gyroSmooth[axis] = gyroData[axis];
        }
    } else if (cfg.mixerConfiguration == MULTITYPE_TRI) {
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

/* Set the Low Pass Filter factor for Magnetometer */
/* Increasing this value would reduce Magnetometer noise (not visible in GUI), but would increase Magnetometer lag time*/
/* Comment this if  you do not want filter at all.*/
/* Default WMC value: n/a*/
//#define MG_LPF_FACTOR 4

/* Set the Gyro Weight for Gyro/Magnetometer complementary filter */
/* Increasing this value would reduce and delay Magnetometer influence on the output of the filter*/
/* Default WMC value: n/a*/
#define GYR_CMPFM_FACTOR 200.0f

//****** end of advanced users settings *************

#define INV_GYR_CMPF_FACTOR   (1.0f / ((float)cfg.gyro_cmpf_factor + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / (GYR_CMPFM_FACTOR + 1.0f))

#define GYRO_SCALE ((1998 * M_PI)/((32767.0f / 4.0f ) * 180.0f * 1000000.0f))     // 32767 / 16.4lsb/dps for MPU3000

// #define GYRO_SCALE ((2380 * M_PI)/((32767.0f / 4.0f ) * 180.0f * 1000000.0f))     //should be 2279.44 but 2380 gives better result (ITG-3200)
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
    uint32_t axis;
    int32_t accMag = 0;
    static t_fp_vector EstG;
    static t_fp_vector EstM;
#if defined(MG_LPF_FACTOR)
    static int16_t mgSmooth[3];
#endif
    static float accLPF[3];
    static uint32_t previousT;
    uint32_t currentT = micros();
    float scale, deltaGyroAngle[3];

    scale = (currentT - previousT) * GYRO_SCALE;
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

        if (sensors(SENSOR_MAG)) {
#if defined(MG_LPF_FACTOR)
            mgSmooth[axis] = (mgSmooth[axis] * (MG_LPF_FACTOR - 1) + magADC[axis]) / MG_LPF_FACTOR; // LPF for Magnetometer values
#define MAG_VALUE mgSmooth[axis]
#else
#define MAG_VALUE magADC[axis]
#endif
        }
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
    // If accel magnitude >1.4G or <0.6G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
    // To do that, we just skip filter, as EstV already rotated by Gyro
    if ((36 < accMag && accMag < 196) || f.SMALL_ANGLES_25) {
        for (axis = 0; axis < 3; axis++)
            EstG.A[axis] = (EstG.A[axis] * (float)cfg.gyro_cmpf_factor + accSmooth[axis]) * INV_GYR_CMPF_FACTOR;
    }

    if (sensors(SENSOR_MAG)) {
        for (axis = 0; axis < 3; axis++)
            EstM.A[axis] = (EstM.A[axis] * GYR_CMPFM_FACTOR + MAG_VALUE) * INV_GYR_CMPFM_FACTOR;
    }

    // Attitude of the estimated vector
    angle[ROLL] = _atan2f(EstG.V.X, EstG.V.Z);
    angle[PITCH] = _atan2f(EstG.V.Y, EstG.V.Z);

#ifdef MAG
    if (sensors(SENSOR_MAG)) {
        // Attitude of the cross product vector GxM
        heading = _atan2f(EstG.V.X * EstM.V.Z - EstG.V.Z * EstM.V.X, EstG.V.Z * EstM.V.Y - EstG.V.Y * EstM.V.Z);
        heading = heading + magneticDeclination;
        heading = heading / 10;

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
#define BARO_TAB_SIZE   40

void getEstimatedAltitude(void)
{
    uint32_t index;
    static uint32_t deadLine = INIT_DELAY;
    static int16_t BaroHistTab[BARO_TAB_SIZE];
    static uint32_t BaroHistIdx;
    static int32_t BaroHigh = 0;
    static int32_t BaroLow = 0;
    int32_t temp32;
    int16_t last;

    if (currentTime < deadLine)
        return;
    deadLine = currentTime + UPDATE_INTERVAL;

    //**** Alt. Set Point stabilization PID ****
    //calculate speed for D calculation
    last = BaroHistTab[BaroHistIdx];
    BaroHistTab[BaroHistIdx] = BaroAlt / 10;
    BaroHigh += BaroHistTab[BaroHistIdx];
    index = (BaroHistIdx + (BARO_TAB_SIZE / 2)) % BARO_TAB_SIZE;
    BaroHigh -= BaroHistTab[index];
    BaroLow  += BaroHistTab[index];
    BaroLow  -= last;
    BaroHistIdx++;
    if (BaroHistIdx >= BARO_TAB_SIZE)
        BaroHistIdx = 0;

    BaroPID = 0;
    //D
    temp32 = cfg.D8[PIDALT] * (BaroHigh - BaroLow) / 40;
    BaroPID -= temp32;

    EstAlt = BaroHigh * 10 / (BARO_TAB_SIZE / 2);

    temp32 = AltHold - EstAlt;
    if (abs(temp32) < 10 && abs(BaroPID) < 10)
        BaroPID = 0;  // remove small D parameter to reduce noise near zero position
    // P
    BaroPID += cfg.P8[PIDALT] * constrain(temp32, (-2) * cfg.P8[PIDALT], 2 * cfg.P8[PIDALT]) / 100;
    BaroPID = constrain(BaroPID, -150, +150); // sum of P and D should be in range 150

    // I
    errorAltitudeI += temp32 * cfg.I8[PIDALT] / 50;
    errorAltitudeI = constrain(errorAltitudeI, -30000, 30000);
    temp32 = errorAltitudeI / 500; // I in range +/-60
    BaroPID += temp32;
}
#endif /* BARO */
