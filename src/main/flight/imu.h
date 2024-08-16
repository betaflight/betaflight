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

#pragma once

#include "common/axis.h"
#include "common/time.h"
#include "common/maths.h"
#include "common/vector.h"

#include "pg/pg.h"
#include "common/vector.h"

// Exported symbols
extern bool canUseGPSHeading;

typedef struct {
    float w,x,y,z;
} quaternion;
#define QUATERNION_INITIALIZE  {.w=1, .x=0, .y=0,.z=0}

typedef struct {
    float ww,wx,wy,wz,xx,xy,xz,yy,yz,zz;
} quaternionProducts;
#define QUATERNION_PRODUCTS_INITIALIZE  {.ww=1, .wx=0, .wy=0, .wz=0, .xx=0, .xy=0, .xz=0, .yy=0, .yz=0, .zz=0}

typedef union {
    int16_t raw[XYZ_AXIS_COUNT];
    struct {
        // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
        int16_t roll;
        int16_t pitch;
        int16_t yaw;
    } values;
} attitudeEulerAngles_t;
#define EULER_INITIALIZE  { { 0, 0, 0 } }

typedef struct {
    quaternion attitude;
    vector3_t integralErr;
} imuAhrsNominalState_t;

typedef struct {
    vector3_t rp;
    float heading;
} imuAhrsErrorState;

typedef struct {
    imuAhrsNominalState_t nominal;
    imuAhrsErrorState error;
    float rpEstimateCovariance;
    // float headingEstimateCovariance;
    float rpCovariance;
    float headingGain;
} imuAhrsState_t;

extern attitudeEulerAngles_t attitude;

typedef struct imuConfig_s {
    uint16_t imu_dcm_kp;          // DCM filter proportional gain ( x 10000)
    uint16_t imu_dcm_ki;          // DCM filter integral gain ( x 10000)
    uint8_t small_angle;
    uint8_t imu_process_denom;
    uint16_t mag_declination;     // Magnetic declination in degrees * 10
    uint8_t gyro_noise_asd;         // gyro noise amplitude spectral density in (decidegrees/s)/sqrt(s)
    uint8_t acc_noise_std;          // accelerometer noise standard deviation in decidegrees/s
} imuConfig_t;

PG_DECLARE(imuConfig_t, imuConfig);

typedef struct imuRuntimeConfig_s {
    float imuDcmKi;
    float imuDcmKp;
    float gyroNoisePsd;   // gyro noise power spectral density, (rad/s)^2/s, i.e. gyro_noise_asd squared
    float accCovariance;   // base accelerometer covariance rad^2
    vector2_t north_ef;   // reference mag field vector heading due North in EF (2D ground plane projection) adjusted for magnetic declination
    float smallAngleCosZ;
    float throttleAngleScale;
    int throttleAngleValue; 
} imuRuntimeConfig_t;

void imuConfigure(uint16_t throttle_correction_angle, uint8_t throttle_correction_value);

float getSinPitchAngle(void);
float getCosTiltAngle(void);
void imuGetQuaternion(quaternion * q);
void imuGetState(imuAhrsState_t* state);
void imuGetRotMatrix(matrix33_t* m);
void imuUpdateAttitude(timeUs_t currentTimeUs);

void imuInit(void);

#ifdef SIMULATOR_BUILD
void imuSetAttitudeRPY(float roll, float pitch, float yaw);  // in deg
void imuSetAttitudeQuat(float w, float x, float y, float z);
#if defined(SIMULATOR_IMU_SYNC)
void imuSetHasNewData(uint32_t dt);
#endif
#endif

bool imuQuaternionHeadfreeOffsetSet(void);
void imuQuaternionHeadfreeTransformVectorEarthToBody(vector3_t *v);
bool shouldInitializeGPSHeading(void);
bool isUpright(void);
