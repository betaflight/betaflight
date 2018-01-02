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

#pragma once

#include "common/axis.h"
#include "common/time.h"
#include "common/maths.h"
#include "pg/pg.h"

// Exported symbols
extern uint32_t accTimeSum;
extern int accSumCount;
extern float accVelScale;
extern int32_t accSum[XYZ_AXIS_COUNT];

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

extern attitudeEulerAngles_t attitude;

typedef struct accDeadband_s {
    uint8_t xy;                 // set the acc deadband for xy-Axis
    uint8_t z;                  // set the acc deadband for z-Axis, this ignores small accelerations
} accDeadband_t;

typedef struct imuConfig_s {
    uint16_t dcm_kp;                        // DCM filter proportional gain ( x 10000)
    uint16_t dcm_ki;                        // DCM filter integral gain ( x 10000)
    uint8_t small_angle;
    uint8_t acc_unarmedcal;                 // turn automatic acc compensation on/off
    accDeadband_t accDeadband;
} imuConfig_t;

PG_DECLARE(imuConfig_t, imuConfig);

typedef struct imuRuntimeConfig_s {
    float dcm_ki;
    float dcm_kp;
    uint8_t acc_unarmedcal;
    uint8_t small_angle;
    accDeadband_t accDeadband;
} imuRuntimeConfig_t;

void imuConfigure(uint16_t throttle_correction_angle);

float getCosTiltAngle(void);
void imuUpdateAttitude(timeUs_t currentTimeUs);
int16_t calculateThrottleAngleCorrection(uint8_t throttle_correction_value);

void imuResetAccelerationSum(void);
void imuInit(void);

#ifdef SIMULATOR_BUILD
void imuSetAttitudeRPY(float roll, float pitch, float yaw);  // in deg
void imuSetAttitudeQuat(float w, float x, float y, float z);
#if defined(SIMULATOR_IMU_SYNC)
void imuSetHasNewData(uint32_t dt);
#endif
#endif

void imuQuaternionComputeProducts(quaternion *quat, quaternionProducts *quatProd);
bool imuQuaternionHeadfreeOffsetSet(void);
void imuQuaternionHeadfreeTransformVectorEarthToBody(t_fp_vector_def * v);
