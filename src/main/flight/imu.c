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

#include "platform.h"
#include "debug.h"

#include "common/axis.h"

#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/sonar.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"

#include "config/runtime_config.h"

//#define DEBUG_IMU
//#define DEBUG_IMU_SPEED

int16_t accSmooth[XYZ_AXIS_COUNT];
int32_t accSum[XYZ_AXIS_COUNT];

uint32_t accTimeSum = 0;        // keep track for integration of acc
int accSumCount = 0;
float accVelScale;

int16_t smallAngle = 0;

float throttleAngleScale;
float fc_acc;

float magneticDeclination = 0.0f;       // calculated at startup from config
float gyroScaleRad;


rollAndPitchInclination_t inclination = { { 0, 0 } };     // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
static float anglerad[2] = { 0.0f, 0.0f };    // absolute angle inclination in radians

static imuRuntimeConfig_t *imuRuntimeConfig;
static pidProfile_t *pidProfile;
static accDeadband_t *accDeadband;
static accProcessor_t accProc;

static void qAccProcessingStateMachine(rollAndPitchTrims_t *accelerometerTrims);


void imuConfigure(
    imuRuntimeConfig_t *initialImuRuntimeConfig,
    pidProfile_t *initialPidProfile,
    accDeadband_t *initialAccDeadband,
    float accz_lpf_cutoff,
    uint16_t throttle_correction_angle
)
{
    imuRuntimeConfig = initialImuRuntimeConfig;
    pidProfile = initialPidProfile;
    accDeadband = initialAccDeadband;
    fc_acc = calculateAccZLowPassFilterRCTimeConstant(accz_lpf_cutoff);
    throttleAngleScale = calculateThrottleAngleScale(throttle_correction_angle);
}

float calculateThrottleAngleScale(uint16_t throttle_correction_angle)
{
    return (1800.0f / M_PIf) * (900.0f / throttle_correction_angle);
}

/*
* Calculate RC time constant used in the accZ lpf.
*/
float calculateAccZLowPassFilterRCTimeConstant(float accz_lpf_cutoff)
{
    return 0.5f / (M_PIf * accz_lpf_cutoff);
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

void imuResetAccelerationSum(void)
{
    accSum[0] = 0;
    accSum[1] = 0;
    accSum[2] = 0;
    accSumCount = 0;
    accTimeSum = 0;
}

/*
* Baseflight calculation by Luggi09 originates from arducopter
* ============================================================
* This function rotates magnetic vector to cancel actual yaw and
* pitch of craft. Then it computes it's direction in X/Y plane.
* This value is returned as compass heading, value is 0-360 degrees.
*
* Note that Earth's magnetic field is not parallel with ground unless
* you are near equator. Its inclination is considerable, >60 degrees
* towards ground in most of Europe.
*
* First we consider it in 2D:
*
* An example, the vector <1, 1> would be turned into the heading
* 45 degrees, representing it's angle clockwise from north.
*
*      ***************** *
*      *       |   <1,1> *
*      *       |  /      *
*      *       | /       *
*      *       |/        *
*      *       *         *
*      *                 *
*      *                 *
*      *                 *
*      *                 *
*      *******************
*
* //TODO: Add explanation for how it uses the Z dimension.
*/
int16_t imuCalculateHeading(t_fp_vector *vec)
{
    int16_t head;

    float cosineRoll = cos_approx(anglerad[AI_ROLL]);
    float sineRoll = sin_approx(anglerad[AI_ROLL]);
    float cosinePitch = cos_approx(anglerad[AI_PITCH]);
    float sinePitch = sin_approx(anglerad[AI_PITCH]);
    float Xh = vec->A[X] * cosinePitch + vec->A[Y] * sineRoll * sinePitch + vec->A[Z] * sinePitch * cosineRoll;
    float Yh = vec->A[Y] * cosineRoll - vec->A[Z] * sineRoll;
    //TODO: Replace this comment with an explanation of why Yh and Xh can never simultanoeusly be zero,
    // or handle the case in which they are and (atan2f(0, 0) is undefined.
    float hd = (atan2_approx(Yh, Xh) * 1800.0f / M_PIf + magneticDeclination) / 10.0f;
    head = lrintf(hd);

    // Arctan returns a value in the range -180 to 180 degrees. We 'normalize' negative angles to be positive.
    if (head < 0)
        head += 360;

    return head;
}

void imuUpdate(rollAndPitchTrims_t *accelerometerTrims, uint8_t imuUpdateSensors)
{
#ifdef DEBUG_IMU_SPEED
	uint32_t time = micros();
#endif
	if (imuUpdateSensors == ONLY_GYRO || imuUpdateSensors == ACC_AND_GYRO) {
        gyroUpdate();
#ifdef DEBUG_IMU_SPEED
    debug[0] = micros() - time; // gyro read time
#endif
    }
    if (sensors(SENSOR_ACC) && (!imuUpdateSensors == ONLY_GYRO)) {
#ifdef DEBUG_IMU_SPEED
        time = micros();
#endif
        qAccProcessingStateMachine(accelerometerTrims);
    } else {
        accADC[X] = 0;
        accADC[Y] = 0;
        accADC[Z] = 0;
    }
#ifdef DEBUG_IMU_SPEED
    debug[1] = micros() - time;     // acc read time
	if (imuUpdateSensors == ACC_AND_GYRO) {
        debug[2] = debug[0] + debug[1]; // gyro + acc read time
	}
#endif
}

int16_t calculateThrottleAngleCorrection(uint8_t throttle_correction_value)
{
    float cosZ = EstG.V.Z / sqrtf(EstG.V.X * EstG.V.X + EstG.V.Y * EstG.V.Y + EstG.V.Z * EstG.V.Z);

    /*
    * Use 0 as the throttle angle correction if we are inverted, vertical or with a
    * small angle < 0.86 deg
    * TODO: Define this small angle in config.
    */
    if (cosZ <= 0.015f) {
        return 0;
    }
    int angle = lrintf(acos_approx(cosZ) * throttleAngleScale);
    if (angle > 900)
        angle = 900;
    return lrintf(throttle_correction_value * sin_approx(angle / (900.0f * M_PIf / 2.0f)));
}

// WITHOUT
//arm - none - eabi - size . / obj / main / cleanflight_CC3D.elf
//text    data     bss     dec     hex filename
//116324     376   12640  129340   1f93c . / obj / main / cleanflight_CC3D.elf

//////////////////////////////////////////////////////////////////////
// 4D Quaternion / 3D Vector Math
//arm - none - eabi - size . / obj / main / cleanflight_CC3D.elf
//text    data     bss     dec     hex filename
//116284     364   12636  129284   1f904 . / obj / main / cleanflight_CC3D.elf

typedef struct v3_s
{
  float x;
  float y;
  float z;
} v3_t;

const v3_t V0 = { .x = 0.0f, .y = 0.0f, .z = 0.0f };
const v3_t VX = { .x = 1.0f, .y = 0.0f, .z = 0.0f };
const v3_t VY = { .x = 0.0f, .y = 1.0f, .z = 0.0f };
const v3_t VZ = { .x = 0.0f, .y = 0.0f, .z = 1.0f };

typedef struct q4_s
{
  float w;
  float x;
  float y;
  float z;
} q4_t;

const q4_t Q0 = { .w = 1.0f, .x = 0.0f, .y = 0.0f, .z = 0.0f };

void MulQQ(const q4_t *a, const q4_t *b, q4_t *o)
{
  q4_t r;  
  r.w = a->w * b->w - a->x * b->x - a->y * b->y - a->z * b->z;
  r.x = a->w * b->x + a->z * b->y - a->y * b->z + a->x * b->w;
  r.y = a->w * b->y + a->x * b->z + a->y * b->w - a->z * b->x;
  r.z = a->y * b->x - a->x * b->y + a->w * b->z + a->z * b->w;
  *o = r;
}

void MulQV(const q4_t *a, const v3_t *b, q4_t *o)
{
  q4_t r;
  r.w = -a->x * b->x - a->y * b->y - a->z * b->z;
  r.x =  a->w * b->x + a->z * b->y - a->y * b->z;
  r.y =  a->w * b->y + a->x * b->z - a->z * b->x;
  r.z =  a->y * b->x - a->x * b->y + a->w * b->z;
  *o = r;
}

void MulQF(const q4_t *a, const float b, q4_t *o)
{
  q4_t r;
  r.w = a->w * b;
  r.x = a->x * b;
  r.y = a->y * b;
  r.z = a->z * b;
  *o = r;
}

void MulVF(const v3_t *a, const float b, v3_t *o)
{
  v3_t r;
  r.x = a->x * b;
  r.y = a->y * b;
  r.z = a->z * b;
  *o = r;
}

void SumQQ(const q4_t *a, const q4_t *b, q4_t *o)
{
  q4_t r;
  r.w = a->w + b->w;
  r.x = a->x + b->x;
  r.y = a->y + b->y;
  r.z = a->z + b->z;
  *o = r;
}


void SumVV(const v3_t *a, const v3_t *b, v3_t *o)
{
  v3_t r;
  r.x = a->x + b->x;
  r.y = a->y + b->y;
  r.z = a->z + b->z;
  *o = r;
}

void SubQQ(const q4_t *a, const q4_t *b, q4_t *o)
{
  q4_t r;
  r.w = a->w - b->w;
  r.x = a->x - b->x;
  r.y = a->y - b->y;
  r.z = a->z - b->z;
  *o = r;
}


void SubVV(const v3_t *a, const v3_t *b, v3_t *o)
{
  v3_t r;
  r.x = a->x - b->x;
  r.y = a->y - b->y;
  r.z = a->z - b->z;
  *o = r;
}

void CrossQQ(const q4_t *a, const q4_t *b, q4_t *o)
{
  q4_t r;
  r.w = 0.0f;
  r.x = a->y * b->z - a->z * b->y;
  r.y = a->z * b->x - a->x * b->z;
  r.z = a->x * b->y - a->y * b->x;
  *o = r;
}

void CrossVV(const v3_t *a, const v3_t *b, v3_t *o)
{
  v3_t r;
  r.x = a->y * b->z - a->z * b->y;
  r.y = a->z * b->x - a->x * b->z;
  r.z = a->x * b->y - a->y * b->x;
  *o = r;
}

float DotQQ(const q4_t *a, const q4_t *b)
{
  return a->w * b->w + a->x * b->x + a->y * b->y + a->z * b->z;
}

float DotVV(const v3_t *a, const v3_t *b)
{
  return a->x * b->x + a->y * b->y + a->z * b->z;
}

float Mag2Q(const q4_t *a) // magnitude squared
{
  return a->w*a->w + a->x*a->x + a->y*a->y + a->z*a->z;
}

#define MagQ(a) sqrtf(Mag2Q(a))

float Mag2V(const v3_t *a) // magnitude squared
{
  return a->x*a->x + a->y*a->y + a->z*a->z; // TODO: optimize for unit vectors (m2 nearly 1.0)
}

#define MagV(a) sqrtf(Mag2V(a))

void NormQ(const q4_t *a, q4_t *o)
{
  q4_t r;
  MulQF(a, 1 / MagQ(a), &r);
  *o = r;
}

void NormV(const v3_t *a, v3_t *o)
{
  v3_t r;
  float m = MagV(a);
  MulVF(a, 1 / m, &r); // TODO: m nearly 0
  *o = r;
}

void ConjQ(const q4_t *a, q4_t *o)
{
  q4_t r;
  r.w = a->w;
  r.x = -a->x;
  r.y = -a->y;
  r.z = -a->z;
  *o = r;
}

void RotateVQ(const v3_t *v, const q4_t *q, v3_t *o) //Vector rotated by a Quaternion(matches V^ = V * Matrix)
{
  // v + 2 * r X(r X v + q.w*v) --https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Performance_comparisons
  // vector r is the three imaginary coefficients of quaternion q

  v3_t r2_rv_vw;
  {
    // reverse signs to change direction of rotation
    v3_t r = { .x = -q->x, .y = -q->y, .z = -q->z };
    v3_t r2;
    SumVV(&r, &r, &r2);

    v3_t rv_vw;
    {
      v3_t vw;
      MulVF(v, q->w, &vw);
      v3_t rv;
      CrossVV(&r, v, &rv);
      SumVV(&rv, &vw, &rv_vw);
    }
    CrossVV(&r2, &rv_vw, &r2_rv_vw);
  }
  SumVV(v, &r2_rv_vw, o);
}

void quaternion_approx(const v3_t *w, q4_t *o) // (angle vector[rad]) --Small angle approximation
{
  q4_t r;
  r.x = w->x / 2;
  r.y = w->y / 2;
  r.z = w->z / 2;
  r.w = 1.0f - (0.5f * ((r.x * r.x) + (r.y * r.y) + (r.z * r.z)));
  *o = r;
}

# define quaternion(w,o) quaternion_approx(w,o) // I think we can get away with the approximation
// TODO - try usining sin_approx, cos_approx

typedef struct rpy_s
{
  float r;
  float p;
  float y;
} rpy_t;
const rpy_t RPY0 = { .r = 0, .p = 0, .y = 0 };

void quaternion_from_rpy(const rpy_t *a, q4_t *o) // (degrees) yaw->pitch->roll order
{
  float cr, sr, cp, sp, cy, sy;

  { float r2 = a->r * (RAD / 2); cr = cos_approx(r2); sr = sin_approx(r2); }
  { float p2 = a->p * (RAD / 2); cp = cos_approx(p2); sp = sin_approx(p2); }
  { float y2 = a->y * (RAD / 2); cy = cos_approx(y2); sy = sin_approx(y2); }

  o->w = cr*cp*cy + sr*sp*sy;
  o->x = sr*cp*cy - cr*sp*sy;
  o->y = cr*sp*cy + sr*cp*sy;
  o->z = cr*cp*sy - sr*sp*cy;
}

void quaternion_to_rpy(const q4_t *q, rpy_t *o) // (degrees) yaw->pitch->roll order
{
  // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  // Body Z - Y - X sequence

  float q0 = q->w;
  float q1 = q->x;
  float q2 = q->y;
  float q3 = q->z;

  float p0 = MAX(-1.0f, MIN(1.0f, 2 * (q0*q2 - q3*q1)));
  o->p = asin(p0);

  if (ABS(ABS(o->p) - (90 * RAD)) < (0.5f*RAD)) // vertical
  {
    o->y = 2 * atan2_approx(q3, q0);
    o->r = 0.0f;
  }
  else
  {
    float r0 = 2 * (q0*q1 + q2*q3);
    float r1 = 1 - 2 * (q1*q1 + q2*q2);
    if ((r0 == 0) && (r1 == 0)) { o->r = 0.0f; }  // atan(0,0)!
    else  { o->r = atan2_approx(r0, r1); }

    float y0 = 2 * (q0*q3 + q1*q2);
    float y1 = 1 - 2 * (q2*q2 + q3*q3);
    if ((y0 == 0) && (y1 == 0)) { o->y = 0.0f; } // atan(0,0)!
    else { o->y = atan2_approx(y0, y1); }
  }

  o->y = -o->y;     // yaw inversion hack for all boards

}

void angle_vector(const q4_t *a, v3_t *o) // convert from quaternion to angle vector[rad]
{
  q4_t a1;
  if (a->w < 0) { MulQF(a, -1, &a1); a = &a1; }

  float t2 = acos_approx(MIN(1, a->w)); // TODO acos_approx??

  if (ABS(t2) > (0.005f * RAD))
  {
    float s = sin_approx(t2) / (2 * t2);
    o->x = a->x / s;
    o->y = a->y / s;
    o->z = a->z / s;
  }
  else
  {
    *o = V0;
  }
}

void nlerp_step(const q4_t *a, const q4_t *b, float max_th, q4_t *o) // max_th in radians (max_rate * update time)
{
  float dot = MAX(-1, MIN(1, DotQQ(a, b)));
  float th = 2*acos_approx(ABS(dot)); // ABS -> change direction for shortest path
  
  if (th <= (0.01f*RAD))  {  *o = *b; } // tiny step
  else
  {
    float tb = MIN(1, ABS(max_th / th));
    float ta = 1-tb;
    if (dot < 0) { tb = -tb; } // change direction for shortest path

    q4_t r, a1, b1;
    MulQF(a, ta, &a1);
    MulQF(b, tb, &b1);
    SumQQ(&a1, &b1, &r);
    NormQ(&r, o);
  }  
}

q4_t attitude_est_e_q;
float acc_rad_scale; // adc -> G
float gyro_rads_scale; // adc -> rad/s
float cosSmallAngle;
float acc_lpf_f0, acc_lpf_f1;
v3_t gravity_lpf_b_v, acc_lpf_b_v;

void qimuInit()
{
  cosSmallAngle = cos_approx(RAD*imuRuntimeConfig->small_angle);

  acc_rad_scale = 1.0f / acc_1G;
  gyro_rads_scale = gyro.scale * RAD;

  acc_lpf_f1 = (1.0f / imuRuntimeConfig->acc_lpf_factor);
  acc_lpf_f0 = 1.0f - acc_lpf_f1;
  gravity_lpf_b_v = VZ;
  acc_lpf_b_v = VZ;

  quaternion_from_rpy(&RPY0, &attitude_est_e_q);
  accProc.state = ACCPROC_READ;
}

//////////////////////////////////////////////////////////////////////

static void qAccProcessingStateMachine(rollAndPitchTrims_t *accelerometerTrims)
{
    int axis;
    const float gyro_drift_factor = 0.00f;
    static v3_t gyro_drift_correction_b_v = { .x = 0.0f, .y = 0.0f, .z = 0.0f }; // rad/s

    const float attitude_correction_factor = 0.001f;
    static v3_t attitude_correction_b_v = { .x = 0.0f, .y = 0.0f, .z = 0.0f }; // rad/s
    static v3_t acc_b_v, gyro_rate_b_v;

    static int16_t normalize_counter = 0;
    static uint32_t previousT = 0;
    static uint32_t currentT;

    // get time step.. TODO: this should really be fixed to division of MPU sample rate
    static float dT;

    bool keepProcessing = true;               // (keepProcessing == true): causes all states to execute (for slow cycle times)

    do {
        switch (accProc.state) {

            case ACCPROC_READ:
                currentT = micros();
                dT = (currentT - previousT)*0.000001f;
                previousT = currentT;
                updateAccelerationReadings(accelerometerTrims); // TODO rename to accelerometerUpdate and rename many other 'Acceleration' references to be 'Accelerometer'
                accProc.state++;
                break;

            case ACCPROC_CHUNK_1:
                for (axis = 0; axis < 3; axis++) {
                  accSmooth[axis] = accADC[axis]; // TODO acc_lpf - or would this work better without it?
                  ((float *)&acc_b_v)[axis] = accSmooth[axis] * acc_rad_scale;
                  ((float *)&gyro_rate_b_v)[axis] = gyroADC[axis] * gyro_rads_scale;
                }

                ////////////////////////////////////////////////////////////////
                // add in drift compensation
                SumVV(&gyro_rate_b_v, &gyro_drift_correction_b_v, &gyro_rate_b_v);

#ifdef DEBUG_IMU
                debug[0] = gyro_drift_correction_b_v.x * 10000;
                debug[1] = gyro_drift_correction_b_v.y * 10000;
                debug[2] = gyro_drift_correction_b_v.z * 10000;
#endif

                ////////////////////////////////////////////////////////////////
                // add in attitude estimate correction, convert to degrees
                v3_t gyro_rotation_b_v;
                SumVV(&gyro_rate_b_v, &attitude_correction_b_v, &gyro_rotation_b_v);
                MulVF(&gyro_rotation_b_v, dT, &gyro_rotation_b_v);

                ////////////////////////////////////////////////////////////////
                // Update attitude estimate with gyro data
                //  small angle approximation should be fine, but error does creep in at high rotational rates on multiple axes - Normalize periodically
                q4_t attitude_est_update_b_q;
                quaternion(&gyro_rotation_b_v, &attitude_est_update_b_q);              // convert angle vector to quaternion
                MulQQ(&attitude_est_update_b_q, &attitude_est_e_q, &attitude_est_e_q); // and rotate estimate

                v3_t gravity_b_v;
                // Calculate expected gravity(allows drift to be compensated on all 3 axis when possible)
                RotateVQ(&VZ, &attitude_est_e_q, &gravity_b_v);

                // check small angle
                if (gravity_b_v.z > cosSmallAngle) {
                  ENABLE_STATE(SMALL_ANGLE);
                } else {
                  DISABLE_STATE(SMALL_ANGLE);
                }

                // acc_lpf
                if (imuRuntimeConfig->acc_lpf_factor > 0) {
                  v3_t a0, a1;
                  MulVF(&acc_lpf_b_v, acc_lpf_f0, &a0);
                  MulVF(&acc_b_v, acc_lpf_f1, &a1);
                  SumVV(&a0, &a1, &acc_lpf_b_v);

                  MulVF(&gravity_lpf_b_v, acc_lpf_f0, &a0);
                  MulVF(&gravity_b_v, acc_lpf_f1, &a1);
                  SumVV(&a0, &a1, &gravity_lpf_b_v);
                } else {
                  acc_lpf_b_v = acc_b_v;
                  gravity_lpf_b_v = gravity_b_v;
                }

                ////////////////////////////////////////////////////////////////
                // Calculate Correction
                float acc_m2 = Mag2V(&acc_b_v);
#ifdef DEBUG_IMU
                debug[3] = acc_m2*1000;
#endif
                if ((acc_m2 > 1.1025f) || (acc_m2 < 0.9025f)) {
                  attitude_correction_b_v = V0;
                } else { // we're not accelerating
                  // Cross product to determine error
                  CrossVV(&acc_lpf_b_v, &gravity_lpf_b_v, &attitude_correction_b_v);
                  MulVF(&attitude_correction_b_v, attitude_correction_factor/dT, &attitude_correction_b_v); // convert to rate for drift correction

                  if (gyro_drift_factor != 0.0f) {
                    // conditionally update drift for valid axes (4.5 degree check)
                    if (ABS(gravity_b_v.x) < 0.997f) {
                      gyro_drift_correction_b_v.x = gyro_drift_correction_b_v.x + (attitude_correction_b_v.x*gyro_drift_factor);
                    }
                    if (ABS(gravity_b_v.y) < 0.997f) {
                      gyro_drift_correction_b_v.y = gyro_drift_correction_b_v.y + (attitude_correction_b_v.y*gyro_drift_factor);
                    }
                    if (ABS(gravity_b_v.z) < 0.997f) {
                      gyro_drift_correction_b_v.z = gyro_drift_correction_b_v.z + (attitude_correction_b_v.z*gyro_drift_factor);
                    }
                  }
                }

                // renormalize every couple of seconds
                if (++normalize_counter == 1000) {
                  NormQ(&attitude_est_e_q, &attitude_est_e_q);
                  normalize_counter = 0;
                }

                // convert to cleanflight values
                // update inclination
                rpy_t rpy;
                quaternion_to_rpy(&attitude_est_e_q, &rpy);
                inclination.values.rollDeciDegrees = lrintf(rpy.r * (10 / RAD));
                inclination.values.pitchDeciDegrees = lrintf(rpy.p * (10 / RAD));
                heading = rpy.y * (1 / RAD);
                if (heading < 0) heading += 360;
#ifdef DEBUG_IMU
                //uint32_t endT = micros();
                //debug[3] = endT - currentT;
#endif

                keepProcessing = false;
                /* no break */

            default:
                accProc.state = ACCPROC_READ;
                break;
        }
    } while (keepProcessing);
}
