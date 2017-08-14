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

#include <stdint.h>
#include <math.h>

#include "axis.h"
#include "maths.h"

// http://lolengine.net/blog/2011/12/21/better-function-approximations
// Chebyshev http://stackoverflow.com/questions/345085/how-do-trigonometric-functions-work/345117#345117
// Thanks for ledvinap for making such accuracy possible! See: https://github.com/cleanflight/cleanflight/issues/940#issuecomment-110323384
// https://github.com/Crashpilot1000/HarakiriWebstore1/blob/master/src/mw.c#L1235
#if defined(FAST_MATH) || defined(VERY_FAST_MATH)
#if defined(VERY_FAST_MATH)
#define sinPolyCoef3 -1.666568107e-1f
#define sinPolyCoef5  8.312366210e-3f
#define sinPolyCoef7 -1.849218155e-4f
#define sinPolyCoef9  0
#else
#define sinPolyCoef3 -1.666665710e-1f                                          // Double: -1.666665709650470145824129400050267289858e-1
#define sinPolyCoef5  8.333017292e-3f                                          // Double:  8.333017291562218127986291618761571373087e-3
#define sinPolyCoef7 -1.980661520e-4f                                          // Double: -1.980661520135080504411629636078917643846e-4
#define sinPolyCoef9  2.600054768e-6f                                          // Double:  2.600054767890361277123254766503271638682e-6
#endif

float sin_approx(float x)
{
    int32_t xint = x;
    if (xint < -32 || xint > 32) return 0.0f;                               // Stop here on error input (5 * 360 Deg)
    while (x >  M_PIf) x -= (2.0f * M_PIf);                                 // always wrap input angle to -PI..PI
    while (x < -M_PIf) x += (2.0f * M_PIf);
    if (x >  (0.5f * M_PIf)) x =  (0.5f * M_PIf) - (x - (0.5f * M_PIf));   // We just pick -90..+90 Degree
    else if (x < -(0.5f * M_PIf)) x = -(0.5f * M_PIf) - ((0.5f * M_PIf) + x);
    float x2 = x * x;
    return x + x * x2 * (sinPolyCoef3 + x2 * (sinPolyCoef5 + x2 * (sinPolyCoef7 + x2 * sinPolyCoef9)));
}

float cos_approx(float x)
{
    return sin_approx(x + (0.5f * M_PIf));
}

// https://github.com/Crashpilot1000/HarakiriWebstore1/blob/396715f73c6fcf859e0db0f34e12fe44bace6483/src/mw.c#L1292
// http://http.developer.nvidia.com/Cg/atan2.html (not working correctly!)
// Poly coefficients by @ledvinap (https://github.com/cleanflight/cleanflight/pull/1107)
// Max absolute error 0,000027 degree
float atan2_approx(float y, float x)
{
    #define atanPolyCoef1  3.14551665884836e-07f
    #define atanPolyCoef2  0.99997356613987f
    #define atanPolyCoef3  0.14744007058297684f
    #define atanPolyCoef4  0.3099814292351353f
    #define atanPolyCoef5  0.05030176425872175f
    #define atanPolyCoef6  0.1471039133652469f
    #define atanPolyCoef7  0.6444640676891548f

    float res, absX, absY;
    absX = fabsf(x);
    absY = fabsf(y);
    res  = MAX(absX, absY);
    if (res) res = MIN(absX, absY) / res;
    else res = 0.0f;
    res = -((((atanPolyCoef5 * res - atanPolyCoef4) * res - atanPolyCoef3) * res - atanPolyCoef2) * res - atanPolyCoef1) / ((atanPolyCoef7 * res + atanPolyCoef6) * res + 1.0f);
    if (absY > absX) res = (M_PIf / 2.0f) - res;
    if (x < 0) res = M_PIf - res;
    if (y < 0) res = -res;
    return res;
}

// http://http.developer.nvidia.com/Cg/acos.html
// Handbook of Mathematical Functions
// M. Abramowitz and I.A. Stegun, Ed.
// Absolute error <= 6.7e-5
float acos_approx(float x)
{
    float xa = fabsf(x);
    float result = sqrtf(1.0f - xa) * (1.5707288f + xa * (-0.2121144f + xa * (0.0742610f + (-0.0187293f * xa))));
    if (x < 0.0f)
        return M_PIf - result;
    else
        return result;
}
#endif

int gcd(int num, int denom)
{
    if (denom == 0) {
        return num;
    }

    return gcd(denom, num % denom);
}

int32_t wrap_18000(int32_t angle)
{
    if (angle > 18000)
        angle -= 36000;
    if (angle < -18000)
        angle += 36000;
    return angle;
}

int32_t wrap_36000(int32_t angle)
{
    if (angle > 36000)
        angle -= 36000;
    if (angle < 0)
        angle += 36000;
    return angle;
}

int32_t applyDeadband(int32_t value, int32_t deadband)
{
    if (ABS(value) < deadband) {
        value = 0;
    } else if (value > 0) {
        value -= deadband;
    } else if (value < 0) {
        value += deadband;
    }
    return value;
}

int constrain(int amt, int low, int high)
{
    if (amt < low)
        return low;
    else if (amt > high)
        return high;
    else
        return amt;
}

float constrainf(float amt, float low, float high)
{
    if (amt < low)
        return low;
    else if (amt > high)
        return high;
    else
        return amt;
}

void devClear(stdev_t *dev)
{
    dev->m_n = 0;
}

void devPush(stdev_t *dev, float x)
{
    dev->m_n++;
    if (dev->m_n == 1) {
        dev->m_oldM = dev->m_newM = x;
        dev->m_oldS = 0.0f;
    } else {
        dev->m_newM = dev->m_oldM + (x - dev->m_oldM) / dev->m_n;
        dev->m_newS = dev->m_oldS + (x - dev->m_oldM) * (x - dev->m_newM);
        dev->m_oldM = dev->m_newM;
        dev->m_oldS = dev->m_newS;
    }
}

float devVariance(stdev_t *dev)
{
    return ((dev->m_n > 1) ? dev->m_newS / (dev->m_n - 1) : 0.0f);
}

float devStandardDeviation(stdev_t *dev)
{
    return sqrtf(devVariance(dev));
}

float degreesToRadians(int16_t degrees)
{
    return degrees * RAD;
}

int scaleRange(int x, int srcMin, int srcMax, int destMin, int destMax) {
    long int a = ((long int) destMax - (long int) destMin) * ((long int) x - (long int) srcMin);
    long int b = (long int) srcMax - (long int) srcMin;
    return ((a / b) - (destMax - destMin)) + destMax;
}

float scaleRangef(float x, float srcMin, float srcMax, float destMin, float destMax) {
    float a = (destMax - destMin) * (x - srcMin);
    float b = srcMax - srcMin;
    return ((a / b) - (destMax - destMin)) + destMax;
}

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

void buildRotationMatrix(fp_angles_t *delta, float matrix[3][3])
{
    float cosx, sinx, cosy, siny, cosz, sinz;
    float coszcosx, sinzcosx, coszsinx, sinzsinx;

    cosx = cos_approx(delta->angles.roll);
    sinx = sin_approx(delta->angles.roll);
    cosy = cos_approx(delta->angles.pitch);
    siny = sin_approx(delta->angles.pitch);
    cosz = cos_approx(delta->angles.yaw);
    sinz = sin_approx(delta->angles.yaw);

    coszcosx = cosz * cosx;
    sinzcosx = sinz * cosx;
    coszsinx = sinx * cosz;
    sinzsinx = sinx * sinz;

    matrix[0][X] = cosz * cosy;
    matrix[0][Y] = -cosy * sinz;
    matrix[0][Z] = siny;
    matrix[1][X] = sinzcosx + (coszsinx * siny);
    matrix[1][Y] = coszcosx - (sinzsinx * siny);
    matrix[1][Z] = -sinx * cosy;
    matrix[2][X] = (sinzsinx) - (coszcosx * siny);
    matrix[2][Y] = (coszsinx) + (sinzcosx * siny);
    matrix[2][Z] = cosy * cosx;
}

// Rotate a vector *v by the euler angles defined by the 3-vector *delta.
void rotateV(struct fp_vector *v, fp_angles_t *delta)
{
    struct fp_vector v_tmp = *v;

    float matrix[3][3];

    buildRotationMatrix(delta, matrix);

    v->X = v_tmp.X * matrix[0][X] + v_tmp.Y * matrix[1][X] + v_tmp.Z * matrix[2][X];
    v->Y = v_tmp.X * matrix[0][Y] + v_tmp.Y * matrix[1][Y] + v_tmp.Z * matrix[2][Y];
    v->Z = v_tmp.X * matrix[0][Z] + v_tmp.Y * matrix[1][Z] + v_tmp.Z * matrix[2][Z];
}

// Quick median filter implementation
// (c) N. Devillard - 1998
// http://ndevilla.free.fr/median/median.pdf
#define QMF_SORT(a,b) { if ((a)>(b)) QMF_SWAP((a),(b)); }
#define QMF_SWAP(a,b) { int32_t temp=(a);(a)=(b);(b)=temp; }
#define QMF_COPY(p,v,n) { int32_t i; for (i=0; i<n; i++) p[i]=v[i]; }

int32_t quickMedianFilter3(int32_t * v)
{
    int32_t p[3];
    QMF_COPY(p, v, 3);

    QMF_SORT(p[0], p[1]); QMF_SORT(p[1], p[2]); QMF_SORT(p[0], p[1]) ;
    return p[1];
}

int32_t quickMedianFilter5(int32_t * v)
{
    int32_t p[5];
    QMF_COPY(p, v, 5);

    QMF_SORT(p[0], p[1]); QMF_SORT(p[3], p[4]); QMF_SORT(p[0], p[3]);
    QMF_SORT(p[1], p[4]); QMF_SORT(p[1], p[2]); QMF_SORT(p[2], p[3]);
    QMF_SORT(p[1], p[2]);
    return p[2];
}

int32_t quickMedianFilter7(int32_t * v)
{
    int32_t p[7];
    QMF_COPY(p, v, 7);

    QMF_SORT(p[0], p[5]); QMF_SORT(p[0], p[3]); QMF_SORT(p[1], p[6]);
    QMF_SORT(p[2], p[4]); QMF_SORT(p[0], p[1]); QMF_SORT(p[3], p[5]);
    QMF_SORT(p[2], p[6]); QMF_SORT(p[2], p[3]); QMF_SORT(p[3], p[6]);
    QMF_SORT(p[4], p[5]); QMF_SORT(p[1], p[4]); QMF_SORT(p[1], p[3]);
    QMF_SORT(p[3], p[4]);
    return p[3];
}

int32_t quickMedianFilter9(int32_t * v)
{
    int32_t p[9];
    QMF_COPY(p, v, 9);

    QMF_SORT(p[1], p[2]); QMF_SORT(p[4], p[5]); QMF_SORT(p[7], p[8]);
    QMF_SORT(p[0], p[1]); QMF_SORT(p[3], p[4]); QMF_SORT(p[6], p[7]);
    QMF_SORT(p[1], p[2]); QMF_SORT(p[4], p[5]); QMF_SORT(p[7], p[8]);
    QMF_SORT(p[0], p[3]); QMF_SORT(p[5], p[8]); QMF_SORT(p[4], p[7]);
    QMF_SORT(p[3], p[6]); QMF_SORT(p[1], p[4]); QMF_SORT(p[2], p[5]);
    QMF_SORT(p[4], p[7]); QMF_SORT(p[4], p[2]); QMF_SORT(p[6], p[4]);
    QMF_SORT(p[4], p[2]);
    return p[4];
}

void arraySubInt32(int32_t *dest, int32_t *array1, int32_t *array2, int count)
{
    for (int i = 0; i < count; i++) {
        dest[i] = array1[i] - array2[i];
    }
}

/**
 * Sensor offset calculation code based on Freescale's AN4246
 * Initial implementation by @HaukeRa
 * Modified to be re-usable by @DigitalEntity
 */
void sensorCalibrationResetState(sensorCalibrationState_t * state)
{
    for (int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++){
            state->XtX[i][j] = 0;
        }

        state->XtY[i] = 0;
    }
}

void sensorCalibrationPushSampleForOffsetCalculation(sensorCalibrationState_t * state, int32_t sample[3])
{
    state->XtX[0][0] += (float)sample[0] * sample[0];
    state->XtX[0][1] += (float)sample[0] * sample[1];
    state->XtX[0][2] += (float)sample[0] * sample[2];
    state->XtX[0][3] += (float)sample[0];

    state->XtX[1][0] += (float)sample[1] * sample[0];
    state->XtX[1][1] += (float)sample[1] * sample[1];
    state->XtX[1][2] += (float)sample[1] * sample[2];
    state->XtX[1][3] += (float)sample[1];

    state->XtX[2][0] += (float)sample[2] * sample[0];
    state->XtX[2][1] += (float)sample[2] * sample[1];
    state->XtX[2][2] += (float)sample[2] * sample[2];
    state->XtX[2][3] += (float)sample[2];

    state->XtX[3][0] += (float)sample[0];
    state->XtX[3][1] += (float)sample[1];
    state->XtX[3][2] += (float)sample[2];
    state->XtX[3][3] += 1;

    float squareSum = ((float)sample[0] * sample[0]) + ((float)sample[1] * sample[1]) + ((float)sample[2] * sample[2]);
    state->XtY[0] += sample[0] * squareSum;
    state->XtY[1] += sample[1] * squareSum;
    state->XtY[2] += sample[2] * squareSum;
    state->XtY[3] += squareSum;
}

void sensorCalibrationPushSampleForScaleCalculation(sensorCalibrationState_t * state, int axis, int32_t sample[3], int target)
{
    for (int i = 0; i < 3; i++) {
        float scaledSample = (float)sample[i] / (float)target;
        state->XtX[axis][i] += scaledSample * scaledSample;
        state->XtX[3][i] += scaledSample * scaledSample;
    }

    state->XtX[axis][3] += 1;
    state->XtY[axis] += 1;
    state->XtY[3] += 1;
}

static void sensorCalibration_gaussLR(float mat[4][4]) {
    uint8_t n = 4;
    int i, j, k;
    for (i = 0; i < 4; i++) {
        // Determine R
        for (j = i; j < 4; j++) {
            for (k = 0; k < i; k++) {
                mat[i][j] -= mat[i][k] * mat[k][j];
            }
        }
        // Determine L
        for (j = i + 1; j < n; j++) {
            for (k = 0; k < i; k++) {
                mat[j][i] -= mat[j][k] * mat[k][i];
            }
            mat[j][i] /= mat[i][i];
        }
    }
}

void sensorCalibration_ForwardSubstitution(float LR[4][4], float y[4], float b[4]) {
    int i, k;
    for (i = 0; i < 4; ++i) {
        y[i] = b[i];
        for (k = 0; k < i; ++k) {
            y[i] -= LR[i][k] * y[k];
        }
        //y[i] /= MAT_ELEM_AT(LR,i,i); //Do not use, LR(i,i) is 1 anyways and not stored in this matrix
    }
}

void sensorCalibration_BackwardSubstitution(float LR[4][4], float x[4], float y[4]) {
    int i, k;
    for (i = 3 ; i >= 0; --i) {
        x[i] = y[i];
        for (k = i + 1; k < 4; ++k) {
            x[i] -= LR[i][k] * x[k];
        }
        x[i] /= LR[i][i];
    }
}

// solve linear equation
// https://en.wikipedia.org/wiki/Gaussian_elimination
static void sensorCalibration_SolveLGS(float A[4][4], float x[4], float b[4]) {
    int i;
    float y[4];

    sensorCalibration_gaussLR(A);

    for (i = 0; i < 4; ++i) {
        y[i] = 0;
    }

    sensorCalibration_ForwardSubstitution(A, y, b);
    sensorCalibration_BackwardSubstitution(A, x, y);
}

void sensorCalibrationSolveForOffset(sensorCalibrationState_t * state, float result[3])
{
    float beta[4];
    sensorCalibration_SolveLGS(state->XtX, beta, state->XtY);

    for (int i = 0; i < 3; i++) {
        result[i] = beta[i] / 2;
    }
}

void sensorCalibrationSolveForScale(sensorCalibrationState_t * state, float result[3])
{
    float beta[4];
    sensorCalibration_SolveLGS(state->XtX, beta, state->XtY);

    for (int i = 0; i < 3; i++) {
        result[i] = sqrtf(beta[i]);
    }
}

float bellCurve(const float x, const float curveWidth)
{
    return powf(M_Ef, -sq(x) / (2.0f * sq(curveWidth)));
}
