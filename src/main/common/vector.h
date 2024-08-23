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

/*
 * some functions are taken from https://github.com/iNavFlight/inav/
 */

#pragma once

#include "common/maths.h"

typedef union {
    float v[2];
    struct {
       float x,y;
    };
} fpVector2_t;


typedef union {
    float v[3];
    struct {
       float x, y, z;
    };
} fpVector3_t;

typedef struct {
    float m[3][3];
} fpMat33_t;

static inline fpVector3_t * vectorZero(fpVector3_t *v)
{
    v->x = 0.0f;
    v->y = 0.0f;
    v->z = 0.0f;
    return v;
}

static inline float vectorNormSquared(const fpVector3_t * v)
{
    return sq(v->x) + sq(v->y) + sq(v->z);
}

static inline float vectorNorm(const fpVector3_t * v)
{
    return sqrtf(vectorNormSquared(v));
}

static inline fpVector3_t * vectorCrossProduct(fpVector3_t *result, const fpVector3_t *a, const fpVector3_t *b)
{
    fpVector3_t ab;

    ab.x = a->y * b->z - a->z * b->y;
    ab.y = a->z * b->x - a->x * b->z;
    ab.z = a->x * b->y - a->y * b->x;

    *result = ab;
    return result;
}

static inline fpVector3_t * vectorAdd(fpVector3_t *result, const fpVector3_t *a, const fpVector3_t *b)
{
    fpVector3_t ab;

    ab.x = a->x + b->x;
    ab.y = a->y + b->y;
    ab.z = a->z + b->z;

    *result = ab;
    return result;
}

static inline fpVector3_t * vectorScale(fpVector3_t *result, const fpVector3_t *a, const float b)
{
    fpVector3_t ab;

    ab.x = a->x * b;
    ab.y = a->y * b;
    ab.z = a->z * b;

    *result = ab;
    return result;
}

static inline fpVector3_t * vectorNormalize(fpVector3_t *result, const fpVector3_t *v)
{
    float normSq = vectorNormSquared(v);
    if (normSq > 0) {              // Hopefully sqrt(nonzero) is quite large
        return vectorScale(result, v, 1.0f / sqrtf(normSq));
    } else {
        return vectorZero(result);
    }
}


static inline float vector2NormSquared(const fpVector2_t *a)
{
    return sq(a->x) + sq(a->y);
}

static inline float vector2Norm(const fpVector2_t *a)
{
    return sqrtf(vector2NormSquared(a));
}


static inline fpVector2_t * vector2Scale(fpVector2_t *result, const fpVector2_t *a, const float b)
{
    fpVector2_t ab;

    ab.x = a->x * b;
    ab.y = a->y * b;

    *result = ab;
    return result;
}

static inline fpVector2_t * vector2Normalize(fpVector2_t *result, const fpVector2_t *v)
{
    float normSq = vector2NormSquared(v);
    if (normSq > 0.0f) {
        return vector2Scale(result, v, 1.0f / sqrtf(normSq));
    } else {
        *result = (fpVector2_t){.x = 0.0f, .y = 0.0f};
        return result;
    }
}

static inline fpVector3_t * matrixVectorMul(fpVector3_t * result, const fpMat33_t * mat, const fpVector3_t * a)
{
    fpVector3_t r;

    r.x = mat->m[0][0] * a->x + mat->m[0][1] * a->y + mat->m[0][2] * a->z;
    r.y = mat->m[1][0] * a->x + mat->m[1][1] * a->y + mat->m[1][2] * a->z;
    r.z = mat->m[2][0] * a->x + mat->m[2][1] * a->y + mat->m[2][2] * a->z;

    *result = r;
    return result;
}

static inline fpMat33_t * yawToRotationMatrixZ(fpMat33_t * result, const float yaw)
{
    fpMat33_t r;
    const float sinYaw = sin_approx(yaw);
    const float cosYaw = cos_approx(yaw);

    r.m[0][0] = cosYaw;
    r.m[1][0] = sinYaw;
    r.m[2][0] = 0.0f;
    r.m[0][1] = -sinYaw;
    r.m[1][1] = cosYaw;
    r.m[2][1] = 0.0f;
    r.m[0][2] = 0.0f;
    r.m[1][2] = 0.0f;
    r.m[2][2] = 1.0f;

    *result = r;
    return result;
}

static inline fpVector3_t * matrixTrnVectorMul(fpVector3_t * result, const fpMat33_t * mat, const fpVector3_t * a)
{
    fpVector3_t r;

    r.x = mat->m[0][0] * a->x + mat->m[1][0] * a->y + mat->m[2][0] * a->z;
    r.y = mat->m[0][1] * a->x + mat->m[1][1] * a->y + mat->m[2][1] * a->z;
    r.z = mat->m[0][2] * a->x + mat->m[1][2] * a->y + mat->m[2][2] * a->z;

    *result = r;
    return result;
}

static inline float vector2Cross(const fpVector2_t *a, const fpVector2_t *b)
{
    return a->x * b->y - a->y * b->x;
}

static inline float vector2Dot(const fpVector2_t *a, const fpVector2_t *b)
{
    return a->x * b->x + a->y * b->y;
}

