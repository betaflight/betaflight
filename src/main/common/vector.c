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

#include "platform.h"

#include <math.h>

#include "common/axis.h"
#include "common/maths.h"

#include "vector.h"

float vector2Dot(const vector2_t *a, const vector2_t *b)
{
    return a->x * b->x + a->y * b->y;
}

float vector2Cross(const vector2_t *a, const vector2_t *b)
{
    return a->x * b->y - a->y * b->x;
}

float vector2Mag(const vector2_t *v)
{
    return sqrtf(sq(v->x) + sq(v->y));
}

void vectorZero(vector3_t *v)
{
    v->x = 0.0f;
    v->y = 0.0f;
    v->z = 0.0f;
}

void vectorAdd(vector3_t *result, const vector3_t *a, const vector3_t *b)
{
    result->x = a->x + b->x;
    result->y = a->y + b->y;
    result->z = a->z + b->z;
}

void vectorScale(vector3_t *result, const vector3_t *a, const float k)
{
    result->x = a->x * k;
    result->y = a->y * k;
    result->z = a->z * k;
}

void vectorCrossProduct(vector3_t *result, const vector3_t *a, const vector3_t *b)
{
    result->x = a->y * b->z - a->z * b->y;
    result->y = a->z * b->x - a->x * b->z;
    result->z = a->x * b->y - a->y * b->x;
}

float vectorNormSquared(const vector3_t *v)
{
    return sq(v->x) + sq(v->y) + sq(v->z);
}

float vectorNorm(const vector3_t *v)
{
    return sqrtf(vectorNormSquared(v));
}

void vectorNormalize(vector3_t *result, const vector3_t *v)
{
    float normSq = vectorNormSquared(v);
    if (normSq > 0) {              // Hopefully sqrt(nonzero) is quite large
        return vectorScale(result, v, 1.0f / sqrtf(normSq));
    } else {
        return vectorZero(result);
    }
}

void matrixVectorMul(vector3_t *result, const matrix33_t *mat, const vector3_t *v)
{
    result->x = mat->m[0][0] * v->x + mat->m[0][1] * v->y + mat->m[0][2] * v->z;
    result->y = mat->m[1][0] * v->x + mat->m[1][1] * v->y + mat->m[1][2] * v->z;
    result->z = mat->m[2][0] * v->x + mat->m[2][1] * v->y + mat->m[2][2] * v->z;
}

void matrixTrnVectorMul(vector3_t *result, const matrix33_t *mat, const vector3_t *v)
{
    result->x = mat->m[0][0] * v->x + mat->m[1][0] * v->y + mat->m[2][0] * v->z;
    result->y = mat->m[0][1] * v->x + mat->m[1][1] * v->y + mat->m[2][1] * v->z;
    result->z = mat->m[0][2] * v->x + mat->m[1][2] * v->y + mat->m[2][2] * v->z;
}

void buildRotationMatrix(matrix33_t *result, const fp_angles_t *delta)
{
    const float cosx = cos_approx(delta->angles.roll);
    const float sinx = sin_approx(delta->angles.roll);
    const float cosy = cos_approx(delta->angles.pitch);
    const float siny = sin_approx(delta->angles.pitch);
    const float cosz = cos_approx(delta->angles.yaw);
    const float sinz = sin_approx(delta->angles.yaw);

    const float coszcosx = cosz * cosx;
    const float sinzcosx = sinz * cosx;
    const float coszsinx = sinx * cosz;
    const float sinzsinx = sinx * sinz;

    result->m[0][X] = cosz * cosy;
    result->m[0][Y] = -cosy * sinz;
    result->m[0][Z] = siny;
    result->m[1][X] = sinzcosx + (coszsinx * siny);
    result->m[1][Y] = coszcosx - (sinzsinx * siny);
    result->m[1][Z] = -sinx * cosy;
    result->m[2][X] = (sinzsinx) - (coszcosx * siny);
    result->m[2][Y] = (coszsinx) + (sinzcosx * siny);
    result->m[2][Z] = cosy * cosx;
}

void applyRotationMatrix(float *v, const matrix33_t *rotationMatrix)
{
    const vector3_t vTmp = *(vector3_t *)v;
    matrixTrnVectorMul((vector3_t *)v, rotationMatrix, &vTmp);
}
