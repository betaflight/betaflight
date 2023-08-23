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

bool vector2Equal(const vector2_t *a, const vector2_t *b)
{
    return (a->x == b->x) && (a->y == b->y);
}

void vector2Zero(vector2_t *v)
{
    v->x = 0.0f;
    v->y = 0.0f;
}

void vector2Add(vector2_t *result, const vector2_t *a, const vector2_t *b)
{
    result->x = a->x + b->x;
    result->y = a->y + b->y;
}

void vector2Scale(vector2_t *result, const vector2_t *v, const float k)
{
    result->x = v->x * k;
    result->y = v->y * k;
}

float vector2Dot(const vector2_t *a, const vector2_t *b)
{
    return a->x * b->x + a->y * b->y;
}

float vector2Cross(const vector2_t *a, const vector2_t *b)
{
    return a->x * b->y - a->y * b->x;
}

float vector2NormSq(const vector2_t *v)
{
    return vector2Dot(v, v);
}

float vector2Norm(const vector2_t *v)
{
    return sqrtf(vector2NormSq(v));
}

void vector2Normalize(vector2_t *result, const vector2_t *v)
{
    const float normSq = vector2NormSq(v);

    if (normSq > 0.0f) {
        vector2Scale(result, v, 1.0f / sqrtf(normSq));
    } else {
        vector2Zero(result);
    }
}

bool vector3Equal(const vector3_t *a, const vector3_t *b)
{
    return (a->x == b->x) && (a->y == b->y) && (a->z == b->z);
}

void vector3Zero(vector3_t *v)
{
    v->x = 0.0f;
    v->y = 0.0f;
    v->z = 0.0f;
}

void vector3Add(vector3_t *result, const vector3_t *a, const vector3_t *b)
{
    result->x = a->x + b->x;
    result->y = a->y + b->y;
    result->z = a->z + b->z;
}

void vector3Scale(vector3_t *result, const vector3_t *v, const float k)
{
    result->x = v->x * k;
    result->y = v->y * k;
    result->z = v->z * k;
}

float vector3Dot(const vector3_t *a, const vector3_t *b)
{
    return a->x * b->x + a->y * b->y + a->z * b->z;
}

void vector3Cross(vector3_t *result, const vector3_t *a, const vector3_t *b)
{
    const vector3_t tmpA = *a;
    const vector3_t tmpB = *b;

    result->x = tmpA.y * tmpB.z - tmpA.z * tmpB.y;
    result->y = tmpA.z * tmpB.x - tmpA.x * tmpB.z;
    result->z = tmpA.x * tmpB.y - tmpA.y * tmpB.x;
}

float vector3NormSq(const vector3_t *v)
{
    return vector3Dot(v, v);
}

float vector3Norm(const vector3_t *v)
{
    return sqrtf(vector3NormSq(v));
}

void vector3Normalize(vector3_t *result, const vector3_t *v)
{
    const float normSq = vector3NormSq(v);

    if (normSq > 0) {  // Hopefully sqrt(nonzero) is quite large
        vector3Scale(result, v, 1.0f / sqrtf(normSq));
    } else {
        vector3Zero(result);
    }
}

void matrixVectorMul(vector3_t * result, const matrix33_t *mat, const vector3_t *v)
{
    const vector3_t tmp = *v;

    result->x = mat->m[0][0] * tmp.x + mat->m[0][1] * tmp.y + mat->m[0][2] * tmp.z;
    result->y = mat->m[1][0] * tmp.x + mat->m[1][1] * tmp.y + mat->m[1][2] * tmp.z;
    result->z = mat->m[2][0] * tmp.x + mat->m[2][1] * tmp.y + mat->m[2][2] * tmp.z;
}

void matrixTrnVectorMul(vector3_t *result, const matrix33_t *mat, const vector3_t *v)
{
    const vector3_t tmp = *v;

    result->x = mat->m[0][0] * tmp.x + mat->m[1][0] * tmp.y + mat->m[2][0] * tmp.z;
    result->y = mat->m[0][1] * tmp.x + mat->m[1][1] * tmp.y + mat->m[2][1] * tmp.z;
    result->z = mat->m[0][2] * tmp.x + mat->m[1][2] * tmp.y + mat->m[2][2] * tmp.z;
}

void buildRotationMatrix(matrix33_t *result, const fp_angles_t *rpy)
{
    const float cosx = cos_approx(rpy->angles.roll);
    const float sinx = sin_approx(rpy->angles.roll);
    const float cosy = cos_approx(rpy->angles.pitch);
    const float siny = sin_approx(rpy->angles.pitch);
    const float cosz = cos_approx(rpy->angles.yaw);
    const float sinz = sin_approx(rpy->angles.yaw);

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
    matrixTrnVectorMul((vector3_t *)v, rotationMatrix, (vector3_t *)v);
}
