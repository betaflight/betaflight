/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * some functions are taken from https://github.com/iNavFlight/inav/
 */

#pragma once

#include <stdbool.h>

#include "common/maths.h"

typedef union vector2_u {
    float v[2];
    struct {
       float x, y;
    };
} vector2_t;

typedef union vector3_u {
    float v[3];
    struct {
       float x, y, z;
    };
} vector3_t;

typedef struct matrix33_s {
    float m[3][3];
} matrix33_t;

bool vector2Equal(const vector2_t *a, const vector2_t *b);
vector2_t *vector2Zero(vector2_t *v);
vector2_t *vector2Add(vector2_t *result, const vector2_t *a, const vector2_t *b);
vector2_t *vector2Sub(vector2_t *result, const vector2_t *a, const vector2_t *b);
vector2_t *vector2Scale(vector2_t *result, const vector2_t *v, const float k);
float vector2Dot(const vector2_t *a, const vector2_t *b);
float vector2Cross(const vector2_t *a, const vector2_t *b);
float vector2NormSq(const vector2_t *v);
float vector2Norm(const vector2_t *v);
vector2_t *vector2Normalize(vector2_t *result, const vector2_t *v);

bool vector3Equal(const vector3_t *a, const vector3_t *b);
vector3_t *vector3Zero(vector3_t *v);
vector3_t *vector3Add(vector3_t *result, const vector3_t *a, const vector3_t *b);
vector3_t *vector3Sub(vector3_t *result, const vector3_t *a, const vector3_t *b);
vector3_t *vector3Scale(vector3_t *result, const vector3_t *v, const float k);
float vector3Dot(const vector3_t *a, const vector3_t *b);
vector3_t *vector3Cross(vector3_t *result, const vector3_t *a, const vector3_t *b);
float vector3NormSq(const vector3_t *v);
float vector3Norm(const vector3_t *v);
vector3_t *vector3Normalize(vector3_t *result, const vector3_t *v);

vector3_t *matrixVectorMul(vector3_t *result, const matrix33_t *mat, const vector3_t *v);
vector3_t *matrixTrnVectorMul(vector3_t *result, const matrix33_t *mat, const vector3_t *v);

matrix33_t *buildRotationMatrix(matrix33_t *result, const fp_angles_t *rpy);
vector3_t *applyRotationMatrix(vector3_t *v, const matrix33_t *rotationMatrix);

matrix33_t *yawToRotationMatrixZ(matrix33_t *result, const float yaw);
