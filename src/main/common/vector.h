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
} vector2_t;

typedef union {
    float v[3];
    struct {
       float x, y, z;
    };
} vector3_t;

typedef struct {
    float m[3][3];
} matrix33_t;

float vector2Dot(const vector2_t *a, const vector2_t *b);
float vector2Cross(const vector2_t *a, const vector2_t *b);
float vector2Mag(const vector2_t *v);

void vectorZero(vector3_t *v);
void vectorAdd(vector3_t *result, const vector3_t *a, const vector3_t *b);
void vectorScale(vector3_t *result, const vector3_t *a, const float k);
void vectorCrossProduct(vector3_t *result, const vector3_t *a, const vector3_t *b);
float vectorNormSquared(const vector3_t *v);
float vectorNorm(const vector3_t *v);
void vectorNormalize(vector3_t *result, const vector3_t *v);

void matrixVectorMul(vector3_t *result, const matrix33_t *mat, const vector3_t *v);
void matrixTrnVectorMul(vector3_t *result, const matrix33_t *mat, const vector3_t *v);
