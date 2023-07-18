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

float vector2Dot(const fpVector2_t *a, const fpVector2_t *b);
float vector2Cross(const fpVector2_t *a, const fpVector2_t *b);
float vector2Mag(const fpVector2_t *a);

fpVector3_t * vectorZero(fpVector3_t *v);
fpVector3_t * vectorAdd(fpVector3_t *result, const fpVector3_t *a, const fpVector3_t *b);
fpVector3_t * vectorScale(fpVector3_t *result, const fpVector3_t *a, const float b);
fpVector3_t * vectorCrossProduct(fpVector3_t *result, const fpVector3_t *a, const fpVector3_t *b);
float vectorNormSquared(const fpVector3_t * v);
float vectorNorm(const fpVector3_t * v);
fpVector3_t * vectorNormalize(fpVector3_t *result, const fpVector3_t *v);

fpVector3_t * matrixVectorMul(fpVector3_t * result, const fpMat33_t * mat, const fpVector3_t * a);
fpVector3_t * matrixTrnVectorMul(fpVector3_t * result, const fpMat33_t * mat, const fpVector3_t * a);
