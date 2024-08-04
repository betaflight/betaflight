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

#pragma once

#include <stdint.h>

#include "utils.h"


#define PWL_DECLARE(name, size, xMinV, xMaxV)                       \
    STATIC_ASSERT(xMinV < xMaxV, "xMinV must be less than xMaxV");  \
    STATIC_ASSERT(size > 1, "size must be more than 1");            \
    STATIC_ASSERT(size < 33, "size must be less than 33");          \
    float name##_yValues[size];                                     \
    pwl_t name = {                                                  \
        .yValues = name##_yValues,                                  \
        .numPoints = size,                                          \
        .xMin = xMinV,                                              \
        .xMax = xMaxV,                                              \
        .dx = (xMaxV - xMinV) / (size - 1)                          \
    }

typedef struct pwl_s {
    float *yValues;
    int numPoints;
    float xMin;
    float xMax;
    float dx;
} pwl_t;

void pwlInitialize(pwl_t *pwl, float *yValues, int numPoints, float xMin, float xMax);
void pwlFill(pwl_t *pwl, float (*function)(float));
float pwlInterpolate(const pwl_t *pwl, float x);
