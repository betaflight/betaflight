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

#include "platform.h"

#include "pwl.h"


void pwlInitialize(pwl_t *pwl, float *yValues, int numPoints, float xMin, float xMax) {
    pwl->numPoints = numPoints;
    pwl->xMin = xMin;
    pwl->xMax = xMax;
    pwl->dx = (xMax - xMin) / (numPoints - 1);
    pwl->yValues = yValues;
}

void pwlFill(pwl_t *pwl, float (*function)(float))
{
    for (int i = 0; i < pwl->numPoints; ++i) {
        const float x = pwl->xMin + i * pwl->dx;
        pwl->yValues[i] = function(x);
    }
}

float pwlInterpolate(const pwl_t *pwl, float x)
{
    if (x <= pwl->xMin) {
        return pwl->yValues[0];
    }
    
    if (x >= pwl->xMax) {
        return pwl->yValues[pwl->numPoints - 1];
    }
    
    const int index = (int)((x - pwl->xMin) / pwl->dx);
    if (index >= pwl->numPoints - 1) {
        return pwl->yValues[pwl->numPoints - 1];
    }

    const float x0 = pwl->xMin + index * pwl->dx;
    const float y0 = pwl->yValues[index];
    const float y1 = pwl->yValues[index + 1];

    return y0 + (x - x0) * (y1 - y0) / pwl->dx;
}
