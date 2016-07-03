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

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "common/filter.h"
#include "common/maths.h"

#define M_LN2_FLOAT 0.69314718055994530942f
#define M_PI_FLOAT  3.14159265358979323846f

#define BIQUAD_BANDWIDTH 1.9f     /* bandwidth in octaves */
#define BIQUAD_Q 1.0f / sqrtf(2.0f)     /* quality factor - butterworth*/

// PT1 Low Pass filter

void pt1FilterInit(pt1Filter_t *filter, uint8_t f_cut, float dT)
{
    filter->RC = 1.0f / ( 2.0f * M_PI_FLOAT * f_cut );
    filter->dT = dT;
}

float pt1FilterApply(pt1Filter_t *filter, float input)
{
    filter->state = filter->state + filter->dT / (filter->RC + filter->dT) * (input - filter->state);
    return filter->state;
}

float pt1FilterApply4(pt1Filter_t *filter, float input, uint8_t f_cut, float dT)
{
    // Pre calculate and store RC
    if (!filter->RC) {
        filter->RC = 1.0f / ( 2.0f * M_PI_FLOAT * f_cut );
        filter->dT = dT;
    }

    filter->state = filter->state + filter->dT / (filter->RC + filter->dT) * (input - filter->state);

    return filter->state;
}

/* sets up a biquad Filter */
void biquadFilterInit(biquadFilter_t *filter, float filterCutFreq, uint32_t refreshRate)
{
    const float sampleRate = 1 / ((float)refreshRate * 0.000001f);

    // setup variables
    const float omega = 2 * M_PI_FLOAT * filterCutFreq / sampleRate;
    const float sn = sinf(omega);
    const float cs = cosf(omega);
    //this is wrong, should be hyperbolic sine
    //alpha = sn * sinf(M_LN2_FLOAT /2 * BIQUAD_BANDWIDTH * omega /sn);
    const float alpha = sn / (2 * BIQUAD_Q);

    const float b0 = (1 - cs) / 2;
    const float b1 = 1 - cs;
    const float b2 = (1 - cs) / 2;
    const float a0 = 1 + alpha;
    const float a1 = -2 * cs;
    const float a2 = 1 - alpha;

    // precompute the coefficients
    filter->b0 = b0 / a0;
    filter->b1 = b1 / a0;
    filter->b2 = b2 / a0;
    filter->a1 = a1 / a0;
    filter->a2 = a2 / a0;

    // zero initial samples
    filter->d1 = filter->d2 = 0;
}

/* sets up a biquad Filter */
void biquadFilterNotchInit(biquadFilter_t *filter, float filterCutFreq, uint32_t refreshRate, float Q)
{
    const float sampleRate = 1 / ((float)refreshRate * 0.000001f);

    // setup variables
    const float omega = 2 * M_PI_FLOAT * filterCutFreq / sampleRate;
    const float sn = sinf(omega);
    const float cs = cosf(omega);
    //this is wrong, should be hyperbolic sine
    //alpha = sn * sinf(M_LN2_FLOAT /2 * BIQUAD_BANDWIDTH * omega /sn);
    const float alpha = sn / (2 * Q);

    const float b0 =  1;
    const float b1 = -2 * cs;
    const float b2 =  1;
    const float a0 =  1 + alpha;
    const float a1 = -2 * cs;
    const float a2 =  1 - alpha;

    // precompute the coefficients
    filter->b0 = b0 / a0;
    filter->b1 = b1 / a0;
    filter->b2 = b2 / a0;
    filter->a1 = a1 / a0;
    filter->a2 = a2 / a0;

    // zero initial samples
    filter->d1 = filter->d2 = 0;
}

/* Computes a biquad_t filter on a sample */
float biquadFilterApply(biquadFilter_t *filter, float input)
{
    const float result = filter->b0 * input + filter->d1;
    filter->d1 = filter->b1 * input - filter->a1 * result + filter->d2;
    filter->d2 = filter->b2 * input - filter->a2 * result;
    return result;
}

int32_t filterApplyAverage(int32_t input, uint8_t averageCount, int32_t averageState[DELTA_MAX_SAMPLES]) {
    int count;
    int32_t averageSum = 0;

    for (count = averageCount-1; count > 0; count--) averageState[count] = averageState[count-1];
    averageState[0] = input;
    for (count = 0; count < averageCount; count++) averageSum += averageState[count];

    return averageSum / averageCount;
}

float filterApplyAveragef(float input, uint8_t averageCount, float averageState[DELTA_MAX_SAMPLES]) {
    int count;
    float averageSum = 0.0f;

    for (count = averageCount-1; count > 0; count--) averageState[count] = averageState[count-1];
    averageState[0] = input;
    for (count = 0; count < averageCount; count++) averageSum += averageState[count];

    return averageSum / averageCount;
}

