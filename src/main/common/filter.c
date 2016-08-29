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
#include <string.h>
#include <math.h>

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"

#include "drivers/gyro_sync.h"

#define BIQUAD_Q    (1.0f / 1.41421356f)     /* quality factor - butterworth (1 / sqrt(2)) */
#define M_PI_FLOAT  3.14159265358979323846f

void biquadFilterInit(biquadFilter_t *filter, uint32_t filterCutFreq, uint32_t refreshRate)
{
    // setup variables
    const float sampleRate = 1.0f / ((float)refreshRate * 0.000001f);
    const float omega = 2 * M_PIf * ((float)filterCutFreq) / sampleRate;
    const float sn = sin_approx(omega);
    const float cs = cos_approx(omega);
    const float alpha = sn / (2 * BIQUAD_Q);

    float a0, a1, a2, b0, b1, b2;

    b0 = (1 - cs) / 2;
    b1 = 1 - cs;
    b2 = (1 - cs) / 2;
    a0 = 1 + alpha;
    a1 = -2 * cs;
    a2 = 1 - alpha;

    // precompute the coefficients
    filter->b0 = b0 / a0;
    filter->b1 = b1 / a0;
    filter->b2 = b2 / a0;
    filter->a1 = a1 / a0;
    filter->a2 = a2 / a0;

    // zero initial samples
    filter->d1 = filter->d2 = 1;
}

/* Computes a biquad_t filter on a sample */
float biquadFilterApply(biquadFilter_t *filter, float input)
{
    const float result = filter->b0 * input + filter->d1;
    filter->d1 = filter->b1 * input - filter->a1 * result + filter->d2;
    filter->d2 = filter->b2 * input - filter->a2 * result;
    return result;
}

// PT1 Low Pass filter

// f_cut = cutoff frequency
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

float pt1FilterApply4(pt1Filter_t *filter, float input, float f_cut, float dT)
{
    // Pre calculate and store RC
    if (!filter->RC) {
        filter->RC = 1.0f / ( 2.0f * (float)M_PI * f_cut );
    }

    filter->state = filter->state + dT / (filter->RC + dT) * (input - filter->state);
    return filter->state;
}

void pt1FilterReset(pt1Filter_t *filter, float input)
{
    filter->state = input;
}

// rate_limit = maximum rate of change of the output value in units per second
void rateLimitFilterInit(rateLimitFilter_t *filter)
{
    filter->state = 0;
}

float rateLimitFilterApply4(rateLimitFilter_t *filter, float input, float rate_limit, float dT)
{
    if (rate_limit > 0) {
        const float rateLimitPerSample = rate_limit * dT;
        filter->state = constrainf(input, filter->state - rateLimitPerSample, filter->state + rateLimitPerSample);
    }
    else {
        filter->state = input;
    }

    return filter->state;
}

// FIR filter
void firFilterInit2(firFilter_t *filter, float *buf, uint8_t bufLength, const float *coeffs, uint8_t coeffsLength)
{
    filter->buf = buf;
    filter->bufLength = bufLength;
    filter->coeffs = coeffs;
    filter->coeffsLength = coeffsLength;
    memset(filter->buf, 0, sizeof(float) * filter->bufLength);
}

void firFilterInit(firFilter_t *filter, float *buf, uint8_t bufLength, const float *coeffs)
{
    firFilterInit2(filter, buf, bufLength, coeffs, bufLength);
}

void firFilterUpdate(firFilter_t *filter, float input)
{
    memmove(&filter->buf[1], &filter->buf[0], (filter->bufLength-1) * sizeof(float));
    filter->buf[0] = input;
}

float firFilterApply(firFilter_t *filter)
{
    float ret = 0.0f;
    for (int ii = 0; ii < filter->coeffsLength; ++ii) {
        ret += filter->coeffs[ii] * filter->buf[ii];
    }
    return ret;
}
