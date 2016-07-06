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

/* sets up a biquad Filter */
void biquadFilterInit(biquadFilter_t *newState, uint8_t filterCutFreq, int16_t samplingRate)
{
    float omega, sn, cs, alpha;
    float a0, a1, a2, b0, b1, b2;

    /* If sampling rate == 0 - use main loop target rate */
    if (!samplingRate) {
        samplingRate = 1000000 / targetLooptime;
    }

    /* setup variables */
    omega = 2 * M_PIf * (float)filterCutFreq / (float)samplingRate;
    sn = sin_approx(omega);
    cs = cos_approx(omega);
    alpha = sn / (2 * BIQUAD_Q);

    b0 = (1 - cs) / 2;
    b1 = 1 - cs;
    b2 = (1 - cs) / 2;
    a0 = 1 + alpha;
    a1 = -2 * cs;
    a2 = 1 - alpha;

    /* precompute the coefficients */
    newState->b0 = b0 / a0;
    newState->b1 = b1 / a0;
    newState->b2 = b2 / a0;
    newState->a1 = a1 / a0;
    newState->a2 = a2 / a0;

    /* zero initial samples */
    newState->d1 = newState->d2 = 1;
}

/* Computes a biquad_t filter on a sample */
float biquadFilterApply(biquadFilter_t *state, float sample)
{
    float result;

    result = state->b0 * sample + state->d1;
    state->d1 = state->b1 * sample - state->a1 * result + state->d2;
    state->d2 = state->b2 * sample - state->a2 * result;

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

// f_cut = cutoff frequency
// rate_limit = maximum rate of change of the output value in units per second
float pt1FilterApplyWithRateLimit(pt1Filter_t *filter, float input, float f_cut, float rate_limit, float dT)
{
    // Pre calculate and store RC
    if (!filter->RC) {
        filter->RC = 1.0f / ( 2.0f * (float)M_PI * f_cut );
    }

    const float newState = filter->state + dT / (filter->RC + dT) * (input - filter->state);
    const float rateLimitPerSample = rate_limit * dT;
    filter->state = constrainf(newState, filter->state - rateLimitPerSample, filter->state + rateLimitPerSample);

    return filter->state;
}

void pt1FilterReset(pt1Filter_t *filter, float input)
{
    filter->state = input;
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
