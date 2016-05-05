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
#include <stdlib.h>
#include <math.h>

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"

#include "drivers/gyro_sync.h"

#define BIQUAD_Q    (1.0f / 1.41421356f)     /* quality factor - butterworth (1 / sqrt(2)) */

/* sets up a biquad Filter */
void filterInitBiQuad(uint8_t filterCutFreq, biquad_t *newState, int16_t samplingRate)
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
float filterApplyBiQuad(float sample, biquad_t *state)
{
    float result;

    result = state->b0 * sample + state->d1;
    state->d1 = state->b1 * sample - state->a1 * result + state->d2;
    state->d2 = state->b2 * sample - state->a2 * result;

    return result;
}

// PT1 Low Pass filter (when no dT specified it will be calculated from the cycleTime)
float filterApplyPt1(float input, filterStatePt1_t *filter, float f_cut, float dT)
{
	// Pre calculate and store RC
	if (!filter->RC) {
		filter->RC = 1.0f / ( 2.0f * (float)M_PI * f_cut );
	}

    filter->state = filter->state + dT / (filter->RC + dT) * (input - filter->state);
    return filter->state;
}

void filterResetPt1(filterStatePt1_t *filter, float input)
{
    filter->state = input;
}

void filterUpdateFIR(int filterLength, float *shiftBuf, float newSample)
{
    // Shift history buffer and push new sample
    for (int i = filterLength - 1; i > 0; i--)
        shiftBuf[i] = shiftBuf[i - 1];

    shiftBuf[0] = newSample;
}

float filterApplyFIR(int filterLength, const float *shiftBuf, const float *coeffBuf, float commonMultiplier)
{
    float accum = 0;

    for (int i = 0; i < filterLength; i++)
        accum += shiftBuf[i] * coeffBuf[i];

    return accum * commonMultiplier;
}
