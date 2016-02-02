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

#define BIQUAD_BANDWIDTH 1.9f     /* bandwidth in octaves */

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
    alpha = sn * sin_approx(M_LN2f / 2 * BIQUAD_BANDWIDTH * omega / sn);

    b0 = (1 - cs) / 2;
    b1 = 1 - cs;
    b2 = (1 - cs) / 2;
    a0 = 1 + alpha;
    a1 = -2 * cs;
    a2 = 1 - alpha;

    /* precompute the coefficients */
    newState->a0 = b0 / a0;
    newState->a1 = b1 / a0;
    newState->a2 = b2 / a0;
    newState->a3 = a1 / a0;
    newState->a4 = a2 / a0;

    /* zero initial samples */
    newState->x1 = newState->x2 = 0;
    newState->y1 = newState->y2 = 0;
}

/* Computes a biquad_t filter on a sample */
float filterApplyBiQuad(float sample, biquad_t *state)
{
    float result;

    /* compute result */
    result = state->a0 * sample + state->a1 * state->x1 + state->a2 * state->x2 -
        state->a3 * state->y1 - state->a4 * state->y2;

    /* shift x1 to x2, sample to x1 */
    state->x2 = state->x1;
    state->x1 = sample;

    /* shift y1 to y2, result to y1 */
    state->y2 = state->y1;
    state->y1 = result;

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
