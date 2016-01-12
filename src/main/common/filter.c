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


// PT1 Low Pass filter (when no dT specified it will be calculated from the cycleTime)
float filterApplyPt1(float input, filterStatePt1_t *filter, uint8_t f_cut, float dT) {

	// Pre calculate and store RC
	if (!filter->RC) {
		filter->RC = 1.0f / ( 2.0f * (float)M_PI * f_cut );
	}

    filter->state = filter->state + dT / (filter->RC + dT) * (input - filter->state);

    return filter->state;
}

void setBiQuadCoefficients(int type, biquad_t *state) {

    /* zero initial samples */
    state->x1=0;
    state->x2=0;
    state->y1=0;
    state->y2=0;

	/* set coefficients */
	switch(type) {
	    case(GYRO_FILTER):
            if (targetLooptime == 500) {
                state->a0= 0.007820199;
                state->a1= 0.015640399;
                state->a2= 0.007820199;
                state->a3= -1.73472382;
                state->a4= 0.766004619;
            } else {
                state->a0= 0.027859711;
                state->a1= 0.055719422;
                state->a2= 0.027859711;
                state->a3= -1.47547752;
                state->a4= 0.586916365;
	        }
            break;
	    case(DELTA_FILTER):
            if (targetLooptime == 500) {
                state->a0= 0.003621679;
                state->a1= 0.007243357;
                state->a2= 0.003621679;
                state->a3= -1.82269350;
                state->a4= 0.837180216;
            } else {
                state->a0= 0.013359181;
                state->a1= 0.026718362;
                state->a2= 0.013359181;
                state->a3= -1.64745762;
                state->a4= 0.700894342;
            }
	}
}

/* Computes a BiQuad filter on a sample */
float applyBiQuadFilter(float sample, biquad_t *state)
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
