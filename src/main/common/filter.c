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


// PT1 Low Pass filter (when no dT specified it will be calculated from the cycleTime)
float filterApplyPt1(float input, filterStatePt1_t *filter, uint8_t f_cut, float dT) {

	// Pre calculate and store RC
	if (!filter->RC) {
		filter->RC = 1.0f / ( 2.0f * (float)M_PI * f_cut );
	}

    filter->state = filter->state + dT / (filter->RC + dT) * (input - filter->state);

    return filter->state;
}

static int8_t gyroFIRCoeff_500[FILTER_TAPS] = { 0, 18, 14, 16, 20, 22, 24, 25, 25, 24, 20, 18, 12, 18 };  // looptime=500;
static int8_t gyroFIRCoeff_1000[FILTER_TAPS] = { 0, 0, 0, 0, 0, 0, 0, 12, 23, 40, 51, 52, 40, 38 }; // looptime=1000; group delay 2.5ms; -0.5db = 32Hz ; -1db = 45Hz; -5db = 97Hz; -10db = 132Hz


static int8_t deltaFIRCoeff_500[FILTER_TAPS] = {0, 0, 0, 0, 0, 18, 12, 28, 40, 44, 40, 32, 22, 20};
static int8_t deltaFIRCoeff_1000[FILTER_TAPS] = {36, 12, 14, 14, 16, 16, 18, 18, 18, 16, 16, 14, 12, 36};

int8_t * filterGetFIRCoefficientsTable(uint8_t filter_type, uint32_t targetLooptime)
{
	int8_t *filterCoeff;

    switch(filter_type){
        case(0):
		    filterCoeff = NULL;
            break;
        case(1):
            if (targetLooptime == 500) {
                filterCoeff = gyroFIRCoeff_500;
            } else {    // filter for 1kHz looptime
                filterCoeff = gyroFIRCoeff_1000;
            }
            break;
        case(2):
            if (targetLooptime == 500) {
                filterCoeff = deltaFIRCoeff_500;
            } else {    // filter for 1kHz looptime
                filterCoeff = deltaFIRCoeff_1000;
            }
    }
    return filterCoeff;
}

// Thanks to Qcopter & BorisB & DigitalEntity
void filterApplyFIR(int16_t *data, int16_t state[FILTER_TAPS], int8_t coeff[FILTER_TAPS])
{
    int32_t FIRsum;
    FIRsum = 0;
    int i;

    for (i = 0; i <= FILTER_TAPS-2; i++) {
        state[i] = state[i + 1];
        FIRsum += state[i] * (int16_t)coeff[i];
    }
    state[FILTER_TAPS-1] = *data;
    FIRsum += state[FILTER_TAPS-1] * coeff[FILTER_TAPS-1];
    *data = FIRsum / 256;
}
