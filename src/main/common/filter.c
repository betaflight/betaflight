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

/**
 * Typical quadcopter motor noise frequency (at 50% throttle):
 *  450-sized, 920kv, 9.4x4.3 props, 3S : 4622rpm = 77Hz
 *  250-sized, 2300kv, 5x4.5 props, 4S : 14139rpm = 235Hz
 */
static int8_t gyroFIRCoeff_500[3][FILTER_TAPS] = { { 0, 18, 14, 16, 20, 22, 24, 25, 25, 24, 20, 18, 12, 18 },  // TODO - NEEDS TABLE
                                                  { 23, 12, 16, 18, 20, 20, 23, 20, 22, 18, 16, 14, 12, 22 },  //
                                                  { 36, 12, 14, 14, 16, 16, 18, 18, 18, 16, 16, 14, 12, 36 } };//
static int8_t gyroFIRCoeff_1000[3][FILTER_TAPS] = { { 0, 0, 0, 0, 0, 0, 0, 12, 23, 40, 51, 52, 40, 38 }, // looptime=1000; group delay 2.5ms; -0.5db = 32Hz ; -1db = 45Hz; -5db = 97Hz; -10db = 132Hz
                                                  { 0, 0, 0, 0, 0, 18, 30, 42, 46, 40, 34, 22, 8, 8},    // looptime=1000; group delay 3ms;   -0.5db = 18Hz ; -1db = 33Hz; -5db = 81Hz; -10db = 113Hz
                                                  { 0, 0, 0, 0, 0, 18, 12, 28, 40, 44, 40, 32, 22, 20} };// looptime=1000; group delay 4ms;   -0.5db = 23Hz ; -1db = 35Hz; -5db = 75Hz; -10db = 103Hz
static int8_t gyroFIRCoeff_2000[3][FILTER_TAPS] = { { 0, 0, 0, 0, 0, 0, 0, 0, 6, 24, 58, 82, 64, 20 },   // looptime=2000, group delay 4ms;   -0.5db = 21Hz ; -1db = 31Hz; -5db = 71Hz; -10db = 99Hz
                                                  { 0, 0, 0, 0, 0, 0, 0, 14, 22, 46, 60, 56, 40, 24},    // looptime=2000, group delay 5ms;   -0.5db = 20Hz ; -1db = 26Hz; -5db = 52Hz; -10db = 71Hz
                                                  { 0, 0, 0, 0, 0, 14, 12, 26, 38, 44, 42, 34, 24, 20} };// looptime=2000, group delay 7ms;   -0.5db = 11Hz ; -1db = 18Hz; -5db = 38Hz; -10db = 52Hz
static int8_t gyroFIRCoeff_3000[3][FILTER_TAPS] = { { 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 36, 88, 88, 44 },    // looptime=3000, group delay 4.5ms; -0.5db = 18Hz ; -1db = 26Hz; -5db = 57Hz; -10db = 78Hz
                                                  { 0, 0, 0, 0, 0, 0, 0, 0, 14, 32, 64, 72, 54, 28},     // looptime=3000, group delay 6.5ms; -0.5db = 16Hz ; -1db = 21Hz; -5db = 42Hz; -10db = 57Hz
                                                  { 0, 0, 0, 0, 0, 0, 6, 10, 28, 44, 54, 54, 38, 22} };  // looptime=3000, group delay 9ms;   -0.5db = 10Hz ; -1db = 13Hz; -5db = 32Hz; -10db = 45Hz

int8_t * filterGetFIRCoefficientsTable(uint8_t filter_level, uint16_t targetLooptime)
{
    if (filter_level == 0) {
        return NULL;
    }

    int firIndex = constrain(filter_level, 1, 3) - 1;

    // For looptimes faster than  700 use filter for 2kHz looptime
    if (targetLooptime < 700) {
        return gyroFIRCoeff_500[firIndex];
    }

    // For looptimes faster than 1499 use filter for 1kHz looptime
    if (targetLooptime < 1500) {
        return gyroFIRCoeff_1000[firIndex];
    }
    // 1500 ... 2499
    else if (targetLooptime < 2500) {
        return gyroFIRCoeff_2000[firIndex];
    }
    // > 2500
    else {
        return gyroFIRCoeff_3000[firIndex];
    }
}

// Thanks to Qcopter & BorisB & DigitalEntity
void filterApplyFIR(int16_t data[3], int16_t state[3][FILTER_TAPS], int8_t coeff[FILTER_TAPS])
{
    int32_t FIRsum;
    int axis, i;

    for (axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        FIRsum = 0;
        for (i = 0; i <= FILTER_TAPS-2; i++) {
            state[axis][i] = state[axis][i + 1];
            FIRsum += state[axis][i] * (int16_t)coeff[i];
        }
        state[axis][FILTER_TAPS-1] = data[axis];
        FIRsum += state[axis][FILTER_TAPS-1] * coeff[FILTER_TAPS-1];
        data[axis] = FIRsum / 256;
    }
}
