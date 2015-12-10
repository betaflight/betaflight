/*
 * filter.c
 *
 *  Created on: 24 jun. 2015
 *      Author: borisb
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "common/axis.h"
#include "common/filter.h"
#include "common/axis.h"
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
static int8_t gyroFIRCoeff_1000[7] = { 12, 23, 40, 51, 52, 40, 38 };   // 1khz; group delay 2.5ms; -0.5db = 32Hz ; -1db = 45Hz; -5db = 97Hz; -10db = 132hz

int8_t * filterGetFIRCoefficientsTable(void)
{
    return gyroFIRCoeff_1000;
}

// 9 Tap FIR filter as described here:
// Thanks to Qcopter & BorisB & DigitalEntity
void filterApplyFIR(int16_t data[3], int16_t state[3][7], int8_t coeff[7])
{
    int32_t FIRsum;
    int axis, i;

    for (axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        FIRsum = 0;
        for (i = 0; i <= 7; i++) {
            state[axis][i] = state[axis][i + 1];
            FIRsum += state[axis][i] * (int16_t)coeff[i];
        }
        state[axis][8] = data[axis];
        FIRsum += state[axis][8] * coeff[8];
        data[axis] = FIRsum / 256;
    }
}
