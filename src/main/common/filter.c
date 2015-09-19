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

#include "common/filter.h"
#include "common/axis.h"


// PT1 Low Pass filter (when no dT specified it will be calculated from the cycleTime)
float filterApplyPt1(float input, filterStatePt1_t *filter, uint8_t f_cut, float dT) {

	// Pre calculate and store RC
	if (!filter->RC) {
		filter->RC = 1.0f / ( 2.0f * (float)M_PI * f_cut );
	}

    filter->state = filter->state + dT / (filter->RC + dT) * (input - filter->state);

    return filter->state;
}

// 7 Tap FIR filter as described here:
// Thanks to Qcopter
void filterApply7TapFIR(int16_t data[]) {
    int16_t FIRcoeff[7] = { 12, 23, 40, 51, 52, 40, 38 }; // TODO - More coefficients needed. Now fixed to 1khz
    static int16_t gyro_delay[3][7] = { {0}, {0}, {0} };
    int32_t FIRsum;
    int axis, i;

    // 7 tap FIR, <-20dB at >170Hz with looptime 1ms, groupdelay = 2.5ms
    for (axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        FIRsum = 0;
        for (i = 0; i <= 5; i++) {    
            gyro_delay[axis][i] = gyro_delay[axis][i + 1];
            FIRsum += gyro_delay[axis][i] * FIRcoeff[i];
        }
        gyro_delay[axis][6] = data[axis];
        FIRsum += gyro_delay[axis][6] * FIRcoeff[6];
        data[axis] = FIRsum / 256;
    }
}
