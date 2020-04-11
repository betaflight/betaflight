/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "flight/imu.h"
#include "flight/propwash_control.h"

#include "sensors/acceleration.h"

#define GRAVITY_THRESHOLD 0.4f
#define ROLL_MAX_ANGLE    900
#define PITCH_MAX_ANGLE   650

#define MAX_DTERM_BOOST   0.5f
#define MAX_DLPF_BOOST    0.5f

bool isInPropwashZone = false;

void checkPropwash(void) {
    const float zAxisAcc = acc.accADC[Z] * acc.dev.acc_1G_rec;

    if (zAxisAcc < GRAVITY_THRESHOLD && ABS(attitude.raw[FD_ROLL]) < ROLL_MAX_ANGLE && ABS(attitude.raw[FD_PITCH]) < PITCH_MAX_ANGLE) {
        isInPropwashZone = true;
    } else {
        isInPropwashZone = false;
    }
    DEBUG_SET(DEBUG_PROPWASH, 0, lrintf(zAxisAcc * 100));
    DEBUG_SET(DEBUG_PROPWASH, 1, isInPropwashZone);
}

bool canApplyBoost() {
    //TODO check throttle, time, etc.
    return true;
}

float computeBoostFactor() {
    if (canApplyBoost) {
        const float zAxisAcc = acc.accADC[Z] * acc.dev.acc_1G_rec;
        const float xAxisAcc = acc.accADC[X] * acc.dev.acc_1G_rec;
        
        const float accel = sqrt(powf(zAxisAcc, 2) + powf(xAxisAcc, 2));
        
        return MIN(1 + accel * MAX_DTERM_BOOST, 1 + MAX_DTERM_BOOST);
    } else {
        return 1.0f;
    }
}
