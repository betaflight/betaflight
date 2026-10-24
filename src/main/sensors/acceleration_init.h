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

#pragma once

#include "platform.h"

#include "common/axis.h"
#include "common/filter.h"

#include "sensors/acceleration.h"
#include "sensors/sensors.h"

typedef struct accelerationRuntime_s {
    uint16_t accLpfCutHz;
    pt2Filter_t accFilter[XYZ_AXIS_COUNT];
#ifdef USE_MOTOR_EXCITATION
    pt2Filter_t tapFilter[XYZ_AXIS_COUNT];   // bench tap capture: fixed cutoff, independent of acc_lpf_hz
#endif
    flightDynamicsTrims_t *accelerationTrims;
    uint16_t calibratingA;      // the calibration is done is the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
} accelerationRuntime_t;

extern accelerationRuntime_t accelerationRuntime;

void performAccelerometerCalibration(rollAndPitchTrims_t *rollAndPitchTrims);
void performInflightAccelerationCalibration(rollAndPitchTrims_t *rollAndPitchTrims);
