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

#include <stdint.h>

#include "common/axis.h"
#include "flight/pid.h"

typedef enum ffInterpolationType_e {
    FF_INTERPOLATE_OFF,
    FF_INTERPOLATE_ON,
    FF_INTERPOLATE_AVG2,
    FF_INTERPOLATE_AVG3,
    FF_INTERPOLATE_AVG4
} ffInterpolationType_t;

void interpolatedSpInit(const pidProfile_t *pidProfile);
float interpolatedSpApply(int axis, bool newRcFrame, ffInterpolationType_t type);
float applyFfLimit(int axis, float value, float Kp, float currentPidSetpoint);
bool shouldApplyFfLimits(int axis);
