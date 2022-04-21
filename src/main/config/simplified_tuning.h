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

#include "flight/pid.h"
#include "sensors/gyro.h"

#define SIMPLIFIED_TUNING_PIDS_MIN 0
#define SIMPLIFIED_TUNING_FILTERS_MIN 10
#define SIMPLIFIED_TUNING_MAX 200
#define SIMPLIFIED_TUNING_DEFAULT 100
#define SIMPLIFIED_TUNING_D_DEFAULT 100

typedef enum {
    PID_SIMPLIFIED_TUNING_OFF = 0,
    PID_SIMPLIFIED_TUNING_RP,
    PID_SIMPLIFIED_TUNING_RPY,
    PID_SIMPLIFIED_TUNING_MODE_COUNT,
} pidSimplifiedTuningMode_e;

void applySimplifiedTuning(pidProfile_t *pidProfile, gyroConfig_t *gyroConfig);

void applySimplifiedTuningPids(pidProfile_t *pidProfile);
void applySimplifiedTuningDtermFilters(pidProfile_t *pidProfile);
void applySimplifiedTuningGyroFilters(gyroConfig_t *gyroConfig);

void disableSimplifiedTuning(pidProfile_t *pidProfile, gyroConfig_t *gyroConfig);
