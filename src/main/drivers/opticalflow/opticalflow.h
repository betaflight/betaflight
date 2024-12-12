/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>
#include "common/vector.h"

#define OPTICALFLOW_OUT_OF_RANGE        -1
#define OPTICALFLOW_HARDWARE_FAILURE    -2
#define OPTICALFLOW_NO_NEW_DATA         -3

#define OPTICALFLOW_HARDWARE_TIMEOUT_US  100000 /* 100ms */

typedef struct opticalflowData_s {
    uint32_t timeStampUs;
    int16_t quality;
    vector2_t flowRate;
} opticalflowData_t;

struct opticalflowDev_s;
typedef void opflowOpInitFunc(struct opticalflowDev_s * dev);
typedef void opflowOpUpdateFunc(struct opticalflowDev_s * dev);
typedef void opflowOpReadFunc(struct opticalflowDev_s * dev, opticalflowData_t * result);

typedef struct opticalflowDev_s {
    timeMs_t delayMs;
    int16_t minRangeCm;
    uint8_t minQualityThreshold;
    
    opflowOpInitFunc  *init;
    opflowOpUpdateFunc *update;
    opflowOpReadFunc  *read;
} opticalflowDev_t;
