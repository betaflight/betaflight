/*
 * This file is part of Betaflight and INAV
 *
 * Betaflight and INAV are free software. You can
 * redistribute this software and/or modify this software under
 * the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * Betaflight and INAV are distributed in the hope that
 * they will be useful, but WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

// MTF01
#define RANGEFINDER_MTF01_MAX_RANGE_CM    800
#define RANGEFINDER_MTF01_TASK_PERIOD_MS  10   // 100Hz

// MTF02
#define RANGEFINDER_MTF02_MAX_RANGE_CM    250
#define RANGEFINDER_MTF02_TASK_PERIOD_MS  20   // 50Hz

// MTF01P
#define RANGEFINDER_MTF01P_MAX_RANGE_CM    1200
#define RANGEFINDER_MTF01P_TASK_PERIOD_MS  10  // 100Hz

// MTF02P
#define RANGEFINDER_MTF02P_MAX_RANGE_CM    600
#define RANGEFINDER_MTF02P_TASK_PERIOD_MS  20  // 50Hz

// MT01P
#define RANGEFINDER_MT01P_MAX_RANGE_CM    1000
#define RANGEFINDER_MT01P_TASK_PERIOD_MS  20   // 50Hz   

#define RANGEFINDER_MT_DETECTION_CONE_DECIDEGREES  900 

#include "drivers/rangefinder/rangefinder.h"

bool mtRangefinderDetect(rangefinderDev_t * dev, uint8_t mtRangefinderToUse);
void mtRangefinderReceiveNewData(uint8_t * bufferPtr);
uint16_t getMtRangefinderTaskPeriodMs(uint8_t mtRangefinderToUse);
