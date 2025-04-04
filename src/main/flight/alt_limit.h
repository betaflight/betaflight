/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "pg/alt_limit.h"
 
#ifdef USE_ALTITUDE_LIMIT
#include "common/time.h"
 
#define ALTLIMIT_TASK_RATE_HZ 20         // hz
 
void altLimitInit(void);
float getAltLimitedThrottle(float throttle);
bool isAltLimitActive(void);
void updateAltLimit(timeUs_t currentTimeUs);
bool altLimitWarn(void);
#endif
