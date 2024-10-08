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

#include "pg/pos_hold.h"

#ifdef USE_ALT_HOLD_MODE
#include "common/time.h"
#include "io/gps.h"

#define POSHOLD_TASK_RATE_HZ 100         // hz

typedef struct {
    bool isPosHoldActive;
    gpsLocation_t targetLocation;
    float targetAdjustRate;
} posHoldState_t;

void posHoldInit(void);
void updatePosHoldState(timeUs_t currentTimeUs);

#endif
