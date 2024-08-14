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

#include "pg/alt_hold.h"

#ifdef USE_ALTHOLD_MODE
#include "common/time.h"

#define ALTHOLD_TASK_RATE_HZ 100         // hz
#define ALTHOLD_ENTER_PERIOD 50         // ms
#define ALTHOLD_MAX_ENTER_PERIOD 3000   // ms

typedef struct {
    bool isAltHoldActive;
    float targetAltitudeCm;
    float targetAltitudeDelta;
    float measuredAltitudeCm;
    float throttleOut;
} altHoldState_t;

void altHoldInit(void);
void updateAltHoldState(timeUs_t currentTimeUs);
float altHoldGetThrottle(void);
bool altHoldIsActive(void);

#endif
