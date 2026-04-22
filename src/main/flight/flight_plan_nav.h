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

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"

typedef enum {
    FP_NAV_IDLE = 0,
    FP_NAV_TARGETING,
    FP_NAV_HOLDING,
    FP_NAV_COMPLETE,
} flightPlanNavState_e;

void flightPlanNavInit(void);

// Called when AUTOPILOT mode transitions to active; resets to the first waypoint
// and feeds it to positionNav. Silently no-ops if the plan is empty or the
// position estimator has no GPS origin.
void flightPlanNavEngage(void);

// Called when AUTOPILOT mode transitions to inactive; clears any pending target.
void flightPlanNavDisengage(void);

// Should be called at the position-control rate. Currently only drives HOLD
// timing; waypoint advance is callback-driven from positionNav.
void flightPlanNavUpdate(timeUs_t currentTimeUs);

bool flightPlanNavIsActive(void);
flightPlanNavState_e flightPlanNavGetState(void);
uint8_t flightPlanNavGetCurrentIndex(void);
