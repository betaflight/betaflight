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

#include "pg/flight_plan.h"

typedef enum {
    FP_NAV_IDLE = 0,
    FP_NAV_TARGETING,
    FP_NAV_HOLDING,
    FP_NAV_COMPLETE,
    FP_NAV_LANDING,
    FP_NAV_ABORTED,
} flightPlanNavState_e;

typedef enum {
    FP_ABORT_NONE = 0,
    FP_ABORT_ESTIMATOR,     // XY position estimate became invalid mid-mission
    FP_ABORT_STALLED,       // no progress toward the target within the stall window
    FP_ABORT_FLYAWAY,       // distance to target grew past the flyaway margin
    FP_ABORT_HEADING,       // rescue heading recovery did not converge in time
} flightPlanAbortReason_e;

void flightPlanNavInit(void);

// Called when AUTOPILOT mode transitions to active; resets to the first waypoint
// and dispatches it to positionNav. If the estimator has no GPS origin yet, the
// executor stays IDLE and flightPlanNavUpdate() retries dispatch.
void flightPlanNavEngage(void);

// Called when AUTOPILOT mode transitions to inactive; clears any pending target.
void flightPlanNavDisengage(void);

// Called periodically while AUTOPILOT is active; the caller determines the
// cadence (currently processRxModes()). currentTimeUs must be the current
// monotonic microsecond timestamp. Drives HOLD-state timeout and retries
// waypoint dispatch when a GPS origin becomes available.
void flightPlanNavUpdate(timeUs_t currentTimeUs);

bool flightPlanNavIsActive(void);
flightPlanNavState_e flightPlanNavGetState(void);
uint8_t flightPlanNavGetCurrentIndex(void);
flightPlanAbortReason_e flightPlanNavGetAbortReason(void);

// Replace the active mission with a small synthesised runtime plan (at most 4
// waypoints, copied). Only valid while the executor is active. The injected
// plan does not survive a switch cycle: engage and disengage revert to the
// stored mission, with no resume of what was preempted.
bool flightPlanNavInjectPlan(const waypoint_t *waypoints, uint8_t count);
bool flightPlanNavIsInjectedPlanActive(void);

#if ENABLE_RESCUE_PLAN
// Synthesise a failsafe rescue mission (climb-in-place, fly home, land) from
// the current position and home, staged for the next flightPlanNavEngage() to
// consume in place of the PG mission; injected immediately if the executor is
// already active. Returns false when no home or fix exists to build a plan —
// the failsafe caller then degrades to auto-landing.
bool flightPlanNavStageRescuePlan(void);
bool flightPlanNavIsRescuePlanActive(void);
#endif

// Single observer slot for "waypoint reached" — invoked with the index of the
// waypoint that was just reached, before any HOLD timer or advance. Pass NULL
// to detach. Used by telemetry/mavlink_mission to emit MISSION_ITEM_REACHED.
typedef void (*flightPlanWaypointReachedFn)(uint8_t index);
void flightPlanNavSetReachedListener(flightPlanWaypointReachedFn fn);
