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
#include "common/vector.h"

#include "flight/position_estimator.h"

// Flies the legs of a flight plan on a fixed wing. The flight plan executor sequences the plan and
// keeps its clocks; this decides how each leg is flown and when it has been reached. A wing cannot
// stop, so it flies lines between waypoints and loiters wherever a multirotor would wait.

typedef struct {
    uint8_t type;           // waypointType_e
    vector3_t targetEnuM;   // estimator frame, metres
    bool hasNext;           // a positional waypoint follows
    uint8_t nextType;
    vector2_t nextEnuM;
    float vertRateMps;      // the leg's climb and descent rate
    float startAltM;        // the altitude already commanded, where the leg's altitude ramp starts
    bool climbOut;          // a HOLD flown from where the aircraft is, done at its height once heading
                            // for the next waypoint
    bool injectedLanding;   // a LAND on the way home or where the aircraft is, rather than one the
                            // mission flies to: on the arming point's ground if that is where it is,
                            // on the wing's own landing heading
} fpWingLeg_t;

typedef enum {
    FPW_FLYING,
    FPW_REACHED,
} fpWingEvent_e;

// A new plan, or none: the next leg starts its line where the aircraft is.
void flightPlanWingReset(void);
void flightPlanWingDispatchLeg(const fpWingLeg_t *leg);
// progressM is the distance left to fly, for the executor's stall and flyaway checks.
fpWingEvent_e flightPlanWingUpdateLeg(const positionEstimate3d_t *est, float *progressM);
// The leg was reached and the plan waits there: loiter about the waypoint, or where a takeoff's
// climb ended.
void flightPlanWingHold(void);
// The plan is over: loiter about the last waypoint reached, or where the aircraft is.
void flightPlanWingFinish(void);
// The LAND waypoint was reached and the plan lands there. A mission's LAND waypoint is at the
// ground's elevation, and its approach is flown on the course the mission flew in on. An injected
// one is on home's ground, and away from home it comes straight down.
void flightPlanWingStartLanding(timeUs_t nowUs);
// How far past its best a leg's distance may grow before it counts as a flyaway: turning back from
// the speed being carried takes room.
float flightPlanWingOvershootM(const positionEstimate3d_t *est);
uint16_t flightPlanWingOrbitPeriodDs(void);
// The altitude the active leg commands, or the aircraft's own without one.
float flightPlanWingCommandedAltitudeM(void);

void flightPlanWingLoiterAt(const vector2_t *centreEnuM, float altM, float vertRateMps, float startAltM);
void flightPlanWingFlyLine(const vector2_t *startEnuM, const vector2_t *endEnuM, float altM, float vertRateMps, float startAltM);
