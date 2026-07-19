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

#include <string.h>

#include "platform.h"

#if ENABLE_FLIGHT_PLAN && !defined(USE_WING)

#include "common/printf.h"
#include "common/time.h"

#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "flight/flight_plan_nav.h"

#include "io/gps.h"

#include "pg/flight_plan.h"

#include "flight_plan_capture.h"

// The gesture commits on RELEASE so a hold can mean delete; the position is
// sampled at the PRESS, so the moment the pilot marks is the point that is
// stored, not wherever the craft has drifted to by the release.
#define FP_CAPTURE_HOLD_DELETE_US 1500000u
#define FP_CAPTURE_MSG_TIME_US    2500000u

static bool switchWas = false;
static timeUs_t pressTimeUs = 0;
static bool deleteFired = false;      // this press already consumed as a delete
static waypoint_t pending;            // sampled at press, committed at release
static bool pendingValid = false;
static bool pressAccepted = false;    // press happened somewhere edits are allowed

static char msgText[16];
static timeUs_t msgExpireUs = 0;
static bool msgValid = false;

static void setMessage(timeUs_t currentTimeUs, const char *fmt, int index)
{
    tfp_sprintf(msgText, fmt, index);
    msgExpireUs = currentTimeUs + FP_CAPTURE_MSG_TIME_US;
    msgValid = true;
}

// Edits are refused while the executor owns the plan: appending behind a
// flying mission or deleting the leg being flown would corrupt its index.
static bool editsAllowed(void)
{
    return !flightPlanNavIsActive();
}

void flightPlanCaptureUpdate(timeUs_t currentTimeUs, bool switchActive, bool channelsValid)
{
    // signal loss: freeze the gesture rather than read the dropout as a
    // release. folding validity into switchActive would make a hold that was
    // heading for a delete look like a release and commit the pending waypoint
    // - the exact ghost-edit this guard exists to stop. the press position and
    // the hold clock are both held, so the gesture just resumes when the link
    // is back (and rxfail driving the box high mid-loss still can't touch it).
    if (!channelsValid) {
        return;
    }

    flightPlanConfig_t *plan = flightPlanConfigMutable();

    if (switchActive && !switchWas) {
        // press: sample the position now. Capacity is deliberately not checked
        // here - a hold-to-delete must work on a full plan - only at commit.
        deleteFired = false;
        pendingValid = false;
        pressAccepted = editsAllowed();
        pressTimeUs = currentTimeUs;
        if (pressAccepted && STATE(GPS_FIX)) {
            pending = (waypoint_t){
                .latitude = gpsSol.llh.lat,
                .longitude = gpsSol.llh.lon,
                .altitude = gpsSol.llh.altCm,
                .speed = 0,               // leg default: autopilot cruise
                .duration = 0,
                .type = WAYPOINT_TYPE_FLYBY,
                .pattern = WAYPOINT_PATTERN_NONE,
            };
            pendingValid = true;
        }
    } else if (switchActive && switchWas && !deleteFired
               && cmpTimeUs(currentTimeUs, pressTimeUs) >= (timeDelta_t)FP_CAPTURE_HOLD_DELETE_US) {
        // held: delete the last waypoint, once per press. The release that
        // follows must not also drop the sampled position.
        deleteFired = true;
        pendingValid = false;
        if (pressAccepted && editsAllowed() && plan->waypointCount > 0) {
            plan->waypointCount--;
            setMessage(currentTimeUs, "WP%d DELETED", plan->waypointCount + 1);
        }
    } else if (!switchActive && switchWas && !deleteFired && pressAccepted) {
        // release before the hold threshold: commit the sampled waypoint
        if (pendingValid && editsAllowed()) {
            if (plan->waypointCount < MAX_WAYPOINTS) {
                plan->waypoints[plan->waypointCount++] = pending;
                setMessage(currentTimeUs, "WP%d SET", plan->waypointCount);
            } else {
                setMessage(currentTimeUs, "WP FULL", 0);
            }
        }
        pendingValid = false;
    }
    switchWas = switchActive;
}

const char *flightPlanCaptureOsdMessage(void)
{
    if (!msgValid) {
        return NULL;
    }
    if (cmpTimeUs(micros(), msgExpireUs) >= 0) {
        msgValid = false;
        return NULL;
    }
    return msgText;
}

#ifdef UNIT_TEST
void flightPlanCaptureResetForTest(void)
{
    switchWas = false;
    pressTimeUs = 0;
    deleteFired = false;
    pendingValid = false;
    pressAccepted = false;
    msgValid = false;
    msgExpireUs = 0;
}
#endif

#endif // ENABLE_FLIGHT_PLAN && !USE_WING
