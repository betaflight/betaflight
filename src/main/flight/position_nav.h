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

#include "common/vector.h"
#include "flight/position_estimator.h"

typedef void (*positionNavReachedCallbackFn)(void *userData);

typedef struct positionNavCommand_s {
    bool active;
    uint32_t sequence;              // bumped on every new target, so consumers can spot a leg change
    bool completed;
    bool completionSignalled;

    vector3_t targetPosEfM;         // target position, metres, ENU (index by ENU_E/ENU_N/ENU_U)
    bool includeAltitude;           // when false, ENU_U is ignored for nav, arrival, and alt coupling

    float cruiseSpeedMps;           // maximum horizontal cruise speed (m/s)
    float vertRateMps;              // maximum climb/descent rate (m/s), 0 = use cruiseSpeedMps
    float acceptanceRadiusM;        // arrival zone radius (metres)
    float completionSpeedMps;       // max ground speed to count as arrived (m/s)

    float rampAltM;                 // altitude the vertical channel is currently commanding (ENU_U, metres)
    bool rampValid;                 // rampAltM has been seeded for this command
    float rampRateMps;              // rate the ramp moved at on the last update (signed, positive climbs)
    bool rampRateSlewed;            // took over from a command still flying: slews out of its rate

    float approachSlowdownM;        // taper the commanded speed linearly inside this range, 0 = off
    float approachStillRadiusM;     // ...down to nothing at this range
    bool approachStill;             // has been inside the still radius: stopped chasing the target
    bool velocityFfValid;           // the owner states the horizontal velocity: no chase law toward the target
    vector2_t velocityFfEfMps;      // that velocity, metres/s (x east, y north)
    bool velocityFromCraft;         // its velocity started from the craft's own motion, not a predecessor's command
    float maxAccelMps2;             // acceleration limit (m/s^2), 0 = unlimited
    float maxDecelMps2;             // deceleration/braking limit (m/s^2), 0 = unlimited

    bool autoClearOnReach;
    bool altitudeArrivalRequired;   // when false, arrival gates on the horizontal radius only

    positionNavReachedCallbackFn callback;
    void *callbackUserData;
} positionNavCommand_t;

void positionNavInit(void);
void positionNavReset(void);

// includeAltitude: when true, z is used for 3D path, vertical arrival, target velocity Z, and
// altitude hold coupling (see alt_hold). When false, only horizontal plane is used (legacy 2D nav).
void positionNavSetTargetEf(
    const vector3_t *targetPosEfM,
    float cruiseSpeedMps,
    float acceptanceRadiusM,
    float completionSpeedMps,
    bool includeAltitude,
    positionNavReachedCallbackFn callback,
    void *userData
);

// Moves the active command's target position without disturbing the velocity
// ramp, completion state, or callback — for continuously moving targets (hold
// pattern carrots). No-op when there is no active command.
void positionNavMoveTargetEf(const vector3_t *targetPosEfM);

void positionNavClearTarget(void);

bool positionNavHasActiveTarget(void);
bool positionNavTargetReached(void);

const positionNavCommand_t *positionNavGetActiveCommand(void);

void positionNavSetAccelLimits(float maxAccelMps2, float maxDecelMps2);

// The horizontal velocity to command, for a target its owner flies along a profile of its own (a
// carrot on a leg line). It replaces the chase law toward the target and is commanded as it stands,
// the owner shaping its acceleration; the target walks at it between the owner's moves. Cleared by
// the next positionNavSetTargetEf(); no-op without an active command.
void positionNavSetVelocityFeedforward(const vector2_t *velEfMps);

// The active command starts as one with nothing before it rather than carrying on the velocity the
// command it replaced was commanding: for a re-target that does not continue the leg being flown.
// The altitude ramp carries on as it is.
void positionNavStartAfresh(void);

// Taper the commanded speed linearly from the cruise at slowdownM from the target to nothing at
// stillRadiusM, the way the legacy rescue bled speed from twice the descent distance; the taper
// replaces the position gain's knee. Once inside the still radius the command stops chasing the
// target for the rest of the leg, and nothing horizontal is commanded. Zero slowdownM leaves the
// leg on its own profile.
void positionNavSetApproachSlowdown(float slowdownM, float stillRadiusM);
// The speed that taper allows distM from the target, for an owner flying the approach ahead of the
// command that tapers it.
float positionNavApproachTaperMps(float cruiseSpeedMps, float slowdownM, float stillRadiusM, float distM);
void positionNavSetAutoClearOnReach(bool autoClear);

// The leg's vertical intent: the rate the altitude target is allowed to move at, and the altitude
// it starts from - the altitude the command it takes over from was commanding, or the craft's when
// there was none, so the altitude target does not step. A command taking over from one still flying
// starts at that one's rate and slews into its own; from rest it starts at its own. Seeds the
// commanded vertical velocity immediately, so a consumer running before the next
// positionNavUpdate() already sees the rate this leg is climbing or descending at.
// rateMps <= 0 falls back to the horizontal cruise speed. No-op without an active command.
void positionNavSetVerticalProfile(float rateMps, float startAltM);

// En-route waypoints advance on horizontal arrival even when the vehicle has
// not reached the commanded altitude; station-keeping targets (hold, land)
// keep the altitude gate. Defaults to true on each new target.
void positionNavSetAltitudeArrivalRequired(bool required);

// Called each control cycle; reads current estimate, computes target velocity,
// and checks arrival conditions.
void positionNavUpdate(float dt, const positionEstimate3d_t *est);

// Returns the target velocity computed by the most recent update (cm/s, ENU).
vector3_t positionNavGetTargetVelocityCmS(void);

// The altitude the vertical channel is commanding right now (cm, estimator frame): the ramp
// walking toward the leg altitude, not the leg altitude itself, and still walking it once the leg
// has completed. Meaningless (returns the leg altitude) when the command does not include altitude.
float positionNavGetTargetAltitudeCm(void);

// The leg's climb/descent rate cap (cm/s), or 0 when no command is active.
float positionNavGetVerticalRateLimitCmS(void);
