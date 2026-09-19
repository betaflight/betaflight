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

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "platform.h"

#include "common/maths.h"
#include "common/vector.h"

#include "flight/position_estimator.h"
#include "flight/position_nav.h"

#define POS_TO_VEL_KP          1.0f
#define MIN_DISTANCE_M         0.01f
#define HYSTERESIS_FACTOR      1.5f
// The commanded altitude may never lead the craft by more than this much flying time at the leg's
// vertical rate. A craft that cannot climb or descend as fast as it was asked to (ceiling, ground,
// battery) would otherwise accumulate altitude error for the altitude P term to answer with a
// throttle slam, which is what a stepped altitude target does today.
#define VERT_RAMP_LEASH_S      1.0f
#define VERT_RAMP_LEASH_MIN_M  1.0f
// The ramp brakes into the leg altitude at this rate rather than tapering on the whole remaining
// error: proportional tapering made every climb shorter than the leg rate in metres a one-second
// lag that never ran at the rate it was given, and left an exponential tail that never arrived.
#define VERT_RAMP_DECEL_MPS2   2.0f
// Vertical arrival window. The horizontal acceptance radius cannot serve: it is sized for the
// position estimate's lateral scatter and for how tightly a leg wants to be flown through, and a
// climb shorter than that radius would count as arrived before it began.
#define VERT_ACCEPTANCE_M      0.5f

static positionNavCommand_t cmd;
static vector3_t previousTargetVelMps;
static vector3_t currentTargetVelCmS;
static bool withinAcceptanceRadius;
static bool withinAcceptanceAltitude;

void positionNavInit(void)
{
    positionNavReset();
}

void positionNavReset(void)
{
    const uint32_t sequence = cmd.sequence;   // stays monotonic so consumers keep spotting the change
    memset(&cmd, 0, sizeof(cmd));
    cmd.sequence = sequence;
    vector3Zero(&previousTargetVelMps);
    vector3Zero(&currentTargetVelCmS);
    withinAcceptanceRadius = false;
    withinAcceptanceAltitude = false;
}

void positionNavSetTargetEf(
    const vector3_t *targetPosEfM,
    float cruiseSpeedMps,
    float acceptanceRadiusM,
    float completionSpeedMps,
    bool includeAltitude,
    positionNavReachedCallbackFn callback,
    void *userData
)
{
    cmd.active = true;
    cmd.sequence++;
    cmd.completed = false;
    cmd.completionSignalled = false;

    cmd.targetPosEfM = *targetPosEfM;
    cmd.includeAltitude = includeAltitude;
    cmd.cruiseSpeedMps = cruiseSpeedMps;
    cmd.vertRateMps = 0.0f;
    cmd.rampAltM = 0.0f;
    cmd.rampValid = false;
    cmd.approachSlowdownM = 0.0f;
    cmd.acceptanceRadiusM = acceptanceRadiusM;
    cmd.completionSpeedMps = completionSpeedMps;
    cmd.altitudeArrivalRequired = true;

    cmd.callback = callback;
    cmd.callbackUserData = userData;

    vector3Zero(&previousTargetVelMps);
    // The commanded velocity deliberately survives the handover: zeroing it here put a one-cycle
    // notch in the target at every leg change, which the position controller answers with a pitch
    // jerk. The next update recomputes it from the new target anyway.
    withinAcceptanceRadius = false;
    withinAcceptanceAltitude = false;
}

void positionNavMoveTargetEf(const vector3_t *targetPosEfM)
{
    if (!cmd.active) {
        return;
    }
    cmd.targetPosEfM = *targetPosEfM;
}

void positionNavClearTarget(void)
{
    cmd.active = false;
    cmd.completed = false;
    cmd.completionSignalled = false;
    vector3Zero(&currentTargetVelCmS);
    vector3Zero(&previousTargetVelMps);
    withinAcceptanceRadius = false;
    withinAcceptanceAltitude = false;
}

bool positionNavHasActiveTarget(void)
{
    return cmd.active;
}

bool positionNavTargetReached(void)
{
    return cmd.completed;
}

const positionNavCommand_t *positionNavGetActiveCommand(void)
{
    return &cmd;
}

// The leg's vertical rate: explicit when the leg states one, otherwise the horizontal cruise.
static float legVertRateMps(void)
{
    return (cmd.vertRateMps > 0.0f) ? cmd.vertRateMps : cmd.cruiseSpeedMps;
}

// Signed rate the altitude ramp moves at this cycle: the leg's rate, braked into the leg altitude
// so it settles instead of overshooting. Positive climbs.
static float verticalRampRateMps(void)
{
    const float errorM = cmd.targetPosEfM.v[ENU_U] - cmd.rampAltM;
    const float rateMps = fminf(legVertRateMps(), sqrtf(2.0f * VERT_RAMP_DECEL_MPS2 * fabsf(errorM)));
    return (errorM < 0.0f) ? -rateMps : rateMps;
}

void positionNavSetVerticalProfile(float rateMps, float startAltM)
{
    if (!cmd.active) {
        return;
    }
    cmd.vertRateMps = rateMps;
    cmd.rampAltM = startAltM;
    cmd.rampValid = true;
    // Seed the commanded rate now: the altitude controller's feedforward is consumed by a task that
    // can run before the next positionNavUpdate(), and a zero there against a moving altitude target
    // is a throttle notch at the start of every climb.
    currentTargetVelCmS.v[ENU_U] = cmd.includeAltitude ? verticalRampRateMps() * 100.0f : 0.0f;
}

void positionNavSetAccelLimits(float maxAccelMps2, float maxDecelMps2)
{
    cmd.maxAccelMps2 = maxAccelMps2;
    cmd.maxDecelMps2 = maxDecelMps2;
}

void positionNavSetCruiseSpeed(float cruiseSpeedMps)
{
    if (cmd.active) {
        cmd.cruiseSpeedMps = cruiseSpeedMps;
    }
}

void positionNavSetApproachSlowdown(float slowdownM)
{
    cmd.approachSlowdownM = slowdownM;
}

void positionNavSetAltitudeArrivalRequired(bool required)
{
    cmd.altitudeArrivalRequired = required;
}

void positionNavSetAutoClearOnReach(bool autoClear)
{
    cmd.autoClearOnReach = autoClear;
}

void positionNavUpdate(float dt, const positionEstimate3d_t *est)
{
    if (!cmd.active || cmd.completed) {
        vector3Zero(&currentTargetVelCmS);
        return;
    }

    const float posEastM  = est->position.v[ENU_E] * 0.01f;
    const float posNorthM = est->position.v[ENU_N] * 0.01f;
    const float posUpM    = est->position.v[ENU_U] * 0.01f;

    // Horizontal and vertical are separate channels. Sharing one 3D speed budget made the climb
    // rate a byproduct of how steep the leg happened to be, and left no way for a leg to state the
    // rate it wants to climb or descend at.
    const float errorEastM  = cmd.targetPosEfM.v[ENU_E] - posEastM;
    const float errorNorthM = cmd.targetPosEfM.v[ENU_N] - posNorthM;
    const float horizDistM = sqrtf(sq(errorEastM) + sq(errorNorthM));

    float desiredSpeedMps = fminf(cmd.cruiseSpeedMps, POS_TO_VEL_KP * horizDistM);

    // Bleed speed off from a stated range rather than waiting for the position gain to bite a few
    // metres out: the craft arrives slow instead of braking hard on the doorstep, and a hot arrival
    // has somewhere to shed its speed. Linear in distance, so the speed decays exponentially in
    // time - the shape the legacy rescue flew.
    if (cmd.approachSlowdownM > 0.0f) {
        desiredSpeedMps = fminf(desiredSpeedMps,
                                cmd.cruiseSpeedMps * (horizDistM / cmd.approachSlowdownM));
    }

    if (cmd.maxDecelMps2 > 0.0f) {
        const float brakingSpeed = sqrtf(2.0f * cmd.maxDecelMps2 * horizDistM);
        desiredSpeedMps = fminf(desiredSpeedMps, brakingSpeed);
    }

    vector3_t targetVelMps;
    vector3Zero(&targetVelMps);
    if (horizDistM > MIN_DISTANCE_M) {
        targetVelMps.v[ENU_E] = errorEastM / horizDistM * desiredSpeedMps;
        targetVelMps.v[ENU_N] = errorNorthM / horizDistM * desiredSpeedMps;
    }

    // Horizontal only: the vertical channel is rate-limited by its own ramp below.
    if (cmd.maxAccelMps2 > 0.0f && dt > 0.0f) {
        vector3_t delta;
        vector3Sub(&delta, &targetVelMps, &previousTargetVelMps);
        const float deltaMag = vector3Norm(&delta);
        const float maxDelta = cmd.maxAccelMps2 * dt;
        if (deltaMag > maxDelta && deltaMag > 0.0f) {
            vector3Scale(&delta, &delta, maxDelta / deltaMag);
        }
        vector3Add(&targetVelMps, &previousTargetVelMps, &delta);
    }

    previousTargetVelMps = targetVelMps;

    // March the commanded altitude toward the leg altitude at the leg's rate. The altitude
    // controller sees a ramp it can track and a feedforward that matches it, rather than a step.
    if (cmd.includeAltitude) {
        if (!cmd.rampValid) {
            cmd.rampAltM = posUpM;
            cmd.rampValid = true;
        }
        const float rampRateMps = verticalRampRateMps();
        if (dt > 0.0f) {
            const float targetAltM = cmd.targetPosEfM.v[ENU_U];
            cmd.rampAltM += rampRateMps * dt;
            if ((rampRateMps > 0.0f && cmd.rampAltM > targetAltM)
                || (rampRateMps < 0.0f && cmd.rampAltM < targetAltM)) {
                cmd.rampAltM = targetAltM;
            }
            const float leashM = fmaxf(VERT_RAMP_LEASH_MIN_M, legVertRateMps() * VERT_RAMP_LEASH_S);
            cmd.rampAltM = constrainf(cmd.rampAltM, posUpM - leashM, posUpM + leashM);
        }
        targetVelMps.v[ENU_U] = rampRateMps;
    }

    vector3Scale(&currentTargetVelCmS, &targetVelMps, 100.0f);  // m/s -> cm/s, ENU

    const float horizSpeedMps = sqrtf(sq(est->velocity.v[ENU_E]) + sq(est->velocity.v[ENU_N])) * 0.01f;
    const float absVzMps = fabsf(est->velocity.v[ENU_U] * 0.01f);
    const float absErrZM = fabsf(cmd.targetPosEfM.v[ENU_U] - posUpM);

    if (!withinAcceptanceRadius) {
        if (horizDistM <= cmd.acceptanceRadiusM) {
            withinAcceptanceRadius = true;
        }
    } else {
        if (horizDistM > cmd.acceptanceRadiusM * HYSTERESIS_FACTOR) {
            withinAcceptanceRadius = false;
        }
    }

    if (cmd.includeAltitude) {
        if (!withinAcceptanceAltitude) {
            if (absErrZM <= VERT_ACCEPTANCE_M) {
                withinAcceptanceAltitude = true;
            }
        } else {
            if (absErrZM > VERT_ACCEPTANCE_M * HYSTERESIS_FACTOR) {
                withinAcceptanceAltitude = false;
            }
        }
    } else {
        withinAcceptanceAltitude = true;
    }

    const bool horizSpeedOk = (horizSpeedMps <= cmd.completionSpeedMps);
    const bool vertSpeedOk = !cmd.includeAltitude || (absVzMps <= cmd.completionSpeedMps);
    const bool altitudeOk = withinAcceptanceAltitude || !cmd.altitudeArrivalRequired;
    const bool reached = withinAcceptanceRadius && altitudeOk && horizSpeedOk && vertSpeedOk;

    if (reached && !cmd.completionSignalled) {
        const bool autoClearOnReach = cmd.autoClearOnReach;
        cmd.completed = true;
        cmd.completionSignalled = true;

        vector3Zero(&currentTargetVelCmS);

        if (cmd.callback) {
            cmd.callback(cmd.callbackUserData);
        }

        if (autoClearOnReach && cmd.completed && cmd.completionSignalled) {
            positionNavClearTarget();
        }
    }
}

vector3_t positionNavGetTargetVelocityCmS(void)
{
    return currentTargetVelCmS;
}

float positionNavGetTargetAltitudeCm(void)
{
    // A completed leg stops updating the ramp, so keep handing back the leg altitude rather than
    // the value the ramp happened to be holding: alt hold latches whatever this returns and would
    // otherwise hold a stale altitude for the rest of the flight.
    if (!cmd.includeAltitude || !cmd.rampValid || cmd.completed) {
        return cmd.targetPosEfM.v[ENU_U] * 100.0f;
    }
    return cmd.rampAltM * 100.0f;
}

float positionNavGetVerticalRateLimitCmS(void)
{
    return cmd.active ? legVertRateMps() * 100.0f : 0.0f;
}
