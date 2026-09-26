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
static bool previousTargetVelValid;     // previousTargetVelMps is a velocity that was commanded
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
    previousTargetVelValid = false;
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
    const bool handOver = cmd.active;
    cmd.active = true;
    cmd.sequence++;
    cmd.completed = false;
    cmd.completionSignalled = false;

    cmd.targetPosEfM = *targetPosEfM;
    cmd.fixedTarget = true;
    cmd.includeAltitude = includeAltitude;
    cmd.cruiseSpeedMps = cruiseSpeedMps;
    cmd.vertRateMps = 0.0f;
    cmd.rampAltM = 0.0f;
    cmd.rampValid = false;
    cmd.rampRateMps = currentTargetVelCmS.v[ENU_U] * 0.01f;
    cmd.rampRateSlewed = handOver;
    cmd.approachSlowdownM = 0.0f;
    cmd.approachStillRadiusM = 0.0f;
    cmd.approachStill = false;
    cmd.velocityFfValid = false;
    cmd.velocityFromCraft = false;
    cmd.acceptanceRadiusM = acceptanceRadiusM;
    cmd.completionSpeedMps = completionSpeedMps;
    cmd.settleTimeoutS = 0.0f;
    cmd.settleS = 0.0f;
    cmd.maxAngleDeg = 0.0f;
    cmd.altitudeArrivalRequired = true;

    cmd.callback = callback;
    cmd.callbackUserData = userData;

    // The commanded velocity deliberately survives the handover: zeroing it here put a one-cycle
    // notch in the target at every leg change, which the position controller answers with a pitch
    // jerk. The next update recomputes it from the new target, ramping out of it where the leg is
    // acceleration limited.
    previousTargetVelMps.v[ENU_E] = currentTargetVelCmS.v[ENU_E] * 0.01f;
    previousTargetVelMps.v[ENU_N] = currentTargetVelCmS.v[ENU_N] * 0.01f;
    previousTargetVelMps.v[ENU_U] = 0.0f;
    previousTargetVelValid = handOver;
    withinAcceptanceRadius = false;
    withinAcceptanceAltitude = false;
}

void positionNavMoveTargetEf(const vector3_t *targetPosEfM)
{
    if (!cmd.active) {
        return;
    }
    cmd.targetPosEfM = *targetPosEfM;
    cmd.fixedTarget = false;
}

void positionNavLowerTargetAltitude(float upM)
{
    if (cmd.active && upM < cmd.targetPosEfM.v[ENU_U]) {
        cmd.targetPosEfM.v[ENU_U] = upM;
    }
}

void positionNavClearTarget(void)
{
    cmd.active = false;
    cmd.completed = false;
    cmd.completionSignalled = false;
    vector3Zero(&currentTargetVelCmS);
    vector3Zero(&previousTargetVelMps);
    previousTargetVelValid = false;
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

// A ramp taking over from a leg still moving the altitude target slews out of that leg's rate into
// its own at the same deceleration, or harder where that would carry it through the leg altitude.
static float slewedVerticalRampRateMps(float dt)
{
    const float rateMps = verticalRampRateMps();
    if (!cmd.rampRateSlewed) {
        return rateMps;
    }
    const float errorM = cmd.targetPosEfM.v[ENU_U] - cmd.rampAltM;
    float decelMps2 = VERT_RAMP_DECEL_MPS2;
    if (cmd.rampRateMps * errorM > 0.0f) {
        decelMps2 = fmaxf(decelMps2, sq(cmd.rampRateMps) / (2.0f * fabsf(errorM)));
    }
    const float stepMps = decelMps2 * dt;
    return constrainf(rateMps, cmd.rampRateMps - stepMps, cmd.rampRateMps + stepMps);
}

void positionNavSetVerticalProfile(float rateMps, float startAltM)
{
    if (!cmd.active) {
        return;
    }
    cmd.vertRateMps = rateMps;
    cmd.rampAltM = startAltM;
    cmd.rampValid = true;
    if (!cmd.rampRateSlewed) {
        cmd.rampRateMps = verticalRampRateMps();
    }
    // Seed the commanded rate now: the altitude controller's feedforward is consumed by a task that
    // can run before the next positionNavUpdate(), and a zero there against a moving altitude target
    // is a throttle notch at the start of every climb.
    currentTargetVelCmS.v[ENU_U] = cmd.includeAltitude ? cmd.rampRateMps * 100.0f : 0.0f;
}

void positionNavSetAccelLimits(float maxAccelMps2, float maxDecelMps2)
{
    cmd.maxAccelMps2 = maxAccelMps2;
    cmd.maxDecelMps2 = maxDecelMps2;
}

void positionNavSetVelocityFeedforward(const vector2_t *velEfMps)
{
    if (!cmd.active) {
        return;
    }
    cmd.velocityFfValid = true;
    cmd.velocityFfEfMps = *velEfMps;
}

void positionNavStartAfresh(void)
{
    previousTargetVelValid = false;
}

void positionNavSetApproachSlowdown(float slowdownM, float stillRadiusM)
{
    cmd.approachSlowdownM = slowdownM;
    cmd.approachStillRadiusM = stillRadiusM;
    cmd.approachStill = false;
}

float positionNavApproachTaperMps(float cruiseSpeedMps, float slowdownM, float stillRadiusM, float distM)
{
    const float spanM = fmaxf(slowdownM - stillRadiusM, MIN_DISTANCE_M);
    return cruiseSpeedMps * constrainf((distM - stillRadiusM) / spanM, 0.0f, 1.0f);
}

void positionNavSetAltitudeArrivalRequired(bool required)
{
    cmd.altitudeArrivalRequired = required;
}

void positionNavSetSettleTimeout(float timeoutS)
{
    cmd.settleTimeoutS = timeoutS;
}

void positionNavSetMaxAngle(float angleDeg)
{
    cmd.maxAngleDeg = angleDeg;
}

void positionNavSetAutoClearOnReach(bool autoClear)
{
    cmd.autoClearOnReach = autoClear;
}

// March the commanded altitude toward the leg altitude at the leg's rate. The altitude controller
// sees a ramp it can track and a feedforward that matches it, rather than a step. Returns the rate.
static float updateVerticalRamp(float dt, float posUpM)
{
    if (!cmd.rampValid) {
        cmd.rampAltM = posUpM;
        cmd.rampValid = true;
    }
    cmd.rampRateMps = slewedVerticalRampRateMps(dt);
    if (dt > 0.0f) {
        const float targetAltM = cmd.targetPosEfM.v[ENU_U];
        const float rampBeforeM = cmd.rampAltM;
        const float errorBeforeM = targetAltM - rampBeforeM;
        cmd.rampAltM += cmd.rampRateMps * dt;
        // The side the ramp started on decides: one still slewing out of a rate pointing away
        // from the leg altitude has not passed it.
        if ((errorBeforeM >= 0.0f && cmd.rampAltM > targetAltM)
            || (errorBeforeM <= 0.0f && cmd.rampAltM < targetAltM)) {
            cmd.rampAltM = targetAltM;
            cmd.rampRateMps = errorBeforeM / dt;
        }
        const float leashM = fmaxf(VERT_RAMP_LEASH_MIN_M, legVertRateMps() * VERT_RAMP_LEASH_S);
        const float walkM = legVertRateMps() * dt;
        if (cmd.rampAltM > posUpM + leashM) {
            cmd.rampAltM = fmaxf(posUpM + leashM, fminf(cmd.rampAltM, rampBeforeM) - walkM);
        } else if (cmd.rampAltM < posUpM - leashM) {
            cmd.rampAltM = fminf(posUpM - leashM, fmaxf(cmd.rampAltM, rampBeforeM) + walkM);
        }
    }
    return cmd.rampRateMps;
}

void positionNavUpdate(float dt, const positionEstimate3d_t *est)
{
    if (!cmd.active) {
        vector3Zero(&currentTargetVelCmS);
        return;
    }

    const float posEastM  = est->position.v[ENU_E] * 0.01f;
    const float posNorthM = est->position.v[ENU_N] * 0.01f;
    const float posUpM    = est->position.v[ENU_U] * 0.01f;

    if (cmd.completed) {
        vector3Zero(&currentTargetVelCmS);
        if (cmd.includeAltitude) {
            currentTargetVelCmS.v[ENU_U] = updateVerticalRamp(dt, posUpM) * 100.0f;
        }
        return;
    }

    // A target flown at a stated velocity walks at it between its owner's moves: the owner may run
    // slower than this, and the position the craft is held to must not move in steps.
    if (cmd.velocityFfValid) {
        cmd.targetPosEfM.v[ENU_E] += cmd.velocityFfEfMps.x * dt;
        cmd.targetPosEfM.v[ENU_N] += cmd.velocityFfEfMps.y * dt;
    }

    // Horizontal and vertical are separate channels. Sharing one 3D speed budget made the climb
    // rate a byproduct of how steep the leg happened to be, and left no way for a leg to state the
    // rate it wants to climb or descend at.
    const float errorEastM  = cmd.targetPosEfM.v[ENU_E] - posEastM;
    const float errorNorthM = cmd.targetPosEfM.v[ENU_N] - posNorthM;
    const float horizDistM = sqrtf(sq(errorEastM) + sq(errorNorthM));

    vector3_t targetVelMps;
    vector3Zero(&targetVelMps);
    if (cmd.velocityFfValid) {
        targetVelMps.v[ENU_E] = cmd.velocityFfEfMps.x;
        targetVelMps.v[ENU_N] = cmd.velocityFfEfMps.y;
    } else {
        float desiredSpeedMps = fminf(cmd.cruiseSpeedMps, POS_TO_VEL_KP * horizDistM);

        // Bleed speed off from a stated range rather than waiting for the position gain to bite a few
        // metres out: the craft arrives slow instead of braking hard on the doorstep, and a hot arrival
        // has somewhere to shed its speed. Linear in distance, so the speed decays exponentially in
        // time - the shape the legacy rescue flew.
        if (cmd.approachSlowdownM > 0.0f) {
            if (horizDistM <= cmd.approachStillRadiusM) {
                cmd.approachStill = true;
            }
            desiredSpeedMps = cmd.approachStill ? 0.0f
                : positionNavApproachTaperMps(cmd.cruiseSpeedMps, cmd.approachSlowdownM, cmd.approachStillRadiusM, horizDistM);
        }

        if (cmd.maxDecelMps2 > 0.0f) {
            const float brakingSpeed = sqrtf(2.0f * cmd.maxDecelMps2 * horizDistM);
            desiredSpeedMps = fminf(desiredSpeedMps, brakingSpeed);
        }

        if (horizDistM > MIN_DISTANCE_M) {
            targetVelMps.v[ENU_E] = errorEastM / horizDistM * desiredSpeedMps;
            targetVelMps.v[ENU_N] = errorNorthM / horizDistM * desiredSpeedMps;
        }

        // Nothing commanded before this to ramp out of: start at the speed the craft is already
        // making toward the target, so a leg picked up at speed does not brake to rest first.
        if (!previousTargetVelValid) {
            vector3Zero(&previousTargetVelMps);
            const float wantMps = sqrtf(sq(targetVelMps.v[ENU_E]) + sq(targetVelMps.v[ENU_N]));
            if (wantMps > 0.0f) {
                const float towardMps = (est->velocity.v[ENU_E] * targetVelMps.v[ENU_E]
                                       + est->velocity.v[ENU_N] * targetVelMps.v[ENU_N]) * 0.01f / wantMps;
                vector3Scale(&previousTargetVelMps, &targetVelMps, constrainf(towardMps, 0.0f, wantMps) / wantMps);
            }
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
    }

    if (!previousTargetVelValid) {
        cmd.velocityFromCraft = cmd.velocityFfValid || cmd.maxAccelMps2 > 0.0f;
    }
    previousTargetVelMps = targetVelMps;
    previousTargetVelValid = true;

    if (cmd.includeAltitude) {
        targetVelMps.v[ENU_U] = updateVerticalRamp(dt, posUpM);
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
    const bool inPlace = withinAcceptanceRadius && altitudeOk;
    const bool slowEnough = horizSpeedOk && vertSpeedOk;
    cmd.settleS = (inPlace && !slowEnough) ? cmd.settleS + dt : 0.0f;
    const bool settleTimedOut = cmd.settleTimeoutS > 0.0f && cmd.settleS >= cmd.settleTimeoutS;
    const bool reached = inPlace && (slowEnough || settleTimedOut);

    if (reached && !cmd.completionSignalled) {
        const bool autoClearOnReach = cmd.autoClearOnReach;
        const uint32_t sequence = cmd.sequence;
        cmd.completed = true;
        cmd.completionSignalled = true;

        if (cmd.callback) {
            cmd.callback(cmd.callbackUserData);
        }

        // A callback that issued the next leg has handed over: this cycle's velocity stands until
        // the next update computes the new leg's.
        if (cmd.sequence == sequence) {
            currentTargetVelCmS.v[ENU_E] = 0.0f;
            currentTargetVelCmS.v[ENU_N] = 0.0f;
            if (autoClearOnReach) {
                positionNavClearTarget();
            }
        }
    }
}

vector3_t positionNavGetTargetVelocityCmS(void)
{
    return currentTargetVelCmS;
}

float positionNavGetTargetAltitudeCm(void)
{
    if (!cmd.includeAltitude || !cmd.rampValid) {
        return cmd.targetPosEfM.v[ENU_U] * 100.0f;
    }
    return cmd.rampAltM * 100.0f;
}

float positionNavGetVerticalRateLimitCmS(void)
{
    return cmd.active ? fmaxf(legVertRateMps(), fabsf(cmd.rampRateMps)) * 100.0f : 0.0f;
}
