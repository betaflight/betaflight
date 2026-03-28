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

#ifndef USE_WING

#include "common/maths.h"
#include "common/vector.h"

#include "flight/position_estimator.h"
#include "flight/position_nav.h"

#define POS_TO_VEL_KP          1.0f
#define MIN_DISTANCE_M         0.01f
#define HYSTERESIS_FACTOR      1.5f

static positionNavCommand_t cmd;
static vector2_t previousTargetVelMps;
static vector2_t currentTargetVelCmS;
static bool withinAcceptanceRadius;

void positionNavInit(void)
{
    positionNavReset();
}

void positionNavReset(void)
{
    memset(&cmd, 0, sizeof(cmd));
    vector2Zero(&previousTargetVelMps);
    vector2Zero(&currentTargetVelCmS);
    withinAcceptanceRadius = false;
}

void positionNavSetTargetEf(
    const vector2_t *targetPosEfM,
    float cruiseSpeedMps,
    float acceptanceRadiusM,
    float completionSpeedMps,
    positionNavReachedCallbackFn callback,
    void *userData
)
{
    cmd.active = true;
    cmd.completed = false;
    cmd.completionSignalled = false;

    cmd.targetPosEfM = *targetPosEfM;
    cmd.cruiseSpeedMps = cruiseSpeedMps;
    cmd.acceptanceRadiusM = acceptanceRadiusM;
    cmd.completionSpeedMps = completionSpeedMps;

    cmd.callback = callback;
    cmd.callbackUserData = userData;

    vector2Zero(&previousTargetVelMps);
    vector2Zero(&currentTargetVelCmS);
    withinAcceptanceRadius = false;
}

void positionNavClearTarget(void)
{
    cmd.active = false;
    cmd.completed = false;
    cmd.completionSignalled = false;
    vector2Zero(&currentTargetVelCmS);
    vector2Zero(&previousTargetVelMps);
    withinAcceptanceRadius = false;
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

void positionNavSetAccelLimits(float maxAccelMps2, float maxDecelMps2)
{
    cmd.maxAccelMps2 = maxAccelMps2;
    cmd.maxDecelMps2 = maxDecelMps2;
}

void positionNavSetAutoClearOnReach(bool autoClear)
{
    cmd.autoClearOnReach = autoClear;
}

void positionNavUpdate(float dt, const positionEstimate3d_t *est)
{
    if (!cmd.active || cmd.completed) {
        vector2Zero(&currentTargetVelCmS);
        return;
    }

    // Current position converted from cm to metres
    const float posEastM  = est->position.x * 0.01f;
    const float posNorthM = est->position.y * 0.01f;

    // Error: target minus current (metres, ENU)
    const vector2_t errorEfM = {{
        cmd.targetPosEfM.x - posEastM,
        cmd.targetPosEfM.y - posNorthM
    }};

    const float distanceM = vector2Norm(&errorEfM);

    // Direction unit vector (safe against near-zero distance)
    vector2_t dirEf;
    if (distanceM > MIN_DISTANCE_M) {
        vector2Scale(&dirEf, &errorEfM, 1.0f / distanceM);
    } else {
        vector2Zero(&dirEf);
    }

    // Speed profile: proportional ramp capped at cruise speed
    float desiredSpeedMps = fminf(cmd.cruiseSpeedMps, POS_TO_VEL_KP * distanceM);

    // Braking curve: v = sqrt(2 * decel * distance)
    if (cmd.maxDecelMps2 > 0.0f) {
        const float brakingSpeed = sqrtf(2.0f * cmd.maxDecelMps2 * distanceM);
        desiredSpeedMps = fminf(desiredSpeedMps, brakingSpeed);
    }

    // Target velocity = direction * desired speed
    vector2_t targetVelMps;
    vector2Scale(&targetVelMps, &dirEf, desiredSpeedMps);

    // Acceleration limiting (applied to the change in target velocity)
    if (cmd.maxAccelMps2 > 0.0f && dt > 0.0f) {
        vector2_t delta;
        vector2Sub(&delta, &targetVelMps, &previousTargetVelMps);
        const float deltaMag = vector2Norm(&delta);
        const float maxDelta = cmd.maxAccelMps2 * dt;
        if (deltaMag > maxDelta && deltaMag > 0.0f) {
            vector2Scale(&delta, &delta, maxDelta / deltaMag);
        }
        vector2Add(&targetVelMps, &previousTargetVelMps, &delta);
    }

    previousTargetVelMps = targetVelMps;

    // Convert m/s to cm/s for autopilot consumption
    currentTargetVelCmS = (vector2_t){{
        targetVelMps.x * 100.0f,
        targetVelMps.y * 100.0f
    }};

    // --- Arrival detection with hysteresis ---
    const float currentSpeedMps = sqrtf(sq(est->velocity.x) + sq(est->velocity.y)) * 0.01f;

    if (!withinAcceptanceRadius) {
        if (distanceM <= cmd.acceptanceRadiusM) {
            withinAcceptanceRadius = true;
        }
    } else {
        if (distanceM > cmd.acceptanceRadiusM * HYSTERESIS_FACTOR) {
            withinAcceptanceRadius = false;
        }
    }

    const bool reached = withinAcceptanceRadius &&
                         (currentSpeedMps <= cmd.completionSpeedMps);

    if (reached && !cmd.completionSignalled) {
        cmd.completed = true;
        cmd.completionSignalled = true;

        vector2Zero(&currentTargetVelCmS);

        if (cmd.callback) {
            cmd.callback(cmd.callbackUserData);
        }

        if (cmd.autoClearOnReach) {
            positionNavClearTarget();
        }
    }
}

vector2_t positionNavGetTargetVelocityCmS(void)
{
    return currentTargetVelCmS;
}

#endif // !USE_WING
