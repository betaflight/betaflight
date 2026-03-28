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
    memset(&cmd, 0, sizeof(cmd));
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
    cmd.completed = false;
    cmd.completionSignalled = false;

    cmd.targetPosEfM = *targetPosEfM;
    cmd.includeAltitude = includeAltitude;
    cmd.cruiseSpeedMps = cruiseSpeedMps;
    cmd.acceptanceRadiusM = acceptanceRadiusM;
    cmd.completionSpeedMps = completionSpeedMps;

    cmd.callback = callback;
    cmd.callbackUserData = userData;

    vector3Zero(&previousTargetVelMps);
    vector3Zero(&currentTargetVelCmS);
    withinAcceptanceRadius = false;
    withinAcceptanceAltitude = false;
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
        vector3Zero(&currentTargetVelCmS);
        return;
    }

    const float posEastM  = est->position.x * 0.01f;
    const float posNorthM = est->position.y * 0.01f;
    const float posUpM    = est->position.z * 0.01f;

    vector3_t errorEfM = {{
        cmd.targetPosEfM.x - posEastM,
        cmd.targetPosEfM.y - posNorthM,
        cmd.includeAltitude ? (cmd.targetPosEfM.z - posUpM) : 0.0f
    }};

    const float horizDistM = sqrtf(sq(errorEfM.x) + sq(errorEfM.y));
    const float distanceM = cmd.includeAltitude ? vector3Norm(&errorEfM) : horizDistM;

    vector3_t dirEf;
    if (distanceM > MIN_DISTANCE_M) {
        vector3Scale(&dirEf, &errorEfM, 1.0f / distanceM);
    } else {
        vector3Zero(&dirEf);
    }

    float desiredSpeedMps = fminf(cmd.cruiseSpeedMps, POS_TO_VEL_KP * distanceM);

    if (cmd.maxDecelMps2 > 0.0f) {
        const float brakingSpeed = sqrtf(2.0f * cmd.maxDecelMps2 * distanceM);
        desiredSpeedMps = fminf(desiredSpeedMps, brakingSpeed);
    }

    vector3_t targetVelMps;
    vector3Scale(&targetVelMps, &dirEf, desiredSpeedMps);

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

    currentTargetVelCmS = (vector3_t){{
        targetVelMps.x * 100.0f,
        targetVelMps.y * 100.0f,
        targetVelMps.z * 100.0f
    }};

    const float horizSpeedMps = sqrtf(sq(est->velocity.x) + sq(est->velocity.y)) * 0.01f;
    const float absVzMps = fabsf(est->velocity.z * 0.01f);
    const float absErrZM = fabsf(cmd.targetPosEfM.z - posUpM);

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
            if (absErrZM <= cmd.acceptanceRadiusM) {
                withinAcceptanceAltitude = true;
            }
        } else {
            if (absErrZM > cmd.acceptanceRadiusM * HYSTERESIS_FACTOR) {
                withinAcceptanceAltitude = false;
            }
        }
    } else {
        withinAcceptanceAltitude = true;
    }

    const bool horizSpeedOk = (horizSpeedMps <= cmd.completionSpeedMps);
    const bool vertSpeedOk = !cmd.includeAltitude || (absVzMps <= cmd.completionSpeedMps);
    const bool reached = withinAcceptanceRadius && withinAcceptanceAltitude && horizSpeedOk && vertSpeedOk;

    if (reached && !cmd.completionSignalled) {
        cmd.completed = true;
        cmd.completionSignalled = true;

        vector3Zero(&currentTargetVelCmS);

        if (cmd.callback) {
            cmd.callback(cmd.callbackUserData);
        }

        if (cmd.autoClearOnReach) {
            positionNavClearTarget();
        }
    }
}

vector3_t positionNavGetTargetVelocityCmS(void)
{
    return currentTargetVelCmS;
}

#endif // !USE_WING
