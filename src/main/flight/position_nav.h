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
#include "common/vector.h"
#include "flight/position_estimator.h"

typedef void (*positionNavReachedCallbackFn)(void *userData);

typedef struct positionNavCommand_s {
    bool active;
    bool completed;
    bool completionSignalled;

    vector3_t targetPosEfM;         // target position, metres, ENU (x=east, y=north, z=up)
    bool includeAltitude;           // when false, z is ignored for nav, arrival, and alt coupling

    float cruiseSpeedMps;           // maximum cruise speed (m/s)
    float acceptanceRadiusM;        // arrival zone radius (metres)
    float completionSpeedMps;       // max ground speed to count as arrived (m/s)

    float maxAccelMps2;             // acceleration limit (m/s^2), 0 = unlimited
    float maxDecelMps2;             // deceleration/braking limit (m/s^2), 0 = unlimited

    bool autoClearOnReach;

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

void positionNavClearTarget(void);

bool positionNavHasActiveTarget(void);
bool positionNavTargetReached(void);

const positionNavCommand_t *positionNavGetActiveCommand(void);

void positionNavSetAccelLimits(float maxAccelMps2, float maxDecelMps2);
void positionNavSetAutoClearOnReach(bool autoClear);

// Called each control cycle; reads current estimate, computes target velocity,
// and checks arrival conditions.
void positionNavUpdate(float dt, const positionEstimate3d_t *est);

// Returns the target velocity computed by the most recent update (cm/s, ENU).
vector3_t positionNavGetTargetVelocityCmS(void);
