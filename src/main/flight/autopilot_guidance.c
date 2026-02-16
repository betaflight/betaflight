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

#include "platform.h"

#if ENABLE_FLIGHT_PLAN

#include "autopilot_guidance.h"
#include "common/maths.h"
#include "common/vector.h"
#include "pg/autopilot.h"

#include <math.h>

#define L1_MIN_PATH_LENGTH_CM 10.0f

// L1 guidance state
static l1Guidance_t l1 = {
    .isActive = false,
    .pathLength = 0.0f,
    .crossTrackError = 0.0f,
    .alongTrackDistance = 0.0f,
    .lookaheadDistance = 0.0f
};

// Initialize L1 guidance system
void l1GuidanceInit(void)
{
    l1GuidanceReset();
}

// Reset L1 guidance state
void l1GuidanceReset(void)
{
    l1.isActive = false;
    l1.pathLength = 0.0f;
    l1.crossTrackError = 0.0f;
    l1.alongTrackDistance = 0.0f;
    l1.lookaheadDistance = 0.0f;
    vector2Zero(&l1.pathStart);
    vector2Zero(&l1.pathEnd);
    vector2Zero(&l1.pathDirection);
    vector2Zero(&l1.carrotPoint);
}

// Set the path segment for L1 to follow
void l1GuidanceSetPath(const vector2_t *start, const vector2_t *end)
{
    l1.pathStart = *start;
    l1.pathEnd = *end;

    // Calculate path vector
    vector2_t pathVector;
    vector2Sub(&pathVector, end, start);

    // Calculate path length
    l1.pathLength = vector2Norm(&pathVector);

    // Normalize path direction
    if (l1.pathLength > L1_MIN_PATH_LENGTH_CM) {
        vector2Normalize(&l1.pathDirection, &pathVector);
    } else {
        // Path too short, set invalid
        l1.pathLength = 0.0f;
        vector2Zero(&l1.pathDirection);
    }
}

// Calculate cross-track error (perpendicular distance from path)
// Positive = right of path, Negative = left of path
static float computeCrossTrackError(const vector2_t *position)
{
    // Vector from path start to current position
    vector2_t toPos;
    vector2Sub(&toPos, position, &l1.pathStart);

    // Cross product: pathDirection × toPos
    // In 2D: (dx, dy) × (px, py) = dx*py - dy*px
    return vector2Cross(&l1.pathDirection, &toPos);
}

// Calculate along-track distance (distance along path from start)
static float computeAlongTrackDistance(const vector2_t *position)
{
    // Vector from path start to current position
    vector2_t toPos;
    vector2Sub(&toPos, position, &l1.pathStart);

    // Dot product: pathDirection · toPos
    return vector2Dot(&l1.pathDirection, &toPos);
}

// Compute L1 lookahead distance based on groundspeed and period
static float computeLookaheadDistance(float groundSpeedCmS, float periodDeciseconds)
{
    const autopilotConfig_t *cfg = autopilotConfig();

    // L1 = V * T / (2 * π)
    // where T is the damping period in seconds
    float periodSeconds = periodDeciseconds * 0.1f;  // Deciseconds to seconds
    float lookahead = groundSpeedCmS * periodSeconds / (2.0f * M_PIf);

    // Constrain to configured limits
    lookahead = constrainf(lookahead, cfg->l1MinLookahead, cfg->l1MaxLookahead);

    return lookahead;
}

// Compute carrot point on the path
static void computeCarrotPoint(const vector2_t *position)
{
    // Calculate along-track distance
    float alongTrack = computeAlongTrackDistance(position);

    // Add lookahead distance to get carrot position
    float carrotAlongTrack = alongTrack + l1.lookaheadDistance;

    // Clamp to path endpoints
    carrotAlongTrack = constrainf(carrotAlongTrack, 0.0f, l1.pathLength);

    // Calculate carrot point on path
    vector2_t offset;
    vector2Scale(&offset, &l1.pathDirection, carrotAlongTrack);
    vector2Add(&l1.carrotPoint, &l1.pathStart, &offset);
}

// Update L1 guidance and compute carrot point
void l1GuidanceUpdate(const vector2_t *position, float groundSpeedCmS)
{
    if (!l1.isActive || l1.pathLength < L1_MIN_PATH_LENGTH_CM) {
        // L1 not active or path invalid
        return;
    }

    const autopilotConfig_t *cfg = autopilotConfig();

    // Calculate cross-track error
    l1.crossTrackError = computeCrossTrackError(position);

    // Calculate along-track distance
    l1.alongTrackDistance = computeAlongTrackDistance(position);

    // Safety check: if cross-track error is excessive, deactivate L1
    if (fabsf(l1.crossTrackError) > cfg->l1MaxCrossTrackError) {
        l1.isActive = false;
        return;
    }

    // Compute lookahead distance
    l1.lookaheadDistance = computeLookaheadDistance(groundSpeedCmS, cfg->l1Period);

    // Compute carrot point
    computeCarrotPoint(position);

    // Deactivate L1 when close to path endpoint
    // When within lookahead distance of endpoint, switch to direct targeting
    float distanceToEnd = l1.pathLength - l1.alongTrackDistance;
    if (distanceToEnd < l1.lookaheadDistance) {
        l1.isActive = false;
    }
}

// Get the carrot point (target position for position controller)
const vector2_t* l1GuidanceGetCarrot(void)
{
    return &l1.carrotPoint;
}

// Get cross-track error for debugging/telemetry
float l1GuidanceGetCrossTrackError(void)
{
    return l1.crossTrackError;
}

// Get along-track distance for debugging/telemetry
float l1GuidanceGetAlongTrackDistance(void)
{
    return l1.alongTrackDistance;
}

// Get lookahead distance for debugging/telemetry
float l1GuidanceGetLookaheadDistance(void)
{
    return l1.lookaheadDistance;
}

// Check if L1 guidance is active
bool l1GuidanceIsActive(void)
{
    return l1.isActive;
}

// Enable/disable L1 guidance
void l1GuidanceSetActive(bool active)
{
    l1.isActive = active;
}

#endif // ENABLE_FLIGHT_PLAN
