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

// Minimum arc bearing change (degrees) to trigger arc transition
#define ARC_MIN_BEARING_CHANGE 20.0f
// Arc convergence threshold (degrees) — switch to normal L1 when within this
#define ARC_CONVERGENCE_DEG 5.0f

// L1 guidance state
static l1Guidance_t l1 = {
    .isActive = false,
    .pathLength = 0.0f,
    .crossTrackError = 0.0f,
    .alongTrackDistance = 0.0f,
    .lookaheadDistance = 0.0f,
    .arcActive = false,
    .arcStartValid = false
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
    l1.desiredBearing = 0.0f;
    l1.arcActive = false;
    l1.arcStartValid = false;
    vector2Zero(&l1.pathStart);
    vector2Zero(&l1.pathEnd);
    vector2Zero(&l1.pathDirection);
    vector2Zero(&l1.carrotPoint);
}

// Normalize angle to 0-360 range
static float normalizeAngle360(float deg)
{
    while (deg >= 360.0f) deg -= 360.0f;
    while (deg < 0.0f) deg += 360.0f;
    return deg;
}

// Compute shortest signed bearing change (result in -180..+180)
static float bearingDelta(float from, float to)
{
    float delta = to - from;
    if (delta > 180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;
    return delta;
}

// Set the path segment for L1 to follow.
// If l1GuidanceSetArcStart() was called beforehand and the bearing change
// exceeds ARC_MIN_BEARING_CHANGE, an arc transition is activated so the
// carrot rotates smoothly from the old heading to the new track bearing.
void l1GuidanceSetPath(const vector2_t *start, const vector2_t *end)
{
    l1.pathStart = *start;
    l1.pathEnd = *end;

    // Reset tracking state so stale values from the previous path
    // are never used before the first l1GuidanceUpdate() call.
    l1.alongTrackDistance = 0.0f;
    l1.crossTrackError = 0.0f;

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
        l1.arcActive = false;
        l1.arcStartValid = false;
        return;
    }

    // Check for arc transition
    const autopilotConfig_t *cfg = autopilotConfig();
    if (l1.arcStartValid && cfg->l1TurnRate > 0) {
        // New path bearing (NED: x=North, y=East)
        float newPathBearing = atan2f(l1.pathDirection.y, l1.pathDirection.x) * (180.0f / M_PIf);
        newPathBearing = normalizeAngle360(newPathBearing);

        float change = bearingDelta(l1.arcStartBearing, newPathBearing);

        if (fabsf(change) > ARC_MIN_BEARING_CHANGE) {
            l1.arcActive = true;
            l1.arcBearing = normalizeAngle360(l1.arcStartBearing);
            l1.arcDirection = (change > 0.0f) ? 1.0f : -1.0f;

            // Use the configured turn rate directly.  Scaling the rate
            // up for large bearing changes caused the arc to advance
            // faster than the craft could physically turn (especially
            // with the buildup angle limit active), resulting in huge
            // cross-track error as the carrot outran the aircraft.
            l1.arcRate = (float)cfg->l1TurnRate;
        } else {
            l1.arcActive = false;
        }
    } else {
        l1.arcActive = false;
    }
    l1.arcStartValid = false;
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

// Compute carrot point on the path with intercept angle limiting.
// When far off-track, push the carrot further ahead so the bearing from the
// current position to the carrot makes at most a 30° angle with the track.
// This prevents aggressive cross-track correction (roll oscillation) and
// creates a smooth intercept arc that naturally captures the track.
// Capture distance scales with groundspeed: as XTE shrinks below the normal
// L1 lookahead * tan(30°), the intercept lookahead falls below the normal
// lookahead and L1 tracking takes over seamlessly.
#define INTERCEPT_ANGLE_TAN 0.577f  // tan(30°): maximum intercept angle

static void computeCarrotPoint(const vector2_t *position)
{
    float alongTrack = computeAlongTrackDistance(position);

    // Intercept-angle-limited lookahead: ensures the bearing from the current
    // position to the carrot never exceeds 30° from the track bearing.
    // interceptLookahead = |XTE| / tan(30°)
    const float interceptLookahead = fabsf(l1.crossTrackError) / INTERCEPT_ANGLE_TAN;

    // Use the larger of normal L1 lookahead and intercept lookahead.
    // Far from track: intercept dominates → shallow 30° approach.
    // Near track: normal L1 dominates → standard path following.
    const float effectiveLookahead = fmaxf(l1.lookaheadDistance, interceptLookahead);

    float carrotAlongTrack = alongTrack + effectiveLookahead;

    // Clamp to path endpoints. Near the endpoint the carrot pins to the
    // waypoint, giving the position PID a proper convergence target.
    // L1 no longer self-deactivates, so there is no reactivation loop.
    carrotAlongTrack = constrainf(carrotAlongTrack, 0.0f, l1.pathLength);

    // Calculate carrot point on path
    vector2_t offset;
    vector2Scale(&offset, &l1.pathDirection, carrotAlongTrack);
    vector2Add(&l1.carrotPoint, &l1.pathStart, &offset);
}

// Compute the normal L1 carrot bearing (degrees, 0-360) for convergence checks.
// Assumes crossTrackError, alongTrackDistance, lookaheadDistance are already computed.
static float computeNormalL1Bearing(const vector2_t *position)
{
    computeCarrotPoint(position);

    vector2_t toCarrot;
    vector2Sub(&toCarrot, &l1.carrotPoint, position);
    float bearingDeg = atan2f(toCarrot.y, toCarrot.x) * (180.0f / M_PIf);
    return normalizeAngle360(bearingDeg);
}

// Update L1 guidance and compute carrot point
void l1GuidanceUpdate(const vector2_t *position, float groundSpeedCmS, float dt)
{
    if (!l1.isActive || l1.pathLength < L1_MIN_PATH_LENGTH_CM) {
        return;
    }

    const autopilotConfig_t *cfg = autopilotConfig();

    // Always compute standard L1 state for diagnostics and altitude interpolation
    l1.crossTrackError = computeCrossTrackError(position);
    l1.alongTrackDistance = computeAlongTrackDistance(position);
    l1.lookaheadDistance = computeLookaheadDistance(groundSpeedCmS, cfg->l1Period);

    if (l1.arcActive) {
        // Arc transition: rotate carrot bearing at the scaled turn rate
        l1.arcBearing += l1.arcDirection * l1.arcRate * dt;
        l1.arcBearing = normalizeAngle360(l1.arcBearing);

        // Compute where the normal L1 carrot would point (for convergence check)
        const float normalBearing = computeNormalL1Bearing(position);

        // Check convergence: arc bearing is within threshold of normal L1 bearing,
        // or has overshot (remaining delta is opposite to arc direction)
        const float remaining = bearingDelta(l1.arcBearing, normalBearing);
        if (fabsf(remaining) < ARC_CONVERGENCE_DEG || (remaining * l1.arcDirection < 0.0f)) {
            // Arc complete — fall through to normal L1
            l1.arcActive = false;
        } else {
            // Place carrot at position + lookahead in the arc bearing direction
            const float bearingRad = l1.arcBearing * (M_PIf / 180.0f);
            l1.carrotPoint.x = position->x + l1.lookaheadDistance * cos_approx(bearingRad);  // North
            l1.carrotPoint.y = position->y + l1.lookaheadDistance * sin_approx(bearingRad);  // East
            l1.desiredBearing = l1.arcBearing * 100.0f;  // degrees → centidegrees
            return;  // Skip endpoint deactivation during arc
        }
    }

    // Normal L1: compute carrot point (with intercept angle limiting)
    computeCarrotPoint(position);

    // Compute desired bearing from position to carrot (centidegrees)
    // NED frame: x=North, y=East → bearing = atan2(East, North)
    vector2_t toCarrot;
    vector2Sub(&toCarrot, &l1.carrotPoint, position);
    float bearingDeg = atan2f(toCarrot.y, toCarrot.x) * (180.0f / M_PIf);
    if (bearingDeg < 0.0f) {
        bearingDeg += 360.0f;
    }
    l1.desiredBearing = bearingDeg * 100.0f;  // degrees → centidegrees

    // No self-deactivation here. The carrot is clamped to the path endpoint
    // by computeCarrotPoint(), so near the endpoint the position PID naturally
    // targets the waypoint. The waypoint state machine handles the transition
    // to the next segment. Self-deactivation caused repeated short-lived
    // re-activation segments when the craft had cross-track error, preventing
    // convergence on the waypoint.
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

// Get path length for altitude interpolation
float l1GuidanceGetPathLength(void)
{
    return l1.pathLength;
}

// Get desired bearing from position to carrot (centidegrees, 0-35999)
float l1GuidanceGetDesiredBearing(void)
{
    return l1.desiredBearing;
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

// Set the arc start bearing for the next path transition.
// Must be called BEFORE l1GuidanceSetPath() to trigger an arc.
void l1GuidanceSetArcStart(float courseDeg)
{
    l1.arcStartBearing = normalizeAngle360(courseDeg);
    l1.arcStartValid = true;
}

// Check if arc transition is active
bool l1GuidanceIsArcActive(void)
{
    return l1.arcActive;
}

#endif // ENABLE_FLIGHT_PLAN
