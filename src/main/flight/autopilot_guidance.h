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

#include "common/vector.h"
#include <stdbool.h>

// L1 Nonlinear Guidance Structure
typedef struct l1Guidance_s {
    vector2_t pathStart;         // Start of path segment (NED 2D, cm)
    vector2_t pathEnd;           // End of path segment (NED 2D, cm)
    vector2_t pathDirection;     // Normalized path direction vector
    float pathLength;            // Length of path segment (cm)
    float crossTrackError;       // Perpendicular distance from path (cm, + = right)
    float alongTrackDistance;    // Distance along path from start (cm)
    float lookaheadDistance;     // L1 lookahead distance (cm)
    vector2_t carrotPoint;       // Target point on path (NED 2D, cm)
    float desiredBearing;        // Bearing from position to carrot (centidegrees, 0-35999)
    bool isActive;               // Whether L1 guidance is active

    // Arc transition state: smooth carrot rotation between track segments
    bool arcActive;              // Arc transition in progress
    float arcBearing;            // Current arc bearing (degrees, 0-360)
    float arcDirection;          // +1.0 = clockwise, -1.0 = counter-clockwise
    float arcRate;               // Effective turn rate for this arc (deg/s, scaled for large turns)
    float arcStartBearing;       // Pending arc start bearing (degrees, 0-360)
    bool arcStartValid;          // Whether arcStartBearing was set
} l1Guidance_t;

// Initialize L1 guidance system
void l1GuidanceInit(void);

// Reset L1 guidance state
void l1GuidanceReset(void);

// Set the path segment for L1 to follow
void l1GuidanceSetPath(const vector2_t *start, const vector2_t *end);

// Update L1 guidance and compute carrot point
void l1GuidanceUpdate(const vector2_t *position, float groundSpeedCmS, float dt);

// Get the carrot point (target position for position controller)
const vector2_t* l1GuidanceGetCarrot(void);

// Get cross-track error for debugging/telemetry
float l1GuidanceGetCrossTrackError(void);

// Get along-track distance for debugging/telemetry
float l1GuidanceGetAlongTrackDistance(void);

// Get lookahead distance for debugging/telemetry
float l1GuidanceGetLookaheadDistance(void);

// Get path length for altitude interpolation
float l1GuidanceGetPathLength(void);

// Get desired bearing from position to carrot (centidegrees, 0-35999)
float l1GuidanceGetDesiredBearing(void);

// Check if L1 guidance is active
bool l1GuidanceIsActive(void);

// Enable/disable L1 guidance
void l1GuidanceSetActive(bool active);

// Set the arc start bearing for the next path transition.
// Call before l1GuidanceSetPath() to enable arc transition from the current heading.
void l1GuidanceSetArcStart(float courseDeg);

// Check if arc transition is active
bool l1GuidanceIsArcActive(void);
