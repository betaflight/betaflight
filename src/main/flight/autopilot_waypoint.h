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

#include <stdint.h>
#include <stdbool.h>

#include "common/time.h"
#include "io/gps.h"

// Waypoint state machine states
typedef enum {
    WP_STATE_IDLE = 0,          // Not active
    WP_STATE_APPROACHING,       // Flying toward waypoint
    WP_STATE_ARRIVED,           // Within arrival radius
    WP_STATE_HOLDING,           // Maintaining position at HOLD waypoint
    WP_STATE_ORBITING,          // Orbit pattern
    WP_STATE_FIGURE8,           // Figure-8 pattern
    WP_STATE_LANDING,           // Landing sequence
    WP_STATE_COMPLETE           // All waypoints complete
} waypointState_e;

// Waypoint tracking data structure
typedef struct {
    uint8_t currentIndex;           // Current waypoint index
    uint8_t nextIndex;              // Next waypoint index
    waypointState_e state;          // Current state
    uint32_t distanceToCurrent;     // Distance to current waypoint (cm)
    int32_t bearingToCurrent;       // Bearing to current waypoint (centidegrees, 0-35999)
    gpsLocation_t targetLocation;   // Current target location
    gpsLocation_t patternCenter;    // Center point for circle/figure-8 patterns
    timeUs_t holdStartTime;         // When hold pattern started
    timeUs_t arrivalTime;           // When arrived at waypoint
    timeUs_t landingStartTime;      // When landing sequence started
    float patternAngle;             // Current angle in pattern (degrees, 0-360)
    float landingStartAltitude;     // Altitude when landing started (cm)
    bool isValid;                   // Is waypoint system valid
} waypointTracker_t;

// Initialize waypoint system
void waypointInit(void);

// Reset waypoint tracker (call when entering autopilot mode)
void waypointReset(void);

// Main state machine update (call at 100Hz)
void waypointUpdate(timeUs_t currentTimeUs);

// Advance to next waypoint
void waypointAdvanceToNext(void);

// Validate waypoint at index
bool waypointIsValid(uint8_t idx);

// Get current target location (for position controller)
const gpsLocation_t* waypointGetTarget(void);

// Get current waypoint index
uint8_t waypointGetCurrentIndex(void);

// Get next waypoint index
uint8_t waypointGetNextIndex(void);

// Get distance to current waypoint (cm)
uint32_t waypointGetDistanceCm(void);

// Get bearing to current waypoint (centidegrees)
int32_t waypointGetBearingCdeg(void);

// Get current state
waypointState_e waypointGetState(void);

// Check if system is valid
bool waypointIsSystemValid(void);

// Get current waypoint type
uint8_t waypointGetCurrentType(void);

// Get time remaining in hold (deciseconds, 0 = infinite)
uint16_t waypointGetHoldTimeRemaining(void);

// Inject emergency landing at current GPS position (for RX loss)
void waypointSetEmergencyLanding(void);

// Inject return-to-home sequence (for geofence violation)
void waypointSetReturnToHome(void);
