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

#include "pg/pg.h"

// Yaw control modes
typedef enum {
    YAW_MODE_VELOCITY = 0,  // Multirotor: point nose in velocity direction
    YAW_MODE_BEARING = 1,   // Multirotor: point nose at waypoint
    YAW_MODE_HYBRID = 2,    // Multirotor: blend based on distance
    YAW_MODE_FIXED = 3,     // Multirotor: no yaw control
    YAW_MODE_DAMPENER = 4,  // Wing: yaw rate damper (coordinated turns)
    YAW_MODE_COUNT
} autopilotYawMode_e;

// RX Loss Policy: What to do when RC signal is lost during autopilot
typedef enum {
    AP_RX_LOSS_DISABLE = 0,     // Disable autopilot, use standard failsafe
    AP_RX_LOSS_CONTINUE = 1,    // Continue mission (if GPS valid)
    AP_RX_LOSS_LAND = 2,        // Land at current position
    AP_RX_LOSS_COUNT
} autopilotRxLossPolicy_e;

// Geofence action: What to do when craft exceeds max distance from home
typedef enum {
    AP_GEOFENCE_LAND = 0,       // Land at current position (safest default)
    AP_GEOFENCE_RTH = 1,        // Return to home then land
    AP_GEOFENCE_COUNT
} autopilotGeofenceAction_e;

typedef struct autopilotConfig_s {
    uint8_t landingAltitudeM;   // altitude below which landing behaviours can change, metres
    uint16_t hoverThrottle;      // value used at the start of a rescue or position hold
    uint16_t throttleMin;
    uint16_t throttleMax;
    uint8_t altitudeP;
    uint8_t altitudeI;
    uint8_t altitudeD;
    uint8_t altitudeF;
    uint8_t positionP;
    uint8_t positionI;
    uint8_t positionD;
    uint8_t positionA;
    uint8_t positionCutoff;
    uint8_t maxAngle;

    // Velocity-based position control with drag compensation
    uint8_t velocityControlEnable;    // 0=position→angle, 1=position→velocity→angle (default 1)
    uint8_t velocityP;                // velocity loop P gain, scaled by 10 (default 50 = 5.0)
    uint8_t velocityI;                // velocity loop I gain, scaled by 10 (default 10 = 1.0)
    uint8_t velocityD;                // velocity loop D gain, scaled by 10 (default 5 = 0.5)
    uint16_t velocityDragCoeff;       // drag coefficient, scaled by 10000 (default 50 = 0.0050)
    uint16_t maxVelocity;             // cm/s, maximum velocity setpoint (default 1000 = 10 m/s)

    // Waypoint navigation parameters
    uint16_t waypointArrivalRadius;   // cm, arrival detection radius (default 500)
    uint16_t waypointHoldRadius;      // cm, hold pattern radius (default 200)
    uint16_t stickDeadband;           // RC units (0-500), deadband for pilot override (default 50)
    uint16_t throttleDeadband;        // RC units (0-500), throttle override deadband (default 50)

    // Yaw control parameters
    uint8_t yawMode;                  // autopilotYawMode_e (default YAW_MODE_VELOCITY)
    uint16_t yawP;                    // scaled by 100 (e.g., 50 = 0.5, default 50)
    uint16_t yawD;                    // scaled by 100 (e.g., 10 = 0.1, default 10)
    uint16_t maxYawRate;              // deg/s, maximum yaw rate (default 90)
    uint16_t minForwardVelocity;      // cm/s, minimum forward velocity: GPS course reliability (multirotor) / stall prevention (wing)

    // Velocity buildup (acceleration from stationary)
    uint8_t velocityBuildupMaxPitch;  // degrees, max pitch bias for acceleration (default 8)

    // Turn rate and holding patterns
    uint8_t maxTurnRate;              // deg/s, maximum turn rate (default 3 for rate-1 turn, configurable 1-90)
    uint16_t holdOrbitRadius;         // cm, radius for orbit holding pattern (default 1000 = 10m)
    uint16_t holdFigure8Width;        // cm, width of figure-8 pattern (default 2000 = 20m)

    // Landing sequence
    uint8_t landingDescentRate;       // cm/s, descent rate during landing (default 50 = 0.5 m/s)
    uint8_t landingDetectionTime;     // deciseconds, time below threshold for touchdown detection (default 10 = 1s)
    uint8_t landingSpiralEnable;      // 0=straight descent, 1=spiral (avoid vortex ring state, default 1)
    uint16_t landingSpiralRadius;     // cm, radius of spiral pattern (default 200 = 2m)
    uint8_t landingSpiralRate;        // deg/s, rotation rate (default 10 = 10 deg/s)
    uint8_t landingVelocityThreshold; // cm/s, max velocity for touchdown (default 50 = 0.5 m/s)
    uint16_t landingThrottleThreshold; // RC units, max throttle deviation for touchdown (default 100)

    // L1 Nonlinear Guidance parameters
    uint8_t  l1Enable;                // 0=direct targeting, 1=L1 guidance (default 1)
    uint16_t l1Period;                // L1 damping period, deciseconds (default 20 = 2.0s)
    uint16_t l1MinLookahead;          // Minimum lookahead distance, cm (default 1000 = 10m)
    uint16_t l1MaxLookahead;          // Maximum lookahead distance, cm (default 10000 = 100m)
    uint16_t l1MaxCrossTrackError;    // Max cross-track error before fallback, cm (default 10000 = 100m)
    uint8_t l1TurnRate;              // deg/s, max turn rate for arc transitions between waypoints (default 8, 0=disabled)

    // Vertical track: climb to min altitude, then follow glide slope to waypoint
    uint8_t minNavAltitudeM;          // metres AGL, minimum altitude before horizontal navigation (default 5)

    // Safety: RX loss policy
    uint8_t rxLossPolicy;             // autopilotRxLossPolicy_e (default AP_RX_LOSS_DISABLE)

    // Safety: Geofence (max distance from home)
    uint16_t maxDistanceFromHomeM;    // meters, 0 = disabled (default 0)
    uint8_t geofenceAction;           // autopilotGeofenceAction_e (default AP_GEOFENCE_LAND)
} autopilotConfig_t;

PG_DECLARE(autopilotConfig_t, autopilotConfig);
