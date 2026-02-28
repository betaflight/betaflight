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

#include "flight/autopilot.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "autopilot.h"

PG_REGISTER_WITH_RESET_TEMPLATE(autopilotConfig_t, autopilotConfig, PG_AUTOPILOT, 6);

PG_RESET_TEMPLATE(autopilotConfig_t, autopilotConfig,
    .landingAltitudeM = 4,
    .hoverThrottle = 1275,
    .throttleMin = 1100,
    .throttleMax = 1900,
    .altitudeP = 15,
    .altitudeI = 15,
    .altitudeD = 15,
    .altitudeF = 15,
    .positionP = 30,
    .positionI = 30,
    .positionD = 30,
    .positionA = 30,
    .positionCutoff = 80,
    .maxAngle = 50,

    // Velocity-based position control with drag compensation
    .velocityControlEnable = 1,       // Enabled by default
    .velocityP = 50,                  // 5.0 P gain
    .velocityI = 10,                  // 1.0 I gain
    .velocityD = 5,                   // 0.5 D gain
    .velocityDragCoeff = 50,          // 0.0050 drag coefficient
    .maxVelocity = 1000,              // 10 m/s max velocity

    // Waypoint navigation parameters
    .waypointArrivalRadius = 500,     // 5m for FLYOVER/FLYBY
    .waypointHoldRadius = 200,        // 2m for HOLD/LAND
    .stickDeadband = 50,              // RC units
    .throttleDeadband = 50,           // RC units

    // Yaw control parameters
    .yawMode = YAW_MODE_VELOCITY,     // Default: follow velocity
    .yawP = 50,                       // 0.5 P gain
    .yawD = 10,                       // 0.1 D gain
    .maxYawRate = 30,                 // 30 deg/s max
    .minForwardVelocity = 300,        // 3.0 m/s minimum forward velocity (GPS course reliability / stall prevention)

    // Velocity buildup (acceleration from stationary)
    .velocityBuildupMaxPitch = 8,     // 8° max pitch bias for acceleration

    // Turn rate and holding patterns
    .maxTurnRate = 3,                 // 3 deg/s = rate-1 turn (360° in 2 minutes)
    .holdOrbitRadius = 1000,          // 10m orbit radius
    .holdFigure8Width = 2000,         // 20m figure-8 width

    // Landing sequence
    .landingDescentRate = 50,         // 50 cm/s = 0.5 m/s descent rate
    .landingDetectionTime = 10,       // 1 second below landing altitude for touchdown
    .landingSpiralEnable = 1,         // Spiral descent enabled (avoid vortex ring state)
    .landingSpiralRadius = 200,       // 2m spiral radius
    .landingSpiralRate = 10,          // 10 deg/s rotation rate
    .landingVelocityThreshold = 50,   // 0.5 m/s max velocity for touchdown
    .landingThrottleThreshold = 100,  // 100 RC units throttle deviation

    // L1 Nonlinear Guidance
    .l1Enable = 1,                    // L1 guidance enabled by default
    .l1Period = 20,                   // 2.0s damping period
    .l1MinLookahead = 1000,           // 10m minimum lookahead
    .l1MaxLookahead = 10000,          // 100m maximum lookahead
    .l1MaxCrossTrackError = 10000,    // 100m max XTE before fallback
    .l1TurnRate = 8,                  // 8 deg/s arc transition rate between waypoints

    // Vertical track
    .minNavAltitudeM = 5,                 // 5m AGL before horizontal navigation begins

    // Safety: RX loss policy
    .rxLossPolicy = AP_RX_LOSS_DISABLE,  // Disable autopilot on RX loss (use standard failsafe)

    // Safety: Geofence
    .maxDistanceFromHomeM = 0,            // Disabled by default
    .geofenceAction = AP_GEOFENCE_LAND,   // Land at current position
);
