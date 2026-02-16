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

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "platform.h"

#if ENABLE_FLIGHT_PLAN

#include "common/maths.h"
#include "common/time.h"

#include "drivers/time.h"

#include "fc/runtime_config.h"
#include "fc/rc.h"

#include "io/gps.h"

#include "pg/flight_plan.h"
#include "pg/autopilot.h"

#include "sensors/sensors.h"

#include "flight/autopilot_waypoint.h"
#include "flight/autopilot.h"
#include "flight/position.h"
#include "flight/autopilot_guidance.h"

// Static waypoint tracker state
static waypointTracker_t wpTracker = {
    .currentIndex = 0,
    .nextIndex = 0,
    .state = WP_STATE_IDLE,
    .distanceToCurrent = 0,
    .bearingToCurrent = 0,
    .holdStartTime = 0,
    .arrivalTime = 0,
    .landingStartTime = 0,
    .patternAngle = 0.0f,
    .landingStartAltitude = 0.0f,
    .isValid = false,
};

// Previous GPS position for plane crossing detection
static gpsLocation_t previousPosition = {0};

void waypointInit(void)
{
    // Called once at system init
    wpTracker.state = WP_STATE_IDLE;
    wpTracker.isValid = false;
    wpTracker.currentIndex = 0;
    wpTracker.nextIndex = 0;
    wpTracker.distanceToCurrent = 0;
    wpTracker.bearingToCurrent = 0;
}

// Check if all waypoints are within the configured geofence distance from home
static bool waypointsMeetGeofence(void)
{
    const autopilotConfig_t *cfg = autopilotConfig();

    if (cfg->maxDistanceFromHomeM == 0) {
        return true;  // Geofence disabled
    }

    if (!STATE(GPS_FIX_HOME)) {
        return false;  // Cannot validate without home position
    }

    const flightPlanConfig_t *plan = flightPlanConfig();
    const uint32_t maxDistanceCm = (uint32_t)cfg->maxDistanceFromHomeM * 100;

    for (uint8_t i = 0; i < plan->waypointCount && i < MAX_WAYPOINTS; i++) {
        gpsLocation_t wpLoc;
        wpLoc.lat = plan->waypoints[i].latitude;
        wpLoc.lon = plan->waypoints[i].longitude;
        wpLoc.altCm = plan->waypoints[i].altitude;

        uint32_t distCm;
        GPS_distance_cm_bearing(&GPS_home_llh, &wpLoc, false, &distCm, NULL);

        if (distCm > maxDistanceCm) {
            return false;
        }
    }

    return true;
}

// Inject a return-to-home sequence: fly to home position then land
void waypointSetReturnToHome(void)
{
    flightPlanConfig_t *config = flightPlanConfigMutable();

    // Waypoint 0: fly to home position
    config->waypoints[0].latitude = GPS_home_llh.lat;
    config->waypoints[0].longitude = GPS_home_llh.lon;
    config->waypoints[0].altitude = (int32_t)getAltitudeCm();
    config->waypoints[0].speed = 0;
    config->waypoints[0].duration = 0;
    config->waypoints[0].type = WAYPOINT_TYPE_FLYOVER;
    config->waypoints[0].pattern = 0;

    // Waypoint 1: land at home position
    config->waypoints[1].latitude = GPS_home_llh.lat;
    config->waypoints[1].longitude = GPS_home_llh.lon;
    config->waypoints[1].altitude = (int32_t)getAltitudeCm();
    config->waypoints[1].speed = 0;
    config->waypoints[1].duration = 0;
    config->waypoints[1].type = WAYPOINT_TYPE_LAND;
    config->waypoints[1].pattern = 0;

    config->waypointCount = 2;

    wpTracker.currentIndex = 0;
    wpTracker.nextIndex = 1;
    wpTracker.state = WP_STATE_APPROACHING;
    wpTracker.holdStartTime = 0;
    wpTracker.arrivalTime = 0;
    wpTracker.isValid = true;

    wpTracker.targetLocation.lat = GPS_home_llh.lat;
    wpTracker.targetLocation.lon = GPS_home_llh.lon;
    wpTracker.targetLocation.altCm = (int32_t)getAltitudeCm();
}

void waypointReset(void)
{
    // Called when entering autopilot mode
    const flightPlanConfig_t *plan = flightPlanConfig();

    // Validate we have waypoints
    if (plan->waypointCount == 0) {
        wpTracker.state = WP_STATE_IDLE;
        wpTracker.isValid = false;
        return;
    }

    // Reject corrupted waypointCount
    if (plan->waypointCount > MAX_WAYPOINTS) {
        wpTracker.state = WP_STATE_IDLE;
        wpTracker.isValid = false;
        return;
    }

    // Validate all waypoints are within geofence before starting mission
    if (!waypointsMeetGeofence()) {
        wpTracker.state = WP_STATE_IDLE;
        wpTracker.isValid = false;
        return;
    }

    // Start at first waypoint
    wpTracker.currentIndex = 0;
    wpTracker.nextIndex = (plan->waypointCount > 1) ? 1 : 0;
    wpTracker.state = WP_STATE_APPROACHING;
    wpTracker.holdStartTime = 0;
    wpTracker.arrivalTime = 0;
    wpTracker.patternAngle = 0.0f;
    wpTracker.isValid = true;

    // Set initial target
    const waypoint_t *wp = &plan->waypoints[0];
    wpTracker.targetLocation.lat = wp->latitude;
    wpTracker.targetLocation.lon = wp->longitude;
    wpTracker.targetLocation.altCm = wp->altitude;

    // Initialize previous position to current GPS position
    previousPosition = gpsSol.llh;
}

void waypointSetEmergencyLanding(void)
{
    // Inject a LAND waypoint at the current GPS position
    flightPlanConfig_t *config = flightPlanConfigMutable();
    config->waypoints[0].latitude = gpsSol.llh.lat;
    config->waypoints[0].longitude = gpsSol.llh.lon;
    config->waypoints[0].altitude = (int32_t)getAltitudeCm();
    config->waypoints[0].speed = 0;
    config->waypoints[0].duration = 0;
    config->waypoints[0].type = WAYPOINT_TYPE_LAND;
    config->waypoints[0].pattern = 0;
    config->waypointCount = 1;

    // Configure tracker to begin landing at this position
    wpTracker.currentIndex = 0;
    wpTracker.nextIndex = 0;
    wpTracker.state = WP_STATE_LANDING;
    wpTracker.landingStartTime = 0;  // Will be initialized on first update
    wpTracker.isValid = true;

    // Set target location to current position
    wpTracker.targetLocation.lat = gpsSol.llh.lat;
    wpTracker.targetLocation.lon = gpsSol.llh.lon;
    wpTracker.targetLocation.altCm = (int32_t)getAltitudeCm();
}

// Calculate orbit pattern target position
// centerLat/Lon: center of orbit
// radiusCm: radius in centimeters
// angleDeg: current angle (0-360 degrees, 0 = North, clockwise)
// result: calculated GPS position on orbit
static void calculateOrbitPosition(int32_t centerLat, int32_t centerLon,
                                    uint16_t radiusCm, float angleDeg,
                                    gpsLocation_t *result)
{
    // Convert radius from cm to degrees (approximate)
    // At equator: 1 degree latitude ≈ 111km = 11,100,000 cm
    // Longitude varies by latitude, use latitude scaling
    const float radiusLatDeg = (float)radiusCm / 11100000.0f;

    // Calculate latitude offset
    float angleRad = angleDeg * 0.0174533f;  // degrees to radians
    float latOffsetDeg = radiusLatDeg * cosf(angleRad);

    // Calculate longitude offset (scale by latitude)
    float latRad = centerLat * 1e-7f * 0.0174533f;
    float lonScaling = cosf(latRad);
    float lonOffsetDeg = (radiusLatDeg * sinf(angleRad)) / lonScaling;

    // Apply offsets
    result->lat = centerLat + (int32_t)(latOffsetDeg * 1e7f);
    result->lon = centerLon + (int32_t)(lonOffsetDeg * 1e7f);
}

// Calculate figure-8 pattern target position
// centerLat/Lon: center of figure-8
// widthCm: width of figure-8 (distance between loop centers)
// angleDeg: current angle (0-720 degrees for complete figure-8)
// result: calculated GPS position
static void calculateFigure8Position(int32_t centerLat, int32_t centerLon,
                                     uint16_t widthCm, float angleDeg,
                                     gpsLocation_t *result)
{
    // Figure-8 is two orbits side-by-side
    // Loop radius is half the width
    const uint16_t loopRadius = widthCm / 2;

    // Determine which loop (0-360 = left loop, 360-720 = right loop)
    const float loopAngle = fmodf(angleDeg, 360.0f);
    const bool rightLoop = (angleDeg >= 360.0f);

    // Calculate center offset for current loop
    // Left loop: offset west (-), Right loop: offset east (+)
    const float offsetLatDeg = (rightLoop ? 1.0f : -1.0f) * (float)loopRadius / 11100000.0f;
    const float latRad = centerLat * 1e-7f * 0.0174533f;
    const float lonScaling = cosf(latRad);
    const int32_t loopCenterLat = centerLat;
    const int32_t loopCenterLon = centerLon + (int32_t)(offsetLatDeg * 1e7f / lonScaling);

    // Calculate position on current loop
    calculateOrbitPosition(loopCenterLat, loopCenterLon, loopRadius, loopAngle, result);
}

/*
 * FLYOVER vs FLYBY Waypoint Transition Logic
 * ==========================================
 *
 * FLYOVER (Precision Path):
 * - Used for photogrammetry, obstacle clearance, or when exact waypoint passage is required
 * - The aircraft maintains its current track until crossing the perpendicular plane through the waypoint
 * - Transition occurs when the dot product of (position - waypoint) with path direction changes from negative to positive
 * - Creates a characteristic "bulb" overshoot as the aircraft turns after passing the waypoint
 * - Behavior: Track(WP_{n-1} → WP_n) maintained until plane crossed, then snap to Track(WP_n → WP_{n+1})
 *
 * FLYBY (Smooth Path / Waypoint Anticipation):
 * - Used for efficient navigation when exact waypoint passage is not critical
 * - The aircraft begins turning before reaching the waypoint to maintain smooth flight
 * - Transition occurs when distance to waypoint ≤ turn radius (calculated from speed and bank angle)
 * - Creates a smooth tangent arc between path segments, "cutting the corner"
 * - Reduces mechanical stress and maintains higher ground speed through turns
 * - Turn radius formula: R = V² / (g × tan(bank_angle)), where g ≈ 980 cm/s²
 *
 * L1 Guidance Integration:
 * - FLYOVER: L1 disabled on arrival to allow natural overshoot completion
 * - FLYBY: L1 remains active through transition for continuous smooth path following
 */

// Calculate turn radius based on ground speed and maximum bank angle
// Returns turn radius in centimeters
// Formula: R = V² / (g × tan(bank_angle))
// where g ≈ 980 cm/s² (gravity)
static uint32_t calculateTurnRadius(float groundSpeedCmS, float maxAngleDeg)
{
    // Convert angle to radians
    const float maxAngleRad = maxAngleDeg * 0.0174533f;

    // Calculate turn radius
    // R = V² / (g × tan(θ))
    const float gravity = 980.0f;  // cm/s²
    const float tanAngle = tanf(maxAngleRad);

    if (tanAngle < 0.1f) {
        // Prevent division by very small numbers
        return 100000;  // 1km maximum turn radius
    }

    const float radiusCm = (groundSpeedCmS * groundSpeedCmS) / (gravity * tanAngle);

    // Constrain to reasonable values (50cm to 1km)
    return constrainf(radiusCm, 50.0f, 100000.0f);
}

// Check if we've crossed the perpendicular plane through the waypoint
// This is used for FLYOVER waypoints to determine when to transition
// Returns true when we've crossed from the approach side to the departure side
static bool hasPassedWaypointPlane(const gpsLocation_t *prevPos,
                                   const gpsLocation_t *currentPos,
                                   const gpsLocation_t *waypointPos,
                                   const gpsLocation_t *nextWaypointPos)
{
    // Calculate path vector (from current waypoint to next waypoint)
    vector3_t pathNED;
    gpsLocation_t wpLoc = *waypointPos;
    navOriginLLHtoNED(&wpLoc, &pathNED);

    vector3_t nextNED;
    gpsLocation_t nextLoc = *nextWaypointPos;
    navOriginLLHtoNED(&nextLoc, &nextNED);

    // Path direction vector (normalize in 2D)
    vector2_t pathDir;
    pathDir.x = nextNED.x - pathNED.x;
    pathDir.y = nextNED.y - pathNED.y;

    float pathLen = sqrtf(pathDir.x * pathDir.x + pathDir.y * pathDir.y);
    if (pathLen < 10.0f) {
        // Path too short, treat as crossed
        return true;
    }

    pathDir.x /= pathLen;
    pathDir.y /= pathLen;

    // Calculate vectors from waypoint to previous and current positions
    vector3_t prevNED, currNED;
    gpsLocation_t prevLoc = *prevPos;
    navOriginLLHtoNED(&prevLoc, &prevNED);

    gpsLocation_t currLoc = *currentPos;
    navOriginLLHtoNED(&currLoc, &currNED);

    // Vector from waypoint to previous position
    vector2_t toPrev;
    toPrev.x = prevNED.x - pathNED.x;
    toPrev.y = prevNED.y - pathNED.y;

    // Vector from waypoint to current position
    vector2_t toCurr;
    toCurr.x = currNED.x - pathNED.x;
    toCurr.y = currNED.y - pathNED.y;

    // Dot product with path direction
    // Positive = ahead of waypoint, Negative = behind waypoint
    float prevDot = toPrev.x * pathDir.x + toPrev.y * pathDir.y;
    float currDot = toCurr.x * pathDir.x + toCurr.y * pathDir.y;

    // Crossed the plane when sign changes from negative to positive
    return (prevDot < 0.0f && currDot >= 0.0f);
}

void waypointUpdate(timeUs_t currentTimeUs)
{
    const flightPlanConfig_t *plan = flightPlanConfig();
    const autopilotConfig_t *cfg = autopilotConfig();

    // Safety check: GPS must be valid
    if (!sensors(SENSOR_GPS) || !STATE(GPS_FIX)) {
        wpTracker.isValid = false;
        wpTracker.state = WP_STATE_IDLE;
        return;
    }

    // Safety check: Must have waypoints
    if (plan->waypointCount == 0) {
        wpTracker.isValid = false;
        wpTracker.state = WP_STATE_IDLE;
        return;
    }

    // Runtime geofence check: enforce max distance from home
    if (cfg->maxDistanceFromHomeM > 0 && STATE(GPS_FIX_HOME)) {
        const uint32_t maxDistanceCm = (uint32_t)cfg->maxDistanceFromHomeM * 100;

        if (GPS_distanceToHomeCm > maxDistanceCm) {
            if (wpTracker.state != WP_STATE_LANDING && wpTracker.state != WP_STATE_COMPLETE) {
                if ((autopilotGeofenceAction_e)cfg->geofenceAction == AP_GEOFENCE_RTH) {
                    waypointSetReturnToHome();
                } else {
                    waypointSetEmergencyLanding();
                }
                return;
            }
        }
    }

    // Update distance and bearing to current waypoint
    GPS_distance_cm_bearing(&gpsSol.llh, &wpTracker.targetLocation, false,
                            &wpTracker.distanceToCurrent, &wpTracker.bearingToCurrent);

    // State machine
    switch (wpTracker.state) {
    case WP_STATE_IDLE:
        // Nothing to do
        break;

    case WP_STATE_APPROACHING: {
        const waypoint_t *currentWp = &plan->waypoints[wpTracker.currentIndex];

        // Enable L1 guidance if configured and we have a path to follow
        if (cfg->l1Enable && !l1GuidanceIsActive() && plan->waypointCount > 1) {
            // Create L1 path from previous waypoint (or current position for first waypoint) to current waypoint
            vector3_t currentNED, targetNED;
            navOriginLLHtoNED(&gpsSol.llh, &currentNED);
            navOriginLLHtoNED(&wpTracker.targetLocation, &targetNED);

            // Set up L1 path (2D: North, East)
            vector2_t pathStart = { .x = currentNED.x, .y = currentNED.y };
            vector2_t pathEnd = { .x = targetNED.x, .y = targetNED.y };

            l1GuidanceSetPath(&pathStart, &pathEnd);
            l1GuidanceSetActive(true);
        }

        bool shouldTransition = false;

        // Handle waypoint type-specific transition logic
        switch (currentWp->type) {
        case WAYPOINT_TYPE_FLYOVER:
            // FLYOVER: Pass directly through waypoint (creates "bulb" overshoot)
            // Transition when crossing the perpendicular plane through the waypoint
            if (wpTracker.nextIndex != wpTracker.currentIndex && plan->waypointCount > 1) {
                const waypoint_t *nextWp = &plan->waypoints[wpTracker.nextIndex];
                gpsLocation_t nextWpLoc = {
                    .lat = nextWp->latitude,
                    .lon = nextWp->longitude,
                    .altCm = nextWp->altitude
                };

                // Check if we've crossed the plane
                if (hasPassedWaypointPlane(&previousPosition, &gpsSol.llh,
                                          &wpTracker.targetLocation, &nextWpLoc)) {
                    shouldTransition = true;
                }
            } else {
                // Last waypoint or single waypoint - use arrival radius
                if (wpTracker.distanceToCurrent <= cfg->waypointArrivalRadius) {
                    shouldTransition = true;
                }
            }
            break;

        case WAYPOINT_TYPE_FLYBY:
            // FLYBY: Cut corners smoothly (waypoint anticipation)
            // Transition when distance to waypoint < turn radius
            if (wpTracker.nextIndex != wpTracker.currentIndex && plan->waypointCount > 1) {
                // Calculate turn radius based on ground speed and max bank angle
                float groundSpeedCmS = gpsSol.groundSpeed;
                if (groundSpeedCmS < 100.0f) {
                    groundSpeedCmS = 100.0f;  // Minimum 1 m/s for calculation
                }

                uint32_t turnRadius = calculateTurnRadius(groundSpeedCmS, cfg->maxAngle);

                // Transition when within turn radius of waypoint
                if (wpTracker.distanceToCurrent <= turnRadius) {
                    shouldTransition = true;
                }
            } else {
                // Last waypoint - use arrival radius
                if (wpTracker.distanceToCurrent <= cfg->waypointArrivalRadius) {
                    shouldTransition = true;
                }
            }
            break;

        case WAYPOINT_TYPE_HOLD:
        case WAYPOINT_TYPE_LAND:
            // HOLD and LAND: Use tight arrival radius for precision
            if (wpTracker.distanceToCurrent <= cfg->waypointHoldRadius) {
                shouldTransition = true;
            }
            break;
        }

        // Perform transition if triggered
        if (shouldTransition) {
            wpTracker.state = WP_STATE_ARRIVED;
            wpTracker.arrivalTime = currentTimeUs;

            // For FLYBY with next waypoint, keep L1 active for smooth transition
            // For FLYOVER, disable L1 to allow overshoot completion
            if (currentWp->type != WAYPOINT_TYPE_FLYBY ||
                wpTracker.nextIndex == wpTracker.currentIndex) {
                l1GuidanceSetActive(false);
            }
        }
        break;
    }

    case WP_STATE_ARRIVED: {
        const waypoint_t *currentWp = &plan->waypoints[wpTracker.currentIndex];
        const uint8_t currentWpType = currentWp->type;  // Save type before advancing

        // Handle based on waypoint type
        switch (currentWpType) {
        case WAYPOINT_TYPE_FLYOVER:
        case WAYPOINT_TYPE_FLYBY: {
            // Save current waypoint location before advancing
            gpsLocation_t arrivedWpLoc = {
                .lat = currentWp->latitude,
                .lon = currentWp->longitude,
                .altCm = currentWp->altitude
            };

            // Continue immediately to next waypoint
            waypointAdvanceToNext();

            // For FLYBY with L1 enabled, immediately set up path to next waypoint for smooth corner cutting
            if (currentWpType == WAYPOINT_TYPE_FLYBY &&
                cfg->l1Enable &&
                wpTracker.state == WP_STATE_APPROACHING &&  // Successfully advanced
                plan->waypointCount > 1) {

                // Set up L1 path from arrived waypoint to new current waypoint
                const waypoint_t *newCurrentWp = &plan->waypoints[wpTracker.currentIndex];
                gpsLocation_t newCurrentWpLoc = {
                    .lat = newCurrentWp->latitude,
                    .lon = newCurrentWp->longitude,
                    .altCm = newCurrentWp->altitude
                };

                vector3_t arrivedWpNED, newCurrentWpNED;
                navOriginLLHtoNED(&arrivedWpLoc, &arrivedWpNED);
                navOriginLLHtoNED(&newCurrentWpLoc, &newCurrentWpNED);

                vector2_t pathStart = { .x = arrivedWpNED.x, .y = arrivedWpNED.y };
                vector2_t pathEnd = { .x = newCurrentWpNED.x, .y = newCurrentWpNED.y };

                l1GuidanceSetPath(&pathStart, &pathEnd);
                l1GuidanceSetActive(true);
            }
            break;
        }

        case WAYPOINT_TYPE_HOLD:
            // Enter hold pattern
            wpTracker.state = WP_STATE_HOLDING;
            wpTracker.holdStartTime = currentTimeUs;
            break;

        case WAYPOINT_TYPE_LAND:
            // Enter landing sequence
            wpTracker.state = WP_STATE_LANDING;
            wpTracker.landingStartTime = 0;  // Will be initialized on first update
            break;
        }
        break;
    }

    case WP_STATE_HOLDING: {
        const waypoint_t *currentWp = &plan->waypoints[wpTracker.currentIndex];

        // Check if pattern requested
        if (currentWp->pattern == WAYPOINT_PATTERN_ORBIT) {
            // Enter orbit pattern
            wpTracker.state = WP_STATE_ORBITING;
            wpTracker.patternAngle = 0.0f;  // Start at north
            wpTracker.patternCenter = wpTracker.targetLocation;  // Orbit around waypoint
        } else if (currentWp->pattern == WAYPOINT_PATTERN_FIGURE8) {
            // Enter figure-8 pattern
            wpTracker.state = WP_STATE_FIGURE8;
            wpTracker.patternAngle = 0.0f;  // Start at left loop north
            wpTracker.patternCenter = wpTracker.targetLocation;  // Figure-8 centered at waypoint
        } else {
            // Just hold position
            // Check if hold duration expired (0 = infinite hold)
            if (currentWp->duration > 0) {
                timeDelta_t holdDuration = (currentWp->duration * 100000);  // deciseconds to microseconds
                if (cmpTimeUs(currentTimeUs, wpTracker.holdStartTime) >= holdDuration) {
                    // Hold complete, advance to next
                    waypointAdvanceToNext();
                }
            }
        }
        break;
    }

    case WP_STATE_ORBITING: {
        const autopilotConfig_t *cfg = autopilotConfig();
        const waypoint_t *currentWp = &plan->waypoints[wpTracker.currentIndex];

        // Increment angle based on turn rate and time
        // Turn rate is in deg/s, update at 100Hz means deg/s * 0.01 per update
        const float angleIncrement = cfg->maxTurnRate * 0.01f;  // degrees per 10ms update
        wpTracker.patternAngle += angleIncrement;

        // Wrap angle to 0-360
        if (wpTracker.patternAngle >= 360.0f) {
            wpTracker.patternAngle -= 360.0f;
        }

        // Calculate target position on orbit
        calculateOrbitPosition(wpTracker.patternCenter.lat,
                                wpTracker.patternCenter.lon,
                                cfg->holdOrbitRadius,
                                wpTracker.patternAngle,
                                &wpTracker.targetLocation);

        // Maintain waypoint altitude
        wpTracker.targetLocation.altCm = currentWp->altitude;

        // Check if hold duration expired (0 = infinite)
        if (currentWp->duration > 0) {
            timeDelta_t holdDuration = (currentWp->duration * 100000);  // deciseconds to microseconds
            if (cmpTimeUs(currentTimeUs, wpTracker.holdStartTime) >= holdDuration) {
                // Hold complete, advance to next
                waypointAdvanceToNext();
            }
        }
        break;
    }

    case WP_STATE_FIGURE8: {
        const autopilotConfig_t *cfg = autopilotConfig();
        const waypoint_t *currentWp = &plan->waypoints[wpTracker.currentIndex];

        // Increment angle based on turn rate
        // Figure-8 requires 720 degrees for complete pattern
        const float angleIncrement = cfg->maxTurnRate * 0.01f;
        wpTracker.patternAngle += angleIncrement;

        // Wrap angle to 0-720
        if (wpTracker.patternAngle >= 720.0f) {
            wpTracker.patternAngle -= 720.0f;
        }

        // Calculate target position on figure-8
        calculateFigure8Position(wpTracker.patternCenter.lat,
                                 wpTracker.patternCenter.lon,
                                 cfg->holdFigure8Width,
                                 wpTracker.patternAngle,
                                 &wpTracker.targetLocation);

        // Maintain waypoint altitude
        wpTracker.targetLocation.altCm = currentWp->altitude;

        // Check if hold duration expired
        if (currentWp->duration > 0) {
            timeDelta_t holdDuration = (currentWp->duration * 100000);
            if (cmpTimeUs(currentTimeUs, wpTracker.holdStartTime) >= holdDuration) {
                waypointAdvanceToNext();
            }
        }
        break;
    }

    case WP_STATE_LANDING: {
        // Progressive descent for gentle landing with optional spiral pattern
        static timeUs_t belowAltitudeStartTime = 0;
        static int32_t landingCenterLat = 0;
        static int32_t landingCenterLon = 0;
        const waypoint_t *currentWp = &plan->waypoints[wpTracker.currentIndex];

        // Initialize landing on first entry
        if (wpTracker.landingStartTime == 0) {
            wpTracker.landingStartTime = currentTimeUs;
            wpTracker.landingStartAltitude = getAltitudeCm();
            belowAltitudeStartTime = 0;  // Reset touchdown timer
            // Store landing center point
            landingCenterLat = currentWp->latitude;
            landingCenterLon = currentWp->longitude;
        }

        // Calculate time in landing phase (seconds)
        const float landingTimeS = (currentTimeUs - wpTracker.landingStartTime) / 1000000.0f;

        // Calculate target altitude based on descent rate
        // landingDescentRate is in cm/s
        const float descentCm = cfg->landingDescentRate * landingTimeS;
        const float targetAltCm = wpTracker.landingStartAltitude - descentCm;

        // Stop descent at landing altitude threshold
        const float landingAltitudeCm = cfg->landingAltitudeM * 100.0f;
        if (targetAltCm < landingAltitudeCm) {
            wpTracker.targetLocation.altCm = (int32_t)landingAltitudeCm;
        } else {
            wpTracker.targetLocation.altCm = (int32_t)targetAltCm;
        }

        // Horizontal position: straight down or spiral pattern
        if (cfg->landingSpiralEnable) {
            // Spiral descent to avoid vortex ring state
            // Calculate spiral angle based on time and rotation rate
            const float spiralAngleDeg = landingTimeS * cfg->landingSpiralRate;
            const float spiralAngleRad = spiralAngleDeg * (M_PIf / 180.0f);

            // Calculate offset from center in NED frame
            const float offsetNorth = cfg->landingSpiralRadius * cos_approx(spiralAngleRad);  // cm
            const float offsetEast = cfg->landingSpiralRadius * sin_approx(spiralAngleRad);   // cm

            // Convert offset to lat/lon (flat-earth approximation)
            // Calculate cos(lat) for longitude scaling at landing location
            const float cosLat = cos_approx(DEGREES_TO_RADIANS((float)landingCenterLat / GPS_DEGREES_DIVIDER));

            const int32_t latOffset = (int32_t)(offsetNorth / EARTH_ANGLE_TO_CM);
            const int32_t lonOffset = (int32_t)(offsetEast / (EARTH_ANGLE_TO_CM * cosLat));

            wpTracker.targetLocation.lat = landingCenterLat + latOffset;
            wpTracker.targetLocation.lon = landingCenterLon + lonOffset;
        } else {
            // Straight descent - maintain position at waypoint
            wpTracker.targetLocation.lat = landingCenterLat;
            wpTracker.targetLocation.lon = landingCenterLon;
        }

        // Enhanced touchdown detection: multiple conditions must be met
        bool altitudeCondition = isBelowLandingAltitude();
        bool velocityCondition = (gpsSol.groundSpeed < cfg->landingVelocityThreshold);
        bool throttleCondition = (rcCommand[THROTTLE] < (cfg->hoverThrottle - cfg->landingThrottleThreshold));

        // All conditions must be met for touchdown
        if (altitudeCondition && velocityCondition && throttleCondition) {
            // Start touchdown timer
            if (belowAltitudeStartTime == 0) {
                belowAltitudeStartTime = currentTimeUs;
            }

            const timeDelta_t belowAltitudeDuration = cmpTimeUs(currentTimeUs, belowAltitudeStartTime);
            const timeDelta_t detectionThreshold = cfg->landingDetectionTime * 100000; // deciseconds to microseconds

            if (belowAltitudeDuration >= detectionThreshold) {
                // Touchdown detected - transition to complete state
                // Disarm will be handled by pid.c jerk detection
                wpTracker.state = WP_STATE_COMPLETE;
                wpTracker.landingStartTime = 0;  // Reset for next landing
                belowAltitudeStartTime = 0;
            }
        } else {
            // Reset timer if any condition fails
            belowAltitudeStartTime = 0;
        }
        break;
    }

    case WP_STATE_COMPLETE:
        // All waypoints complete, maintain last position
        break;
    }

    // Store current position for next iteration's plane crossing detection
    previousPosition = gpsSol.llh;
}

void waypointAdvanceToNext(void)
{
    const flightPlanConfig_t *plan = flightPlanConfig();
    const autopilotConfig_t *cfg = autopilotConfig();

    // Check if there are more waypoints
    if (wpTracker.nextIndex < plan->waypointCount &&
        wpTracker.nextIndex != wpTracker.currentIndex) {

        // Save previous waypoint location for L1 path setup
        gpsLocation_t prevWpLoc = wpTracker.targetLocation;

        // Advance to next waypoint
        wpTracker.currentIndex = wpTracker.nextIndex;
        wpTracker.nextIndex = (wpTracker.currentIndex + 1 < plan->waypointCount)
                                ? wpTracker.currentIndex + 1
                                : wpTracker.currentIndex;

        // Update target location
        const waypoint_t *wp = &plan->waypoints[wpTracker.currentIndex];
        wpTracker.targetLocation.lat = wp->latitude;
        wpTracker.targetLocation.lon = wp->longitude;
        wpTracker.targetLocation.altCm = wp->altitude;

        // Reset to approaching state
        wpTracker.state = WP_STATE_APPROACHING;
        wpTracker.holdStartTime = 0;
        wpTracker.arrivalTime = 0;

        // Re-enable L1 guidance for the new path segment
        // l1GuidanceUpdate() can deactivate itself near endpoints, so
        // explicitly re-activate when starting a new segment
        if (cfg->l1Enable && plan->waypointCount > 1) {
            vector3_t prevNED, newNED;
            navOriginLLHtoNED(&prevWpLoc, &prevNED);
            gpsLocation_t newWpLoc = wpTracker.targetLocation;
            navOriginLLHtoNED(&newWpLoc, &newNED);

            vector2_t pathStart = { .x = prevNED.x, .y = prevNED.y };
            vector2_t pathEnd = { .x = newNED.x, .y = newNED.y };

            l1GuidanceSetPath(&pathStart, &pathEnd);
            l1GuidanceSetActive(true);
        }

    } else {
        // No more waypoints - mission complete
        wpTracker.state = WP_STATE_COMPLETE;
        wpTracker.nextIndex = wpTracker.currentIndex;
    }
}

bool waypointIsValid(uint8_t idx)
{
    const flightPlanConfig_t *plan = flightPlanConfig();
    return (idx < MAX_WAYPOINTS && idx < plan->waypointCount);
}

const gpsLocation_t* waypointGetTarget(void)
{
    return &wpTracker.targetLocation;
}

uint8_t waypointGetCurrentIndex(void)
{
    return wpTracker.currentIndex;
}

uint8_t waypointGetNextIndex(void)
{
    return wpTracker.nextIndex;
}

uint32_t waypointGetDistanceCm(void)
{
    return wpTracker.distanceToCurrent;
}

int32_t waypointGetBearingCdeg(void)
{
    return wpTracker.bearingToCurrent;
}

waypointState_e waypointGetState(void)
{
    return wpTracker.state;
}

bool waypointIsSystemValid(void)
{
    return wpTracker.isValid;
}

uint8_t waypointGetCurrentType(void)
{
    const flightPlanConfig_t *plan = flightPlanConfig();
    if (wpTracker.currentIndex < plan->waypointCount) {
        return plan->waypoints[wpTracker.currentIndex].type;
    }
    return WAYPOINT_TYPE_FLYOVER;
}

uint16_t waypointGetHoldTimeRemaining(void)
{
    if (wpTracker.state != WP_STATE_HOLDING) {
        return 0;
    }

    const flightPlanConfig_t *plan = flightPlanConfig();
    const waypoint_t *wp = &plan->waypoints[wpTracker.currentIndex];

    if (wp->duration == 0) {
        return 65535;  // Infinite hold
    }

    timeDelta_t elapsed = cmpTimeUs(micros(), wpTracker.holdStartTime);
    timeDelta_t total = wp->duration * 100000;  // deciseconds to microseconds

    if (elapsed >= total) {
        return 0;
    }

    return (uint16_t)((total - elapsed) / 100000);  // Convert back to deciseconds
}

#endif // ENABLE_FLIGHT_PLAN
