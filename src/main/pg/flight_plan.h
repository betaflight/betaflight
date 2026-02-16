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

#include "pg/pg.h"

#define MAX_WAYPOINTS 30

typedef enum {
    WAYPOINT_TYPE_FLYOVER = 0,
    WAYPOINT_TYPE_FLYBY,
    WAYPOINT_TYPE_HOLD,
    WAYPOINT_TYPE_LAND
} waypointType_e;

typedef enum {
    WAYPOINT_PATTERN_ORBIT = 0,
    WAYPOINT_PATTERN_FIGURE8
} waypointPattern_e;

typedef struct {
    int32_t latitude;       // Latitude in degrees * 10^7 (7 decimal places, ±90.0000000)
    int32_t longitude;      // Longitude in degrees * 10^7 (7 decimal places, ±180.0000000)
    int32_t altitude;       // Altitude in centimeters above home point
    uint16_t speed;         // Speed in cm/s
    uint16_t duration;      // Duration in deciseconds (0.1s)
    uint8_t type;           // waypointType_e
    uint8_t pattern;        // waypointPattern_e (for hold type)
} waypoint_t;

typedef struct {
    uint8_t waypointCount;
    waypoint_t waypoints[MAX_WAYPOINTS];
} flightPlanConfig_t;

PG_DECLARE(flightPlanConfig_t, flightPlanConfig);
