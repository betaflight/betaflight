/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>

#include "pg/pg.h"

typedef enum {
    NAV_HUD_MODE_OFF = 0,
    NAV_HUD_MODE_COMPACT,
    NAV_HUD_MODE_STANDARD,
    NAV_HUD_MODE_FULL,
    NAV_HUD_MODE_COUNT
} navHudMode_e;

typedef enum {
    NAV_HUD_ORIENTATION_NORTH_UP = 0,
    NAV_HUD_ORIENTATION_HEADING_UP,
    NAV_HUD_ORIENTATION_COUNT
} navHudOrientation_e;

typedef enum {
    NAV_HUD_CENTER_AUTO = 0,    // frame both craft and home
    NAV_HUD_CENTER_HOME,
    NAV_HUD_CENTER_CRAFT,
    NAV_HUD_CENTER_COUNT
} navHudCenter_e;

typedef struct navHudConfig_s {
    uint8_t mode;                 // navHudMode_e; OFF disables all nav HUD processing
    uint8_t orientation;          // navHudOrientation_e
    uint8_t center;               // navHudCenter_e
    uint8_t autoZoom;             // bool; when off, fixedScaleM is used
    uint16_t fixedScaleM;         // map width in metres when autoZoom is off
    uint8_t breadcrumbs;          // bool; record and draw the flown trail
    uint8_t breadcrumbCount;      // max stored trail points (RAM is reserved for the build-time cap)
    uint16_t breadcrumbSpacingM;  // minimum distance between stored trail points
    uint8_t projectedTrack;       // bool; draw short projected motion vector from ground course
    uint8_t homeLine;             // bool; draw direct line to home during normal flight
    uint8_t rescueRoute;          // bool; draw the planned GPS Rescue route while rescue is active
    uint8_t rescueExpand;         // bool; grow the map one size while rescue is active
    uint8_t showEta;              // bool
    uint8_t showXtrack;           // bool
    uint8_t showTargets;          // bool; show rescue target altitude/speed next to current values
    uint8_t showGpsHealth;        // bool; show satellite count and HDOP in the expanded footer
    uint8_t showSpeed;            // bool; ground speed centred in the top border
    uint8_t rangeRing;            // bool; faint dotted ring around home at a round distance
    uint8_t mission3D;            // bool; missions fly recorded waypoint altitudes (stadium profile)
} navHudConfig_t;

PG_DECLARE(navHudConfig_t, navHudConfig);
