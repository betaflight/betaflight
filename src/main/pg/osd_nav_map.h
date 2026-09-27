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

typedef enum {
    OSD_NAV_MAP_MODE_NORTH_UP = 0,
    OSD_NAV_MAP_MODE_HEADING_UP,
    OSD_NAV_MAP_MODE_COUNT
} osdNavMapMode_e;

typedef enum {
    OSD_NAV_MAP_CENTRE_HOME = 0,
    OSD_NAV_MAP_CENTRE_CRAFT,
    OSD_NAV_MAP_CENTRE_COUNT
} osdNavMapCentre_e;

typedef struct osdNavMapConfig_s {
    uint8_t mode;         // osdNavMapMode_e
    uint8_t centre;       // osdNavMapCentre_e
    uint16_t minScaleM;   // auto-zoom never zooms in past this map width: with home
                          // pinned at the centre, arriving home reads as flying from
                          // the map edge into the centre
} osdNavMapConfig_t;

PG_DECLARE(osdNavMapConfig_t, osdNavMapConfig);
