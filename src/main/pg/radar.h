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

/* Settings for the radar OSD elements: the fixed peer readout and the HUD */
typedef struct radarConfig_s {
    uint8_t peerDisplayTimeS;       // fixed readout: seconds on each peer before moving to the next
    uint8_t hudMaxPeers;            // HUD: most peers drawn at once
    uint8_t hudRangeMinM;           // HUD: peers closer than this are not drawn
    uint16_t hudRangeMaxM;          // HUD: peers further than this are not drawn
    uint8_t hudAltTimeS;            // HUD: seconds showing the altitude difference ...
    uint8_t hudDistTimeS;           // ... then seconds showing the distance
    uint8_t cameraFovH;             // degrees
    uint8_t cameraFovV;             // degrees
    int8_t cameraUptilt;            // degrees
    uint8_t hudMarginH;             // columns
    uint8_t hudMarginV;             // rows
} radarConfig_t;

PG_DECLARE(radarConfig_t, radarConfig);
