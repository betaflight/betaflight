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

#include <stdbool.h>

#include "common/time.h"

#include "drivers/display.h"

#define OSD_SCREENSAVER_TIMEOUT_MAX 600     // seconds

typedef enum {
    OSD_SCREENSAVER_MATRIX,
    OSD_SCREENSAVER_TUNNEL,
    OSD_SCREENSAVER_TERRAIN,
    OSD_SCREENSAVER_CUBE,
    OSD_SCREENSAVER_STARS,
    OSD_SCREENSAVER_ARROWS,
    OSD_SCREENSAVER_WAVES,
    OSD_SCREENSAVER_EFFECT_COUNT,
    OSD_SCREENSAVER_CYCLE = OSD_SCREENSAVER_EFFECT_COUNT,   // all of them in turn
} osdScreensaverEffect_e;

#define OSD_SCREENSAVER_CYCLE_DEFAULT_S 20  // seconds an effect runs in CYCLE
#define OSD_SCREENSAVER_CYCLE_MIN_S 5
#define OSD_SCREENSAVER_CYCLE_MAX_S 600

// Disarmed, no menu and no failsafe, and either no stick or switch movement for osd_screensaver_timeout
// seconds or the SCREENSAVER mode on
bool osdScreensaverActive(displayPort_t *displayPort, timeUs_t currentTimeUs);
// Draw the next rows of a frame of the screensaver; true while rows of the frame are left. On an MSP
// display port's 53x20 canvas (DJI O3 / O4) the cells under the goggles' own widgets are left blank.
bool osdScreensaverDraw(displayPort_t *displayPort, bool mspDisplayPort, timeUs_t currentTimeUs);
