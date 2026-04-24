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

#include "platform.h"

#if ENABLE_DRONECAN

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"
#include "io/gps.h"

// Install the Fix2 subscriber. Call once from dronecanInit() after the base
// subscriber table has been initialised.
void dronecanGnssInit(void);

// Pull the last-received solution into the caller's buffer. Returns false if
// no frame has been received yet. Populated fields today: llh, numSat,
// groundSpeed, groundCourse, speed3d, velned. dop / acc / dateTime are left
// zero until a follow-up decodes covariance + pdop past the Fix2 TAO fields.
bool dronecanGnssGetLatest(gpsSolutionData_t *out);

// Microsecond timestamp of the most recent accepted Fix2 (host clock, not
// the DSDL timestamp). Used by gps.c to detect staleness.
timeUs_t dronecanGnssLastUpdateUs(void);

#endif // ENABLE_DRONECAN
