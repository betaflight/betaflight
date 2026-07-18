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

#if ENABLE_DRONECAN && defined(USE_MAG)

#include <stdbool.h>
#include <stdint.h>

#include "common/axis.h"
#include "common/time.h"

// Install the MagneticFieldStrength2 subscriber. Call once from dronecanInit()
// after the base subscriber table has been initialised.
void dronecanMagInit(void);

// Copy the last-received field vector (milligauss, body frame) into the
// caller's buffer. Returns false if no frame has been received yet.
bool dronecanMagGetLatest(int16_t mag[XYZ_AXIS_COUNT]);

// Microsecond timestamp of the most recent accepted frame (host clock, not the
// DSDL timestamp). Used by the compass driver to detect staleness.
timeUs_t dronecanMagLastUpdateUs(void);

#endif // ENABLE_DRONECAN && USE_MAG
