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

#if defined(USE_WING) && defined(USE_LAUNCH_WING)

#include <stdint.h>

#include "pg/pg.h"

typedef struct launchWingConfig_s {
    uint8_t climbAngleDeg;          // pitch target held through the climb-out
    uint8_t maxAngleDeg;            // tilt gate for throw detection
    uint8_t detectAccelDecig;       // forward specific force, 0.1 g
    uint16_t detectVelocityCmS;     // swing and ground-speed detection threshold
    uint16_t detectTimeMs;          // how long detection must hold continuously
    uint8_t idleThrottlePercent;    // throttle while waiting for the throw
    uint16_t idleDelayMs;           // throttle raise to idle spin-up
    uint8_t throttlePercent;        // climb-out throttle
    uint16_t motorDelayMs;          // detection to spin-up
    uint16_t spinupTimeMs;          // idle to launch throttle ramp
    uint16_t minTimeMs;             // detection to stick-abort arming
    uint16_t timeoutMs;             // hard exit from the climb-out
    uint16_t maxAltitudeM;          // AGL exit, 0 disables
    uint16_t endTimeMs;             // cross-fade back to the pilot
    uint8_t abortDeadbandPercent;   // roll/pitch deflection that aborts
    uint8_t abortAngleDeg;          // bank and dive bound, 0 disables
} launchWingConfig_t;

PG_DECLARE(launchWingConfig_t, launchWingConfig);

#endif
