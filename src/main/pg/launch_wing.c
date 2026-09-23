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

#if defined(USE_WING) && defined(USE_LAUNCH_WING)

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "launch_wing.h"

PG_REGISTER_WITH_RESET_TEMPLATE(launchWingConfig_t, launchWingConfig, PG_LAUNCH_WING_CONFIG, 1);

PG_RESET_TEMPLATE(launchWingConfig_t, launchWingConfig,
    .climbAngleDeg = 18,
    .maxAngleDeg = 45,
    .detectAccelDecig = 19,
    .detectVelocityCmS = 300,
    .detectTimeMs = 40,
    .idleThrottlePercent = 5,
    .idleDelayMs = 0,
    .throttlePercent = 80,
    .motorDelayMs = 500,
    .spinupTimeMs = 100,
    .minTimeMs = 0,
    .timeoutMs = 5000,
    .maxAltitudeM = 0,
    .endTimeMs = 3000,
    .abortDeadbandPercent = 20,
    .abortAngleDeg = 60,
);

#endif
