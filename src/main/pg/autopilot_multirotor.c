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

#ifndef USE_WING

#include "flight/autopilot.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "autopilot.h"

PG_REGISTER_WITH_RESET_TEMPLATE(autopilotConfig_t, autopilotConfig, PG_AUTOPILOT, 3);

PG_RESET_TEMPLATE(autopilotConfig_t, autopilotConfig,
    .landingAltitudeM = 4,
    .hoverThrottle = 1275,
    .throttleMin = 1100,
    .throttleMax = 1700,
    .altitudeP = 15,
    .altitudeI = 15,
    .altitudeD = 15,
    .altitudeF = 15,
    .positionP = 30,
    .positionI = 30,
    .positionII = 30,
    .positionD = 30,
    .positionA = 30,
    .positionCutoff = 80,
    .maxAngle = 50,
);

#endif // !USE_WING
