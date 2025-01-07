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

PG_REGISTER_WITH_RESET_TEMPLATE(apConfig_t, apConfig, PG_AUTOPILOT, 2);

PG_RESET_TEMPLATE(apConfig_t, apConfig,
    .ap_landing_altitude_m = 4,
    .ap_hover_throttle = 1275,
    .ap_throttle_min = 1100,
    .ap_throttle_max = 1700,
    .ap_altitude_P = 15,
    .ap_altitude_I = 15,
    .ap_altitude_D = 15,
    .ap_altitude_F = 15,
    .ap_position_P = 30,
    .ap_position_I = 30,
    .ap_position_D = 30,
    .ap_position_A = 30,
    .ap_position_cutoff = 80,
    .ap_max_angle = 50,
);

#endif // !USE_WING
