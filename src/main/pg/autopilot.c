/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform.h"

#include "flight/autopilot.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "autopilot.h"

PG_REGISTER_WITH_RESET_TEMPLATE(autopilotConfig_t, autopilotConfig, PG_AUTOPILOT, 1);

PG_RESET_TEMPLATE(autopilotConfig_t, autopilotConfig,
    .landing_altitude_m = 4,
    .hover_throttle = 1275,
    .throttle_min = 1100,
    .throttle_max = 1700,
    .altitude_P = 15,
    .altitude_I = 15,
    .altitude_D = 15,
    .altitude_F = 15,
    .position_P = 15,
    .position_I = 15,
    .position_D = 15,
    .position_J = 15,
    .position_filter_hz = 75,
);
