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

#include "flight/position_control.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "position_control.h"

PG_REGISTER_WITH_RESET_TEMPLATE(positionControlConfig_t, positionControlConfig, PG_POSITION_CONTROL, 1);

PG_RESET_TEMPLATE(positionControlConfig_t, positionControlConfig,
    .hover_throttle = 1275,
    .alt_control_throttle_min = 1100,
    .alt_control_throttle_max = 1700,
    .landing_altitude_m = 4,
    .altitude_P = 15,
    .altitude_I = 15,
    .altitude_D = 15,
    .altitude_F = 15,
);
