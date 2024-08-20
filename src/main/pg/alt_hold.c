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

#ifdef USE_ALT_HOLD_MODE

#include "flight/alt_hold.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "alt_hold.h"

PG_REGISTER_WITH_RESET_TEMPLATE(altholdConfig_t, altholdConfig, PG_ALTHOLD_CONFIG, 3);

PG_RESET_TEMPLATE(altholdConfig_t, altholdConfig,
    .alt_hold_pid_p = 15,
    .alt_hold_pid_i = 15,
    .alt_hold_pid_d = 15,
    .alt_hold_throttle_min = 1100,
    .alt_hold_throttle_max = 1700,
    .alt_hold_target_adjust_rate = 100, // max rate of change of altitude target using sticks in cm/s
);
#endif
