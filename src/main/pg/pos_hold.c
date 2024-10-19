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

#ifdef USE_POS_HOLD_MODE

#include "flight/pos_hold.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "pos_hold.h"

PG_REGISTER_WITH_RESET_TEMPLATE(posHoldConfig_t, posHoldConfig, PG_POSHOLD_CONFIG, 0);

PG_RESET_TEMPLATE(posHoldConfig_t, posHoldConfig,
    .pos_hold_without_mag = false,     // position hold within this percentage stick deflection
    .pos_hold_deadband = 15, // deadband in percent of stick travel for roll and pitch. Must be non-zero, and exceeded, for target location to be changed with sticks
);
#endif
