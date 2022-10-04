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

#ifdef USE_DYN_NOTCH_FILTER

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "dyn_notch.h"

PG_REGISTER_WITH_RESET_TEMPLATE(dynNotchConfig_t, dynNotchConfig, PG_DYN_NOTCH_CONFIG, 1);

PG_RESET_TEMPLATE(dynNotchConfig_t, dynNotchConfig,
    .dyn_notch_min_hz = 100,
    .dyn_notch_max_hz = 600,
    .dyn_notch_q = 300,
    .dyn_notch_count = 3
);

#endif // USE_DYN_NOTCH_FILTER
