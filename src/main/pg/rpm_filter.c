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

#ifdef USE_RPM_FILTER

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "rpm_filter.h"

PG_REGISTER_WITH_RESET_TEMPLATE(rpmFilterConfig_t, rpmFilterConfig, PG_RPM_FILTER_CONFIG, 5);

PG_RESET_TEMPLATE(rpmFilterConfig_t, rpmFilterConfig,
    .rpm_filter_harmonics = 3,
    .rpm_filter_min_hz = 100,
    .rpm_filter_fade_range_hz = 50,
    .rpm_filter_q = 500,
    .rpm_filter_lpf_hz = 150
);

#endif // USE_RPM_FILTER
