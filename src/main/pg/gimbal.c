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

#ifdef USE_GIMBAL

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "gimbal.h"

PG_REGISTER_WITH_RESET_TEMPLATE(gimbalTrackConfig_t, gimbalTrackConfig, PG_GIMBAL_TRACK_CONFIG, 0);

PG_RESET_TEMPLATE(gimbalTrackConfig_t, gimbalTrackConfig,
    // Default to full gain and no offset
    .gimbal_roll_rc_gain        = 40,
    .gimbal_pitch_rc_thr_gain   = 10,
    .gimbal_pitch_rc_low_gain   = 10,
    .gimbal_pitch_rc_high_gain  = -20,
    .gimbal_yaw_rc_gain         = 20,
    .gimbal_roll_gain           = 100,
    .gimbal_roll_offset         = 0,
    .gimbal_roll_limit          = 100,
    .gimbal_pitch_gain          = 50,
    .gimbal_pitch_offset        = -10,
    .gimbal_pitch_low_limit     = 100,
    .gimbal_pitch_high_limit    = 100,
    .gimbal_yaw_gain            = 50,
    .gimbal_yaw_offset          = 0,
    .gimbal_yaw_limit           = 100,
    .gimbal_stabilisation       = 0,
    .gimbal_sensitivity         = 15
);
#endif
