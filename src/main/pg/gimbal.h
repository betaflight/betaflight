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

#pragma once

#include "pg/pg.h"
#include "drivers/io_types.h"

typedef struct gimbalTrackConfig_s {
    int8_t gimbal_roll_rc_gain;
    int8_t gimbal_pitch_rc_thr_gain;
    int8_t gimbal_pitch_rc_low_gain;
    int8_t gimbal_pitch_rc_high_gain;
    int8_t gimbal_yaw_rc_gain;
    int8_t gimbal_roll_gain;
    int8_t gimbal_roll_offset;
    int8_t gimbal_roll_limit;
    int8_t gimbal_pitch_gain;
    int8_t gimbal_pitch_offset;
    int8_t gimbal_pitch_low_limit;
    int8_t gimbal_pitch_high_limit;
    int8_t gimbal_yaw_gain;
    int8_t gimbal_yaw_offset;
    int8_t gimbal_yaw_limit;
    int8_t gimbal_stabilisation;
    int8_t gimbal_sensitivity;
} gimbalTrackConfig_t;

PG_DECLARE(gimbalTrackConfig_t, gimbalTrackConfig);
