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

#include "drivers/io_types.h"

#include "pg/pg.h"

typedef struct headtrackerConfig_s {
    ioTag_t headtracker_ioTag;                 // Pin used to reset yaw on headtracker
    uint8_t headtracker_max_angle;             // angle for headtracker to report 100% as output; Leave at 0 to have 0 - 360 range
    uint8_t headtracker_yaw_shimmy_enable;     // when true: resets yaw headtracker with head shimmy (quick small shaking)
    uint8_t headtracker_yaw_shimmy_amplitude;  // amplitutde of required head shimmy (quick small shaking)
    uint8_t headtracker_yaw_shimmy_count;      // count of required head shimmy (quick small shaking)
} headtrackerConfig_t;

PG_DECLARE(headtrackerConfig_t, headtrackerConfig);
