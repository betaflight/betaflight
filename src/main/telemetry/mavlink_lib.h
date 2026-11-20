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

// Single include point for the MAVLink library. All users must include this
// header rather than "common/mavlink.h" so the library is configured the same
// way in every translation unit.

#define MAVLINK_COMM_NUM_BUFFERS 2

// Declare the MAVLink helpers here and define them once in mavlink_helpers.c.
// Otherwise each including file gets its own static copy of the helpers,
// including the message CRC table and the per-channel status (sequence
// numbers and parser state).
#define MAVLINK_SEPARATE_HELPERS

// The MAVLink library uses unnamed unions and packed members.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include "common/mavlink.h"
#pragma GCC diagnostic pop
