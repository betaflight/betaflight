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

#pragma once

typedef enum {
    X = 0,
    Y,
    Z
} axis_e;

#define XYZ_AXIS_COUNT 3

// See http://en.wikipedia.org/wiki/Flight_dynamics
typedef enum {
    FD_ROLL = 0,
    FD_PITCH,
    FD_YAW
} flight_dynamics_index_t;

#define FLIGHT_DYNAMICS_INDEX_COUNT 3

typedef enum {
    AI_ROLL = 0,
    AI_PITCH
} angle_index_t;

#define RP_AXIS_COUNT 2
#define EF_AXIS_COUNT 2

#define GET_DIRECTION(isReversed) ((isReversed) ? -1 : 1)
