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

//
// alignment bitmasks
//

#define ALIGNMENT_ROTATION_WIDTH 2

#define ALIGNMENT_YAW_ROTATIONS_POSITION   (0 * ALIGNMENT_ROTATION_WIDTH)
#define ALIGNMENT_PITCH_ROTATIONS_POSITION (1 * ALIGNMENT_ROTATION_WIDTH)
#define ALIGNMENT_ROLL_ROTATIONS_POSITION  (2 * ALIGNMENT_ROTATION_WIDTH)

#define ALIGNMENT_YAW_ROTATIONS_MASK (0x3 << ALIGNMENT_YAW_ROTATIONS_POSITION)
#define ALIGNMENT_PITCH_ROTATIONS_MASK (0x3 << ALIGNMENT_PITCH_ROTATIONS_POSITION)
#define ALIGNMENT_ROLL_ROTATIONS_MASK (0x3 << ALIGNMENT_ROLL_ROTATIONS_POSITION)

#define ALIGNMENT_AXIS_ROTATIONS_MASK(axis) (0x3 << ((FD_YAW - axis) * ALIGNMENT_ROTATION_WIDTH))

#define ALIGNMENT_YAW_ROTATIONS(bits) ((bits & ALIGNMENT_YAW_ROTATIONS_MASK) >> ALIGNMENT_YAW_ROTATIONS_POSITION)
#define ALIGNMENT_PITCH_ROTATIONS(bits) ((bits & ALIGNMENT_PITCH_ROTATIONS_MASK) >> ALIGNMENT_PITCH_ROTATIONS_POSITION)
#define ALIGNMENT_ROLL_ROTATIONS(bits) ((bits & ALIGNMENT_ROLL_ROTATIONS_MASK) >> ALIGNMENT_ROLL_ROTATIONS_POSITION)

#define ALIGNMENT_AXIS_ROTATIONS(bits, axis) ((bits & ALIGNMENT_AXIS_ROTATIONS_MASK(axis)) >> ((FD_YAW - axis) * ALIGNMENT_ROTATION_WIDTH))

// [1:0] count of 90 degree rotations from 0
// [3:2] indicates 90 degree rotations on pitch
// [5:4] indicates 90 degree rotations on roll
#define ALIGNMENT_TO_BITMASK(alignment) ((alignment - CW0_DEG) & 0x3) | (((alignment - CW0_DEG) & 0x4) << 1)
