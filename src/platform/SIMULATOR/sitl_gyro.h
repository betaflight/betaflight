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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

// Map a simulator IMU angular-velocity sample (rad/s) from the simulator's IMU
// sensor frame onto Betaflight's body gyro axes (roll, pitch, yaw), still rad/s.
//
// Betaflight's body frame is NWU (X=fwd, Y=left, Z=up) with right-hand rates:
// positive roll = right side down, positive pitch = nose down, positive yaw =
// counter-clockwise viewed from above.
//
// The simulators deliver the IMU rates in an FRD sensor frame (X=fwd, Y=right,
// Z=down). For Gazebo, the BetaflightPlugin reads the IMU *sensor* entity, whose
// pose is Rx(pi) relative to the FLU body link. Converting FRD to NWU keeps X
// and negates Y and Z:
//   Roll  = +wx_FRD -> keep X
//   Pitch = -wy_FRD -> negate Y
//   Yaw   = -wz_FRD -> negate Z
//
// The yaw sign applies to every bridge. Feeding +wz_FRD (as #15354 did for
// Gazebo) inverts the yaw feedback: with the default mixer the quad spins up
// after takeoff, and "yaw_motors_reversed" only masks it by also reversing the
// yaw stick (#16009).
static inline void sitlGyroBodyFromSim(const double rpy[3], double *roll, double *pitch, double *yaw)
{
    *roll  =  rpy[0];
    *pitch = -rpy[1];
    *yaw   = -rpy[2];
}
