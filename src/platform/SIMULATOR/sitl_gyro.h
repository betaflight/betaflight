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

#include <stdbool.h>

// Map a simulator IMU angular-velocity sample (rad/s) from the simulator's IMU
// sensor frame onto Betaflight's body gyro axes (roll, pitch, yaw), still rad/s.
//
// Gazebo (BetaflightPlugin): angular velocity is read from the IMU *sensor*
// entity, whose pose is Rx(pi) relative to the FLU body link, making the sensor
// frame effectively FRD (X=fwd, Y=right, Z=down):
//   Roll  = +wx_FRD (roll right)                               -> keep X
//   Pitch = -wy_FRD (BF positive = nose down, opposite to FRD) -> negate Y
//   Yaw   = +wz_FRD (CW viewed from above)                     -> keep Z
// Legacy bridges (X-Plane, RealFlight) deliver yaw with the opposite polarity
// and therefore require Z to be negated.
//
// The bridge is selected at compile time by ENABLE_GAZEBO_BRIDGE but is passed
// in as an argument so the convention lives in exactly one place (preventing the
// silent loss of the bridge split that caused regression #15294) and so the unit
// test can exercise both bridges.
static inline void sitlGyroBodyFromSim(const double rpy[3], bool gazeboBridge,
                                       double *roll, double *pitch, double *yaw)
{
    *roll  =  rpy[0];
    *pitch = -rpy[1];
    *yaw   =  gazeboBridge ? rpy[2] : -rpy[2];
}
