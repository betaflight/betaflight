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

#include "unittest_macros.h"
#include "gtest/gtest.h"

extern "C" {
#include "sitl_gyro.h"
}

// Pins the FRD sensor frame -> Betaflight NWU body frame gyro mapping used for
// all simulator bridges. The yaw sign matters: with +wz_FRD the yaw loop has
// positive feedback and the quad spins up after takeoff in Gazebo (#16009).
TEST(SitlGyroUnittest, FrdToBodyMapping)
{
    const double rpy[3] = { 1.0, 2.0, 3.0 };
    double roll, pitch, yaw;

    sitlGyroBodyFromSim(rpy, &roll, &pitch, &yaw);

    EXPECT_DOUBLE_EQ(roll,   1.0);   // +wx -> keep X
    EXPECT_DOUBLE_EQ(pitch, -2.0);   // -wy -> negate Y
    EXPECT_DOUBLE_EQ(yaw,   -3.0);   // -wz -> negate Z
}

// A clockwise rotation viewed from above is +wz in the FRD sensor frame and must
// read as negative yaw (Betaflight yaw is positive counter-clockwise).
TEST(SitlGyroUnittest, ClockwiseYawIsNegative)
{
    const double rpy[3] = { 0.0, 0.0, 1.0 };
    double roll, pitch, yaw;

    sitlGyroBodyFromSim(rpy, &roll, &pitch, &yaw);

    EXPECT_DOUBLE_EQ(roll,  0.0);
    EXPECT_DOUBLE_EQ(pitch, 0.0);
    EXPECT_LT(yaw, 0.0);
}
