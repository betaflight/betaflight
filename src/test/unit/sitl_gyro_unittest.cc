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

#include <stdbool.h>

#include "unittest_macros.h"
#include "gtest/gtest.h"

extern "C" {
#include "sitl_gyro.h"
}

// Regression guard for #15294: PR #15280 dropped the ENABLE_GAZEBO_BRIDGE split
// around the gyro Z assignment, silently inverting yaw on the (default) Gazebo
// build. sitlGyroBodyFromSim() now carries the bridge-specific sign, so these
// tests pin the yaw polarity for *both* bridges in a single binary.

// Gazebo's IMU sensor frame is FRD: roll keeps X, pitch negates Y, yaw keeps Z.
TEST(SitlGyroUnittest, GazeboBridgeMapping)
{
    const double rpy[3] = { 1.0, 2.0, 3.0 };
    double roll, pitch, yaw;

    sitlGyroBodyFromSim(rpy, true, &roll, &pitch, &yaw);

    EXPECT_DOUBLE_EQ(roll,   1.0);   // +wx -> keep X
    EXPECT_DOUBLE_EQ(pitch, -2.0);   // -wy -> negate Y
    EXPECT_DOUBLE_EQ(yaw,    3.0);   // +wz -> keep Z (the bit #15280 broke)
}

// Legacy bridges (X-Plane, RealFlight) deliver yaw with the opposite polarity.
TEST(SitlGyroUnittest, LegacyBridgeMapping)
{
    const double rpy[3] = { 1.0, 2.0, 3.0 };
    double roll, pitch, yaw;

    sitlGyroBodyFromSim(rpy, false, &roll, &pitch, &yaw);

    EXPECT_DOUBLE_EQ(roll,   1.0);   // +wx -> keep X
    EXPECT_DOUBLE_EQ(pitch, -2.0);   // -wy -> negate Y
    EXPECT_DOUBLE_EQ(yaw,   -3.0);   // -wz -> negate Z
}

// The defining property of the regression: a positive simulator yaw rate must
// stay positive under Gazebo, and the two bridges must disagree on yaw sign.
TEST(SitlGyroUnittest, YawSignDiffersByBridge)
{
    const double rpy[3] = { 0.0, 0.0, 1.0 };  // pure positive yaw rate
    double roll, pitch, gazeboYaw, legacyYaw;

    sitlGyroBodyFromSim(rpy, true,  &roll, &pitch, &gazeboYaw);
    sitlGyroBodyFromSim(rpy, false, &roll, &pitch, &legacyYaw);

    EXPECT_GT(gazeboYaw, 0.0);                 // Gazebo preserves yaw direction
    EXPECT_LT(legacyYaw, 0.0);                 // legacy inverts it
    EXPECT_DOUBLE_EQ(gazeboYaw, -legacyYaw);   // exact opposites
}
