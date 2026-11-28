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

#include "gtest/gtest.h"

extern "C" {
#include "sitl_dshot.h"
}

TEST(SitlDshotUnittest, Throttle)
{
    bool reversed = false;

    EXPECT_FLOAT_EQ(0.0f, sitlDshotMotorSpeed(DSHOT_CMD_MOTOR_STOP, false, &reversed));
    EXPECT_FLOAT_EQ(0.0f, sitlDshotMotorSpeed(DSHOT_MIN_THROTTLE, false, &reversed));
    EXPECT_NEAR(0.5f, sitlDshotMotorSpeed(DSHOT_MIN_THROTTLE + DSHOT_RANGE / 2, false, &reversed), 0.001f);
    EXPECT_FLOAT_EQ(1.0f, sitlDshotMotorSpeed(DSHOT_MAX_THROTTLE, false, &reversed));
    EXPECT_FALSE(reversed);
}

TEST(SitlDshotUnittest, SpinDirectionCommands)
{
    bool reversed = false;

    EXPECT_FLOAT_EQ(0.0f, sitlDshotMotorSpeed(DSHOT_CMD_SPIN_DIRECTION_REVERSED, false, &reversed));
    EXPECT_TRUE(reversed);
    EXPECT_FLOAT_EQ(-1.0f, sitlDshotMotorSpeed(DSHOT_MAX_THROTTLE, false, &reversed));

    sitlDshotMotorSpeed(DSHOT_CMD_BEACON1, false, &reversed);
    EXPECT_TRUE(reversed);

    EXPECT_FLOAT_EQ(0.0f, sitlDshotMotorSpeed(DSHOT_CMD_SPIN_DIRECTION_NORMAL, false, &reversed));
    EXPECT_FALSE(reversed);
    EXPECT_FLOAT_EQ(1.0f, sitlDshotMotorSpeed(DSHOT_MAX_THROTTLE, false, &reversed));
}

TEST(SitlDshotUnittest, Throttle3D)
{
    bool reversed = false;

    EXPECT_FLOAT_EQ(0.0f, sitlDshotMotorSpeed(DSHOT_CMD_MOTOR_STOP, true, &reversed));
    EXPECT_FLOAT_EQ(-1.0f, sitlDshotMotorSpeed(DSHOT_3D_FORWARD_MIN_THROTTLE - 1, true, &reversed));
    EXPECT_FLOAT_EQ(0.0f, sitlDshotMotorSpeed(DSHOT_3D_FORWARD_MIN_THROTTLE, true, &reversed));
    EXPECT_FLOAT_EQ(1.0f, sitlDshotMotorSpeed(DSHOT_MAX_THROTTLE, true, &reversed));
}
