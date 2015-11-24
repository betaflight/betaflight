/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>

extern "C" {
    #include "drivers/sonar_hcsr04.h"
    #include "sensors/sonar.h"
    #include "build_config.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

TEST(SonarUnittest, TestConstants)
{
    // SONAR_OUT_OF_RANGE must be negative
    EXPECT_LE(SONAR_OUT_OF_RANGE, 0);
    // Check reasonable values for maximum tilt
    EXPECT_GE(SONAR_MAX_TILT_ANGLE, 0);
    EXPECT_LE(SONAR_MAX_TILT_ANGLE, 450);
    // Check against gross errors in max range constants
    EXPECT_LE(SONAR_MAX_RANGE_WITH_TILT, SONAR_MAX_RANGE);
    EXPECT_LE(SONAR_MAX_RANGE_ACCURACY_HIGH, SONAR_MAX_RANGE);
}

