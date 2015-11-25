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
    #include "build_config.h"
    #include "drivers/sonar_hcsr04.h"
    #include "sensors/sonar.h"
    extern int32_t measurement;
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

TEST(SonarUnittest, TestDistance)
{
    // Check sonar pulse time converted correctly to cm
    const int returnMicroSecondsPerCm = 59;
    measurement =  0;
    EXPECT_EQ(hcsr04_get_distance(), 0);

    measurement =  returnMicroSecondsPerCm;
    EXPECT_EQ(hcsr04_get_distance(), 1);

    measurement =  10 * returnMicroSecondsPerCm;
    EXPECT_EQ(hcsr04_get_distance(), 10);

    measurement =  SONAR_MAX_RANGE_WITH_TILT * returnMicroSecondsPerCm;
    EXPECT_EQ(hcsr04_get_distance(), SONAR_MAX_RANGE_WITH_TILT);
}

TEST(SonarUnittest, TestAltitude)
{
    // Check distance not modified if no tilt
    EXPECT_EQ(sonarCalculateAltitude(0, 0), 0);
    EXPECT_EQ(sonarGetLatestAltitude(), 0);
    EXPECT_EQ(sonarCalculateAltitude(100, 0), 100);
    EXPECT_EQ(sonarGetLatestAltitude(), 100);

    // Check that out of range is returned if tilt is too large
    EXPECT_EQ(sonarCalculateAltitude(0, SONAR_MAX_TILT_ANGLE+1), SONAR_OUT_OF_RANGE);
    EXPECT_EQ(sonarGetLatestAltitude(), SONAR_OUT_OF_RANGE);

    // Check distance at various tilt angles
    // distance 400, 5 degrees of tilt
    EXPECT_EQ(sonarCalculateAltitude(400, 50), 377);
    EXPECT_EQ(sonarGetLatestAltitude(), 377);
    // distance 400, 10 degrees of tilt
    EXPECT_EQ(sonarCalculateAltitude(400, 100), 355);
    EXPECT_EQ(sonarGetLatestAltitude(), 355);
    // distance 400, 20 degrees of tilt
    EXPECT_EQ(sonarCalculateAltitude(400, 200), 311);
    EXPECT_EQ(sonarGetLatestAltitude(), 311);
    // distance 400, maximum tilt
    EXPECT_EQ(sonarCalculateAltitude(400, SONAR_MAX_TILT_ANGLE), 288);
    EXPECT_EQ(sonarGetLatestAltitude(), 288);
}

// STUBS
extern "C" {
void sensorsSet(uint32_t mask) {UNUSED(mask);}
void hcsr04_init(const sonarHardware_t *initialSonarHardware) {UNUSED(initialSonarHardware);}
void hcsr04_start_reading(void) {}
}

