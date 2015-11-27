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
    EXPECT_GE(HCSR04_MAX_TILT_ANGLE_DECIDEGREES, 0);
    EXPECT_LE(HCSR04_MAX_TILT_ANGLE_DECIDEGREES, 450);
    // Check against gross errors in max range constants
    EXPECT_GT(HCSR04_MAX_RANGE_CM, 100);
    EXPECT_LE(SONAR_MAX_RANGE_ACCURACY_HIGH_CM, HCSR04_MAX_RANGE_CM);
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

    measurement =  HCSR04_MAX_RANGE_CM * returnMicroSecondsPerCm;
    EXPECT_EQ(hcsr04_get_distance(), HCSR04_MAX_RANGE_CM);
}

TEST(SonarUnittest, TestAltitude)
{
    // Check distance not modified if no tilt
    EXPECT_EQ(sonarCalculateAltitude(0, 0, 0), 0);
    EXPECT_EQ(sonarGetLatestAltitude(), 0);
    EXPECT_EQ(sonarCalculateAltitude(100, 0, 0), 100);
    EXPECT_EQ(sonarGetLatestAltitude(), 100);

    // Check that out of range is returned if tilt is too large
    EXPECT_EQ(sonarCalculateAltitude(0, HCSR04_MAX_TILT_ANGLE_DECIDEGREES+1, 0), SONAR_OUT_OF_RANGE);
    EXPECT_EQ(sonarGetLatestAltitude(), SONAR_OUT_OF_RANGE);

    // Check distance at various roll angles
    // distance 400, 5 degrees of roll
    EXPECT_EQ(sonarCalculateAltitude(400, 50, 0), 398);
    EXPECT_EQ(sonarGetLatestAltitude(), 398);
    // distance 400, 10 degrees of roll
    EXPECT_EQ(sonarCalculateAltitude(400, 100, 0), 393);
    EXPECT_EQ(sonarGetLatestAltitude(), 393);
    // distance 400, 20 degrees of roll
    EXPECT_EQ(sonarCalculateAltitude(400, 200, 0), 375);
    EXPECT_EQ(sonarGetLatestAltitude(), 375);
    // distance 400, maximum roll
    EXPECT_EQ(sonarCalculateAltitude(400, HCSR04_MAX_TILT_ANGLE_DECIDEGREES, 0), 369);
    EXPECT_EQ(sonarGetLatestAltitude(), 369);
}

typedef struct rollAndPitch_s {
    // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
    int16_t rollDeciDegrees;
    int16_t pitchDeciDegrees;
} rollAndPitch_t;

typedef struct inclinationAngleExpectations_s {
    rollAndPitch_t inclination;
    int16_t expected_angle;
} inclinationAngleExpectations_t;

TEST(SonarUnittest, TestCalculateTiltAngle)
{
    const int testCount = 9;
    inclinationAngleExpectations_t inclinationAngleExpectations[testCount] = {
        { { 0,  0}, 0},
        { { 1,  0}, 1},
        { { 0,  1}, 1},
        { { 0, -1}, 1},
        { {-1,  0}, 1},
        { {-1, -2}, 2},
        { {-2, -1}, 2},
        { { 1,  2}, 2},
        { { 2,  1}, 2}
    };

    rollAndPitch_t inclination = {0, 0};
    int tilt_angle = sonarCalculateTiltAngle(inclination.rollDeciDegrees, inclination.pitchDeciDegrees);
    EXPECT_EQ(tilt_angle, 0);

    for (int i = 0; i < testCount; i++) {
        inclinationAngleExpectations_t *expectation = &inclinationAngleExpectations[i];
        int result = sonarCalculateTiltAngle(expectation->inclination.rollDeciDegrees, expectation->inclination.pitchDeciDegrees);
        EXPECT_EQ(expectation->expected_angle, result);
    }
}


// STUBS
extern "C" {
void sensorsSet(uint32_t mask) {UNUSED(mask);}
void hcsr04_init(const sonarHardware_t *initialSonarHardware) {UNUSED(initialSonarHardware);}
void hcsr04_start_reading(void) {}
}

