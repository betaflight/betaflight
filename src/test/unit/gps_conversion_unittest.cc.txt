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

#include <limits.h>

//#ifdef DEBUG_GPS_CONVERSION

extern "C" {
    #include "flight/gps_conversion.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// See http://en.wikipedia.org/wiki/Geographic_coordinate_conversion

TEST(GpsConversionTest, GPSCoordToDegrees_BadString)
{
    // expect
    uint32_t result = GPS_coord_to_degrees("diediedie");
    EXPECT_EQ(result, 0);
}

typedef struct gpsConversionExpectation_s {
    const char *coord;
    uint32_t degrees;
} gpsConversionExpectation_t;

TEST(GpsConversionTest, GPSCoordToDegrees_NMEA_Values)
{
    const gpsConversionExpectation_t gpsConversionExpectations[] = {
        {"0.0", 0},
        {"000.0", 0},
        {"00000.0000", 0},
        {"0.0001", 16}, // smallest value
        {"25599.9999", 2566666650UL}, // largest value
        {"25599.99999", 2566666650UL}, // too many fractional digits
        {"25699.9999", 16666650UL}, // overflowed without detection
        {"5128.3727", 514728783UL},
        {"5321.6802", 533613366UL},
        {"00630.3372", 65056200UL},
    };

    // expect

    uint8_t testIterationCount = sizeof(gpsConversionExpectations) / sizeof(gpsConversionExpectation_t);

    // expect

    for (uint8_t index = 0; index < testIterationCount; index ++) {
        const gpsConversionExpectation_t *expectation = &gpsConversionExpectations[index];
#ifdef DEBUG_GPS_CONVERSION
        printf("iteration: %d\n", index);
#endif
        uint32_t result = GPS_coord_to_degrees(expectation->coord);
        EXPECT_EQ(result, expectation->degrees);
    }
}

