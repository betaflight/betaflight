#include <stdint.h>

#include <limits.h>
#include "io/gps_conversion.h"

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
    char *coord;
    uint32_t degrees;
} gpsConversionExpectation_t;

TEST(GpsConversionTest, GPSCoordToDegrees_NMEA_Values)
{
    gpsConversionExpectation_t gpsConversionExpectations[] = {
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
        gpsConversionExpectation_t *expectation = &gpsConversionExpectations[index];
        printf("iteration: %d\n", index);

        uint32_t result = GPS_coord_to_degrees(expectation->coord);
        EXPECT_EQ(result, expectation->degrees);
    }
}

