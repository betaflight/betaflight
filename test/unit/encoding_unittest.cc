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
    #include "common/encoding.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

typedef struct zigzagEncodingExpectation_s {
    int32_t input;
    uint32_t expected;
} zigzagEncodingExpectation_t;

typedef struct floatToIntEncodingExpectation_s {
    float input;
    uint32_t expected;
} floatToIntEncodingExpectation_t;

TEST(EncodingTest, ZigzagEncodingTest)
{
    // given
    zigzagEncodingExpectation_t expectations[] = {
        { 0, 0},
        {-1, 1},
        { 1, 2},
        {-2, 3},
        { 2, 4},

        { 2147483646, 4294967292},
        {-2147483647, 4294967293},
        { 2147483647, 4294967294},
        {-2147483648, 4294967295},
    };
    int expectationCount = sizeof(expectations) / sizeof(expectations[0]);

    // expect

    for (int i = 0; i < expectationCount; i++) {
        zigzagEncodingExpectation_t *expectation = &expectations[i];

        EXPECT_EQ(expectation->expected, zigzagEncode(expectation->input));
    }
}

TEST(EncodingTest, FloatToIntEncodingTest)
{
    // given
    floatToIntEncodingExpectation_t expectations[] = {
        {0.0, 0x00000000},
        {2.0, 0x40000000}, // Exponent should be in the top bits
        {4.5, 0x40900000}
    };
    int expectationCount = sizeof(expectations) / sizeof(expectations[0]);

    // expect

    for (int i = 0; i < expectationCount; i++) {
        floatToIntEncodingExpectation_t *expectation = &expectations[i];

        EXPECT_EQ(expectation->expected, castFloatBytesToInt(expectation->input));
    }
}

// STUBS

extern "C" {
}
