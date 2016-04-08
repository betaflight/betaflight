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
#include <stdbool.h>

#include <limits.h>

#include <math.h>

extern "C" {
    #include "common/filter.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

TEST(FilterUnittest, TestFilterApplyAverage)
{
#define VALUE_COUNT 4
    int32_t valueState[VALUE_COUNT];
    for (int ii = 0; ii < VALUE_COUNT; ++ii) {
        valueState[ii] = 0;
    }
    int average = filterApplyAverage(8, VALUE_COUNT, valueState);
    EXPECT_EQ(2, average); // 8/4
    EXPECT_EQ(8, valueState[0]);
    EXPECT_EQ(0, valueState[1]);

    average = filterApplyAverage(16, VALUE_COUNT, valueState);
    EXPECT_EQ(6, average); // (8+16)/4
    EXPECT_EQ(16, valueState[0]);
    EXPECT_EQ(8, valueState[1]);


    average = filterApplyAverage(4, VALUE_COUNT, valueState);
    EXPECT_EQ(7, average); // (8+16+4)/4
    EXPECT_EQ(4, valueState[0]);
    EXPECT_EQ(16, valueState[1]);
    EXPECT_EQ(8, valueState[2]);

    average = filterApplyAverage(-12, VALUE_COUNT, valueState);
    EXPECT_EQ(4, average); // (8+16+4-12)/4
    EXPECT_EQ(-12, valueState[0]);
    EXPECT_EQ(4, valueState[1]);
    EXPECT_EQ(16, valueState[2]);
    EXPECT_EQ(8, valueState[3]);

    average = filterApplyAverage(48, VALUE_COUNT, valueState);
    EXPECT_EQ(14, average); // (16+4-12+48)/4
    EXPECT_EQ(48, valueState[0]);
    EXPECT_EQ(-12, valueState[1]);
    EXPECT_EQ(4, valueState[2]);
    EXPECT_EQ(16, valueState[3]);

    average = filterApplyAverage(4, VALUE_COUNT, valueState);
    EXPECT_EQ(11, average); // (4-12+48+4)/4
    EXPECT_EQ(4, valueState[0]);
    EXPECT_EQ(48, valueState[1]);
    EXPECT_EQ(-12, valueState[2]);
    EXPECT_EQ(4, valueState[3]);
}

TEST(FilterUnittest, TestFilterApplyAverage2)
{
#define VALUE_COUNT 4
    int32_t valueState[3][VALUE_COUNT];
    for (int ii = 0; ii < VALUE_COUNT; ++ii) {
        valueState[0][ii] = 0;
    }
    int average = filterApplyAverage(8, VALUE_COUNT, valueState[0]);
    EXPECT_EQ(2, average); // 8/4
    EXPECT_EQ(8, valueState[0][0]);
    EXPECT_EQ(0, valueState[0][1]);

    average = filterApplyAverage(16, VALUE_COUNT, valueState[0]);
    EXPECT_EQ(6, average); // (8+16)/4
    EXPECT_EQ(16, valueState[0][0]);
    EXPECT_EQ(8, valueState[0][1]);

}

TEST(FilterUnittest, TestFilterApplyAveragef)
{
#define VALUE_COUNT 4
    float valueState[VALUE_COUNT];
    for (int ii = 0; ii < VALUE_COUNT; ++ii) {
        valueState[ii] = 0;
    }
    int average = filterApplyAveragef(8, VALUE_COUNT, valueState);
    EXPECT_EQ(2, average); // 8/4
    EXPECT_EQ(8, valueState[0]);
    EXPECT_EQ(0, valueState[1]);

    average = filterApplyAveragef(16, VALUE_COUNT, valueState);
    EXPECT_EQ(6, average); // (8+16)/4
    EXPECT_EQ(16, valueState[0]);
    EXPECT_EQ(8, valueState[1]);


    average = filterApplyAveragef(4, VALUE_COUNT, valueState);
    EXPECT_EQ(7, average); // (8+16+4)/4
    EXPECT_EQ(4, valueState[0]);
    EXPECT_EQ(16, valueState[1]);
    EXPECT_EQ(8, valueState[2]);

    average = filterApplyAveragef(-12, VALUE_COUNT, valueState);
    EXPECT_EQ(4, average); // (8+16+4-12)/4
    EXPECT_EQ(-12, valueState[0]);
    EXPECT_EQ(4, valueState[1]);
    EXPECT_EQ(16, valueState[2]);
    EXPECT_EQ(8, valueState[3]);

    average = filterApplyAveragef(48, VALUE_COUNT, valueState);
    EXPECT_EQ(14, average); // (16+4-12+48)/4
    EXPECT_EQ(48, valueState[0]);
    EXPECT_EQ(-12, valueState[1]);
    EXPECT_EQ(4, valueState[2]);
    EXPECT_EQ(16, valueState[3]);

    average = filterApplyAveragef(4, VALUE_COUNT, valueState);
    EXPECT_EQ(11, average); // (4-12+48+4)/4
    EXPECT_EQ(4, valueState[0]);
    EXPECT_EQ(48, valueState[1]);
    EXPECT_EQ(-12, valueState[2]);
    EXPECT_EQ(4, valueState[3]);
}

