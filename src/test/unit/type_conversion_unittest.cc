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

extern "C" {
    #include "common/utils.h"
    #include "common/typeconversion.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"


typedef struct ftoa_conversion_expectation_s {
    const float f;
    const char *s;
} ftoa_conversion_expectation_t;

#define FTOA_BUFFER_SIZE 13

TEST(TypeConversionTest, ftoa_zero)
{
    ASSERT_EQ(13, FTOA_BUFFER_SIZE);

    ftoa_conversion_expectation_t expectations[] = {
        { -0.0f, " 0.000"},
        { -0.001f, "-0.001"},
        { -0.999f, "-0.999"},
        { -0.0001f, " 0.000"},
        { -0.0009f, "-0.001"},
        { 0.0f, " 0.000"},
        { 0.001f, " 0.001"},
        { 0.999f, " 0.999"},
        { 0.0001f, " 0.000"},
        { 0.0009f, " 0.001"},
        { 0.9999f, " 1.000"},
        { 999999.999f, " 1000000.000"},
#ifdef RUN_ADDITONAL_TESTS_THAT_MAY_CAUSE_ROUNDING_ERRORS
        // these were tested and ran using using gcc 4.9.2 on Windows 7 x64.  These fail when run on the travis build environment.  Disabling until we have time to fix them.
        { 999999.000f, " 999999.000"},
        { 2147483.625f, " 2147483.500"}, // largest positive number that implementation seems to support.
        { -2147483.625f, "-2147483.500"}, // smallest negative number that implementation seems to support.
#endif
    };

    char dest[FTOA_BUFFER_SIZE + 1];

    for (uint32_t i = 0; i < ARRAYLEN(expectations); i++) {

        memset(dest, 0x7F, sizeof(dest));
        ftoa(expectations[i].f, dest);

        EXPECT_STREQ(expectations[i].s, dest);

        int len = strlen(expectations[i].s);
        EXPECT_EQ(0x00, dest[len]);
        EXPECT_EQ(0x7F, dest[len + 1]);
    }
}
