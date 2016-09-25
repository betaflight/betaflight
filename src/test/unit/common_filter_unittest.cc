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


TEST(NotchFilterTest, Initialise)
{
    // given
    biquadFilter_t filter;

    // when
    biquadFilterInitNotch(&filter, 100, 1000, 100, 70);

    // then
    EXPECT_FLOAT_EQ( 0.82364058f, filter.b0);
    EXPECT_FLOAT_EQ(-1.3326783f,  filter.b1);
    EXPECT_FLOAT_EQ( 0.82364058f, filter.b2);
    EXPECT_FLOAT_EQ(-1.3326783f,  filter.a1);
    EXPECT_FLOAT_EQ( 0.64728117f, filter.a2);

    // and
    EXPECT_FLOAT_EQ( 0.0f, filter.d1);
    EXPECT_FLOAT_EQ( 0.0f, filter.d2);
}


TEST(NotchFilterTest, ApplyZero)
{
    // given
    biquadFilter_t filter;
    biquadFilterInitNotch(&filter, 100, 1000, 100, 70);

    // when
    float result = biquadFilterApply(&filter, 0);

    // then
    EXPECT_FLOAT_EQ(0.0f, result);
}


TEST(NotchFilterTest, ApplySweepWithNoFiltering)
{
    // given
    biquadFilter_t filter;
    biquadFilterInitNotch(&filter, 100, 1000, 100, 100);

    float result;

    // when
    for (int i = 0; i <= 100; i++) {
        result = biquadFilterApply(&filter, i);
    }

    // then
    EXPECT_FLOAT_EQ(100.0f, result);
}

TEST(NotchFilterTest, ApplySweepWithDefaults)
{
    // given
    biquadFilter_t filter;
    biquadFilterInitNotch(&filter, 260, 1000, 260, 160);

    float result;

    // when
    for (int i = 0; i <= 100; i++) {
        result = biquadFilterApply(&filter, i);
    }

    // then
    EXPECT_FLOAT_EQ(99.525948f, result);
}
