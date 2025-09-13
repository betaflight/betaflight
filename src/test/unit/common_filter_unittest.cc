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

TEST(FilterUnittest, TestPt1FilterInit)
{
    pt1Filter_t filter;
    pt1FilterInitAlpha(&filter, 0.0f);
    EXPECT_EQ(0, filter.coeffs.c[0]);

    pt1FilterInitAlpha(&filter, 1.0f);
    EXPECT_EQ(1.0, filter.coeffs.c[0]);
}

TEST(FilterUnittest, TestPt1FilterGain)
{
    EXPECT_FLOAT_EQ(0.999949, rcFilterGain(100.0f, 31.25f));
    // handle cases over uint8_t boundary
    EXPECT_FLOAT_EQ(0.99998301, rcFilterGain(300.0f, 31.25f));
}

TEST(FilterUnittest, TestPt1FilterApply)
{
    pt1Filter_t filter;
    pt1FilterInitLPF(&filter, 100.0f, 31.25f);
    EXPECT_EQ(0, filter.state[0].s[0]);

    float y = pt1FilterApply(&filter, 1800.0f);
    EXPECT_FLOAT_EQ(1799.9083, y);

    y = pt1FilterApply(&filter, -1800.0f);
    EXPECT_FLOAT_EQ(-1799.8165, y);

    y = pt1FilterApply(&filter, -200.0f);
    EXPECT_FLOAT_EQ(-200.08142, y);
}

TEST(FilterUnittest, TestSlewFilterInit)
{
    slewFilter_t filter;

    slewFilterInit(&filter, 0.0f, 0.0f);
    EXPECT_EQ(0, filter.state);
    EXPECT_EQ(0, filter.slewLimit);
    EXPECT_EQ(0, filter.threshold);

    slewFilterInit(&filter, 1800.0f, 1900.0f);
    EXPECT_EQ(0, filter.state);
    EXPECT_EQ(1800, filter.slewLimit);
    EXPECT_EQ(1900, filter.threshold);
}

TEST(FilterUnittest, TestSlewFilter)
{
    slewFilter_t filter;
    slewFilterInit(&filter, 2000.0f, 1900.0f);
    EXPECT_EQ(0, filter.state);
    EXPECT_EQ(2000, filter.slewLimit);
    EXPECT_EQ(1900, filter.threshold);

    slewFilterApply(&filter, 1800.0f);
    EXPECT_EQ(1800, filter.state);
    slewFilterApply(&filter, -1800.0f);
    EXPECT_EQ(-1800, filter.state);
    slewFilterApply(&filter, -200.0f);
    EXPECT_EQ(-200, filter.state);

    slewFilterApply(&filter, 1900.0f);
    EXPECT_EQ(1900, filter.state);
    slewFilterApply(&filter, -2000.0f);
    EXPECT_EQ(1900, filter.state);
    slewFilterApply(&filter, -200.0f);
    EXPECT_EQ(1900, filter.state);
    slewFilterApply(&filter, 1800.0f);
    EXPECT_EQ(1800, filter.state);
    slewFilterApply(&filter, -200.0f);
    EXPECT_EQ(-200, filter.state);

    slewFilterApply(&filter, -1900.0f);
    EXPECT_EQ(-1900, filter.state);
    slewFilterApply(&filter, 2000.0f);
    EXPECT_EQ(-1900, filter.state);
    slewFilterApply(&filter, 200.0f);
    EXPECT_EQ(-1900, filter.state);
    slewFilterApply(&filter, -1800.0f);
    EXPECT_EQ(-1800, filter.state);
    slewFilterApply(&filter, 200.0f);
    EXPECT_EQ(200, filter.state);
}
