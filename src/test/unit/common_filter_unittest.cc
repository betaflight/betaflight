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

TEST(FilterUnittest, TestFirFilterInit)
{
#define BUFLEN 4
    float buf[BUFLEN];
    firFilter_t filter;


    firFilterInit(&filter, buf, BUFLEN, NULL);

    EXPECT_EQ(buf, filter.buf);
    EXPECT_EQ(0, filter.coeffs);
    EXPECT_EQ(0, filter.movingSum);
    EXPECT_EQ(0, filter.index);
    EXPECT_EQ(0, filter.count);
    EXPECT_EQ(BUFLEN, filter.bufLength);
    EXPECT_EQ(BUFLEN, filter.coeffsLength);
}

TEST(FilterUnittest, TestFirFilterUpdateAverage)
{
#define BUFLEN 4
    float buf[BUFLEN];
    const float coeffs[BUFLEN] = {1.0f, 1.0f, 1.0f, 1.0f};
    firFilter_t filter;

    firFilterInit(&filter, buf, BUFLEN, coeffs);

    firFilterUpdateAverage(&filter, 2.0f);
    EXPECT_FLOAT_EQ(2.0f, filter.buf[0]);
    EXPECT_FLOAT_EQ(2.0f, filter.movingSum);
    EXPECT_EQ(1, filter.index);
    EXPECT_EQ(1, filter.count);
    EXPECT_EQ(2.0f, firFilterCalcMovingAverage(&filter));
    EXPECT_FLOAT_EQ(2.0f, firFilterCalcPartialAverage(&filter, 1));
    EXPECT_FLOAT_EQ(2.0f, firFilterApply(&filter));

    firFilterUpdateAverage(&filter, 3.0f);
    EXPECT_FLOAT_EQ(3.0f, filter.buf[1]);
    EXPECT_FLOAT_EQ(5.0f, filter.movingSum);
    EXPECT_EQ(2, filter.index);
    EXPECT_EQ(2, filter.count);
    EXPECT_EQ(2.5f, firFilterCalcMovingAverage(&filter));
    EXPECT_FLOAT_EQ(2.5f, firFilterCalcPartialAverage(&filter, 2));
    EXPECT_FLOAT_EQ(5.0f, firFilterApply(&filter));

    firFilterUpdateAverage(&filter, 4.0f);
    EXPECT_FLOAT_EQ(4.0f, filter.buf[2]);
    EXPECT_FLOAT_EQ(9.0f, filter.movingSum);
    EXPECT_EQ(3, filter.index);
    EXPECT_EQ(3, filter.count);
    EXPECT_EQ(3.0f, firFilterCalcMovingAverage(&filter));
    EXPECT_FLOAT_EQ(3.0f, firFilterCalcPartialAverage(&filter, 3));
    EXPECT_FLOAT_EQ(9.0f, firFilterApply(&filter));

    firFilterUpdateAverage(&filter, 5.0f);
    EXPECT_FLOAT_EQ(5.0f, filter.buf[3]);
    EXPECT_FLOAT_EQ(14.0f, filter.movingSum);
    EXPECT_EQ(0, filter.index);
    EXPECT_EQ(4, filter.count);
    EXPECT_EQ(3.5f, firFilterCalcMovingAverage(&filter));
    EXPECT_FLOAT_EQ(3.5f, firFilterCalcPartialAverage(&filter, 4));
    EXPECT_FLOAT_EQ(14.0f, firFilterApply(&filter));

    firFilterUpdateAverage(&filter, 6.0f);
    EXPECT_FLOAT_EQ(6.0f, filter.buf[0]);
    EXPECT_FLOAT_EQ(18.0f, filter.movingSum);
    EXPECT_EQ(1, filter.index);
    EXPECT_EQ(BUFLEN, filter.count);
    EXPECT_EQ(4.5f, firFilterCalcMovingAverage(&filter));
    EXPECT_FLOAT_EQ(4.5f, firFilterCalcPartialAverage(&filter, BUFLEN));
    EXPECT_FLOAT_EQ(18.0f, firFilterApply(&filter));

    firFilterUpdateAverage(&filter, 7.0f);
    EXPECT_FLOAT_EQ(7.0f, filter.buf[1]);
    EXPECT_FLOAT_EQ(22.0f, filter.movingSum);
    EXPECT_EQ(2, filter.index);
    EXPECT_EQ(BUFLEN, filter.count);
    EXPECT_EQ(5.5f, firFilterCalcMovingAverage(&filter));
    EXPECT_FLOAT_EQ(5.5f, firFilterCalcPartialAverage(&filter, BUFLEN));
    EXPECT_FLOAT_EQ(22.0f, firFilterApply(&filter));
}

TEST(FilterUnittest, TestFirFilterApply)
{
#define BUFLEN 4
    float buf[BUFLEN];
    firFilter_t filter;
    const float coeffs[BUFLEN] = {26.0f, 27.0f, 28.0f, 29.0f};

    float expected = 0.0f;
    firFilterInit(&filter, buf, BUFLEN, coeffs);

    firFilterUpdate(&filter, 2.0f);
    expected = 2.0f * 26.0f;
    EXPECT_FLOAT_EQ(expected, firFilterApply(&filter));

    firFilterUpdate(&filter, 3.0f);
    expected = 3.0f * 26.0f + 2.0 * 27.0;
    EXPECT_FLOAT_EQ(expected, firFilterApply(&filter));

    firFilterUpdate(&filter, 4.0f);
    expected = 4.0f * 26.0f + 3.0 * 27.0 + 2.0 * 28.0;
    EXPECT_FLOAT_EQ(expected, firFilterApply(&filter));

    firFilterUpdate(&filter, 5.0f);
    expected = 5.0f * 26.0f + 4.0 * 27.0 + 3.0 * 28.0 + 2.0f * 29.0f;
    EXPECT_FLOAT_EQ(expected, firFilterApply(&filter));

    firFilterUpdate(&filter, 6.0f);
    expected = 6.0f * 26.0f + 5.0 * 27.0 + 4.0 * 28.0 + 3.0f * 29.0f;
    EXPECT_FLOAT_EQ(expected, firFilterApply(&filter));

    firFilterUpdate(&filter, 7.0f);
    expected = 7.0f * 26.0f + 6.0 * 27.0 + 5.0 * 28.0 + 4.0f * 29.0f;
    EXPECT_FLOAT_EQ(expected, firFilterApply(&filter));
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
