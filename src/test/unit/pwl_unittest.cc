/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>

#include <limits.h>

#include <math.h>

extern "C" {
    #include "common/pwl.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

float xSquared(float x)
{
    return x * x;
}

PWL_DECLARE(pwlXSquared, 21, 0.0f, 10.0f);

TEST(PwlUnittest, TestXSquared21)
{
    pwlFill(&pwlXSquared, xSquared);

    EXPECT_EQ(pwlInterpolate(&pwlXSquared, -1.0f), 0.0f); // outside of X bounds on the left
    EXPECT_EQ(pwlInterpolate(&pwlXSquared, 11.0f), 100.0f); // outside of X bounds on the right
    EXPECT_EQ(pwlInterpolate(&pwlXSquared, 0.0f), 0.0f);
    EXPECT_EQ(pwlInterpolate(&pwlXSquared, 1.0f), 1.0f);
    EXPECT_EQ(pwlInterpolate(&pwlXSquared, 2.0f), 4.0f);
    EXPECT_EQ(pwlInterpolate(&pwlXSquared, 9.0f), 81.0f);
    EXPECT_EQ(pwlInterpolate(&pwlXSquared, 10.0f), 100.0f);

    float x = 0.0f;
    while (x<=10.0f) {        
        EXPECT_NEAR(pwlInterpolate(&pwlXSquared, x), xSquared(x), 0.1f);
        x+=0.1;
    }    
}


PWL_DECLARE(pwlXSquaredTwoPoints, 2, 1.0f, 5.0f);

TEST(PwlUnittest, TestXSquared2)
{
    pwlFill(&pwlXSquaredTwoPoints, xSquared);

    EXPECT_EQ(pwlInterpolate(&pwlXSquaredTwoPoints, -1.0f), 1.0f); // outside of X bounds on the left
    EXPECT_EQ(pwlInterpolate(&pwlXSquaredTwoPoints, 11.0f), 25.0f); // outside of X bounds on the right
    EXPECT_EQ(pwlInterpolate(&pwlXSquaredTwoPoints, 1.0f), 1.0f);
    EXPECT_EQ(pwlInterpolate(&pwlXSquaredTwoPoints, 5.0f), 25.0f);

    EXPECT_EQ(pwlInterpolate(&pwlXSquaredTwoPoints, 3.5f), 16.0f);
}
