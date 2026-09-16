/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

extern "C" {
#include "fc/core.h"
#include "build/debug.h"
#include "sensors/gyro.h"
#include "drivers/time.h"
#include "flight/dyn_notch_filter.h"

uint8_t debugMode = 0;
int16_t debug[DEBUG16_VALUE_COUNT];
gyro_t gyro = {};
}

#include "gtest/gtest.h"

TEST(DynNotchFilterUnittest, TestUpdateRateBoundary)
{
    // 499us ~= 2004Hz: above the minimum update rate.
    EXPECT_TRUE(dynNotchUpdateRateSupported(499));

    // 500us = exactly 2000Hz: the regression case, must remain enabled.
    EXPECT_TRUE(dynNotchUpdateRateSupported(500));

    // 501us ~= 1996Hz: below the minimum update rate.
    EXPECT_FALSE(dynNotchUpdateRateSupported(501));
}
