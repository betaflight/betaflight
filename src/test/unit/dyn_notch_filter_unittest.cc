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

extern "C" {
#include "build/debug.h"
#include "sensors/gyro.h"
#include "flight/dyn_notch_filter.h"

uint8_t debugMode = 0;
int16_t debug[DEBUG16_VALUE_COUNT];
gyro_t gyro = {};

uint32_t micros(void) { return 0; }
uint8_t calculateThrottlePercentAbs(void) { return 0; }
}

#include "gtest/gtest.h"

TEST(DynNotchFilterUnittest, TestUpdateRateBoundary)
{
    // PG defaults: min_hz, max_hz, q, count
    const dynNotchConfig_t config = { 100, 600, 300, 3 };

    // 499us ~= 2004Hz: above the minimum update rate.
    dynNotchInit(&config, 499);
    EXPECT_TRUE(isDynNotchActive());

    // 500us = exactly 2000Hz: the regression case, must remain enabled.
    dynNotchInit(&config, 500);
    EXPECT_TRUE(isDynNotchActive());

    // 501us ~= 1996Hz: below the minimum update rate.
    dynNotchInit(&config, 501);
    EXPECT_FALSE(isDynNotchActive());

    // 0us: an invalid looptime must not enable the filter.
    dynNotchInit(&config, 0);
    EXPECT_FALSE(isDynNotchActive());
}
