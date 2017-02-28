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

extern "C" {
    #include "common/time.h"
    #include "drivers/time.h"
    extern timeUs_t usTicks;
    extern volatile timeMs_t sysTickUptime;
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

SysTick_Type SysTickValue;
SysTick_Type *SysTick = &SysTickValue;

TEST(TimeUnittest, TestMillis)
{
    sysTickUptime = 0;
    EXPECT_EQ(0, millis());
    sysTickUptime = 1;
    EXPECT_EQ(1, millis());
}

TEST(TimeUnittest, TestMicros)
{
    usTicks = 168;
    SysTick->VAL = 1000 * usTicks;
    sysTickUptime = 0;
    EXPECT_EQ(0, micros());
    sysTickUptime = 1;
    EXPECT_EQ(1000, micros());
    // ULONG_MAX = 4294967295
    sysTickUptime = 429496; // ULONG_MAX / 1000;
    EXPECT_EQ(429496000, micros());
    sysTickUptime = 429497;
    EXPECT_EQ(429497000, micros());
    sysTickUptime = 500000;
    EXPECT_EQ(500000000, micros());

    sysTickUptime = 0;
    SysTick->VAL = 0;
    EXPECT_EQ(1000, micros());
    sysTickUptime = 1;
    EXPECT_EQ(2000, micros());
    // ULONG_MAX = 4294967295
    sysTickUptime = 429496; // ULONG_MAX / 1000;
    EXPECT_EQ(429497000, micros());
    sysTickUptime = 429497;
    EXPECT_EQ(429498000, micros());
    sysTickUptime = 500000;
    EXPECT_EQ(500001000, micros());
}
