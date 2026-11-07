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

extern "C" {
    #include "platform.h"
    #include "common/time.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

static uint32_t mockMillis = 0;

static dateTime_t validDateTime(void)
{
    dateTime_t dt;
    dt.year = 2026;
    dt.month = 10;
    dt.day = 30;
    dt.hours = 12;
    dt.minutes = 34;
    dt.seconds = 56;
    dt.millis = 789;
    return dt;
}

class RtcTimeTest : public ::testing::Test {
protected:
    void SetUp() override {
        mockMillis = 0;
    }
};

TEST_F(RtcTimeTest, AcceptsValidDateTime)
{
    dateTime_t dt = validDateTime();

    EXPECT_TRUE(rtcSetDateTime(&dt));

    dateTime_t readBack;
    EXPECT_TRUE(rtcGetDateTime(&readBack));
    EXPECT_EQ(2026, readBack.year);
    EXPECT_EQ(10, readBack.month);
    EXPECT_EQ(30, readBack.day);
    EXPECT_EQ(12, readBack.hours);
    EXPECT_EQ(34, readBack.minutes);
    EXPECT_EQ(56, readBack.seconds);
    EXPECT_EQ(789, readBack.millis);
}

TEST_F(RtcTimeTest, RejectsOutOfRangeFields)
{
    // Each of these is a value a GPS sentence can carry that would index the
    // month/year tables in dateTimeToRtcTime() out of bounds, or produce an
    // impossible time.
    struct {
        const char *name;
        void (*mutate)(dateTime_t *);
    } cases[] = {
        { "month 0",      [](dateTime_t *dt) { dt->month = 0; } },
        { "month 13",     [](dateTime_t *dt) { dt->month = 13; } },
        { "month 99",     [](dateTime_t *dt) { dt->month = 99; } },
        { "day 0",        [](dateTime_t *dt) { dt->day = 0; } },
        { "day 32",       [](dateTime_t *dt) { dt->day = 32; } },
        { "year 1999",    [](dateTime_t *dt) { dt->year = 1999; } },
        { "year 2100",    [](dateTime_t *dt) { dt->year = 2100; } },
        { "Feb 30",       [](dateTime_t *dt) { dt->month = 2; dt->day = 30; } },
        { "Feb 29 2026",  [](dateTime_t *dt) { dt->month = 2; dt->day = 29; } },
        { "Apr 31",       [](dateTime_t *dt) { dt->month = 4; dt->day = 31; } },
        { "hours 24",     [](dateTime_t *dt) { dt->hours = 24; } },
        { "hours 99",     [](dateTime_t *dt) { dt->hours = 99; } },
        { "minutes 60",   [](dateTime_t *dt) { dt->minutes = 60; } },
        { "seconds 60",   [](dateTime_t *dt) { dt->seconds = 60; } },
        { "millis 1000",  [](dateTime_t *dt) { dt->millis = 1000; } },
    };

    for (const auto &c : cases) {
        dateTime_t dt = validDateTime();
        c.mutate(&dt);
        EXPECT_FALSE(rtcSetDateTime(&dt)) << c.name;
    }
}

TEST_F(RtcTimeTest, AcceptsLeapDay)
{
    dateTime_t dt = validDateTime();
    dt.year = 2028;
    dt.month = 2;
    dt.day = 29;

    EXPECT_TRUE(rtcSetDateTime(&dt));

    dateTime_t readBack;
    EXPECT_TRUE(rtcGetDateTime(&readBack));
    EXPECT_EQ(2028, readBack.year);
    EXPECT_EQ(2, readBack.month);
    EXPECT_EQ(29, readBack.day);
}

TEST_F(RtcTimeTest, RejectedDateLeavesStoredTimeUnchanged)
{
    dateTime_t stored = validDateTime();
    ASSERT_TRUE(rtcSetDateTime(&stored));

    dateTime_t invalid = validDateTime();
    invalid.month = 0;
    EXPECT_FALSE(rtcSetDateTime(&invalid));

    dateTime_t readBack;
    ASSERT_TRUE(rtcGetDateTime(&readBack));
    EXPECT_EQ(2026, readBack.year);
    EXPECT_EQ(10, readBack.month);
    EXPECT_EQ(30, readBack.day);
    EXPECT_EQ(12, readBack.hours);
    EXPECT_EQ(34, readBack.minutes);
    EXPECT_EQ(56, readBack.seconds);
    EXPECT_EQ(789, readBack.millis);
}

// STUBS
extern "C" {
uint32_t millis(void) { return mockMillis; }
void persistentObjectWrite(int, uint32_t) {}
uint32_t persistentObjectRead(int) { return 0; }
}
