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
    #include "pg/pg.h"
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

// Signed 32-bit Unix time runs out at 2038-01-19 03:14:07 UTC.
static dateTime_t boundaryDateTime(uint8_t seconds, uint16_t millis)
{
    dateTime_t dt;
    dt.year = 2038;
    dt.month = 1;
    dt.day = 19;
    dt.hours = 3;
    dt.minutes = 14;
    dt.seconds = seconds;
    dt.millis = millis;
    return dt;
}

static void expectDateTimeEq(const dateTime_t &expected, const dateTime_t &actual, const char *label)
{
    EXPECT_EQ(expected.year, actual.year) << label;
    EXPECT_EQ(expected.month, actual.month) << label;
    EXPECT_EQ(expected.day, actual.day) << label;
    EXPECT_EQ(expected.hours, actual.hours) << label;
    EXPECT_EQ(expected.minutes, actual.minutes) << label;
    EXPECT_EQ(expected.seconds, actual.seconds) << label;
    EXPECT_EQ(expected.millis, actual.millis) << label;
}

TEST_F(RtcTimeTest, RoundTripsAcrossThe2038Boundary)
{
    struct {
        const char *name;
        dateTime_t dt;
        int64_t unixSeconds;
    } cases[] = {
        { "one second before", boundaryDateTime(6, 250), 2147483646LL },
        { "at the boundary",   boundaryDateTime(7, 500), 2147483647LL },
        { "one second after",  boundaryDateTime(8, 750), 2147483648LL },
    };

    for (const auto &c : cases) {
        dateTime_t dt = c.dt;
        ASSERT_TRUE(rtcSetDateTime(&dt)) << c.name;

        rtcTime_t t;
        ASSERT_TRUE(rtcGet(&t)) << c.name;
        EXPECT_EQ(c.unixSeconds, rtcTimeGetSeconds(&t)) << c.name;
        EXPECT_EQ(c.dt.millis, rtcTimeGetMillis(&t)) << c.name;

        dateTime_t readBack;
        ASSERT_TRUE(rtcGetDateTime(&readBack)) << c.name;
        expectDateTimeEq(c.dt, readBack, c.name);
    }
}

TEST_F(RtcTimeTest, SecondsHelpersDoNotTruncateAboveInt32Max)
{
    const int64_t secs = 4102444799LL; // 2099-12-31 23:59:59 UTC
    rtcTime_t t = rtcTimeMake(secs, 999);

    EXPECT_EQ(secs, rtcTimeGetSeconds(&t));
    EXPECT_EQ(999, rtcTimeGetMillis(&t));
}

TEST_F(RtcTimeTest, RoundTripsEndOfSupportedRange)
{
    dateTime_t dt;
    dt.year = 2099;
    dt.month = 12;
    dt.day = 31;
    dt.hours = 23;
    dt.minutes = 59;
    dt.seconds = 59;
    dt.millis = 999;

    ASSERT_TRUE(rtcSetDateTime(&dt));

    rtcTime_t t;
    ASSERT_TRUE(rtcGet(&t));
    EXPECT_EQ(4102444799LL, rtcTimeGetSeconds(&t));

    dateTime_t readBack;
    ASSERT_TRUE(rtcGetDateTime(&readBack));
    expectDateTimeEq(dt, readBack, "2099-12-31");
}

TEST_F(RtcTimeTest, TimezoneOffsetCrossesThe2038Boundary)
{
    // 03:14:07 UTC at the boundary, shifted +60 minutes, lands after it.
    timeConfigMutable()->tz_offsetMinutes = 60;

    dateTime_t utc = boundaryDateTime(7, 123);
    dateTime_t local;
    dateTimeUTCToLocal(&utc, &local);

    dateTime_t expected = boundaryDateTime(7, 123);
    expected.hours = 4;
    expectDateTimeEq(expected, local, "+60 min across boundary");

    timeConfigMutable()->tz_offsetMinutes = 0;
}

// STUBS
extern "C" {
uint32_t millis(void) { return mockMillis; }
void persistentObjectWrite(int, uint32_t) {}
uint32_t persistentObjectRead(int) { return 0; }
}
