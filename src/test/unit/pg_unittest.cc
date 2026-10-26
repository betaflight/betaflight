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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <limits.h>

extern "C" {
    #include <platform.h>
    #include "build/debug.h"
    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/motor.h"

    #include "flight/mixer.h"

//PG_DECLARE(motorConfig_t, motorConfig);

PG_REGISTER_WITH_RESET_TEMPLATE(motorConfig_t, motorConfig, PG_MOTOR_CONFIG, 1);

PG_RESET_TEMPLATE(motorConfig_t, motorConfig,
    .dev = {
        .motorPwmRate = 400,
        .motorProtocol = 0,
        .motorInversion = 0,
        .useContinuousUpdate = 0,
        .useBurstDshot = 0,
        .useDshotTelemetry = 0,
        .useDshotEdt = 0,
        .ioTags = {IO_TAG_NONE, IO_TAG_NONE, IO_TAG_NONE, IO_TAG_NONE},
        .motorTransportProtocol = 0,
        .useDshotBitbang = 0,
        .useDshotBitbangedTimer = 0,
        .motorOutputReordering = {0, 1, 2, 3},
    },
    .motorIdle = 0,
    .maxthrottle = 1850,
    .mincommand = 1000,
    .kv = 1000,
    .motorPoleCount = 14,
);

typedef struct testElement_s {
    uint16_t a;
    uint16_t b;
} testElement_t;

#define TEST_ARRAY_LENGTH 3

PG_DECLARE_ARRAY(testElement_t, TEST_ARRAY_LENGTH, testArray);
PG_REGISTER_ARRAY_WITH_RESET_FN(testElement_t, TEST_ARRAY_LENGTH, testArray, PG_RESERVED_FOR_TESTING_1, 0);

void pgResetFn_testArray(testElement_t *elements)
{
    for (int i = 0; i < TEST_ARRAY_LENGTH; i++) {
        elements[i].a = 10 + i;
        elements[i].b = 20 + i;
    }
}
}


#include "unittest_macros.h"
#include "gtest/gtest.h"

TEST(ParameterGroupsfTest, Test_pgResetAll)
{
    memset(motorConfigMutable(), 0, sizeof(motorConfig_t));
    pgResetAll();
    EXPECT_EQ(1850, motorConfig()->maxthrottle);
    EXPECT_EQ(1000, motorConfig()->mincommand);
    EXPECT_EQ(400, motorConfig()->dev.motorPwmRate);
}

TEST(ParameterGroupsfTest, Test_pgFind)
{
    memset(motorConfigMutable(), 0, sizeof(motorConfig_t));
    const pgRegistry_t *pgRegistry = pgFind(PG_MOTOR_CONFIG);
    pgReset(pgRegistry);
    EXPECT_EQ(1850, motorConfig()->maxthrottle);
    EXPECT_EQ(1000, motorConfig()->mincommand);
    EXPECT_EQ(400, motorConfig()->dev.motorPwmRate);

    motorConfig_t motorConfig2;
    memset(&motorConfig2, 0, sizeof(motorConfig_t));
    motorConfigMutable()->dev.motorPwmRate = 500;
    pgStore(pgRegistry, &motorConfig2, sizeof(motorConfig_t));
    EXPECT_EQ(1850, motorConfig2.maxthrottle);
    EXPECT_EQ(1000, motorConfig2.mincommand);
    EXPECT_EQ(500, motorConfig2.dev.motorPwmRate);

    motorConfig_t motorConfig3;
    memset(&motorConfig3, 0, sizeof(motorConfig_t));
    pgResetCopy(&motorConfig3, PG_MOTOR_CONFIG);
    EXPECT_EQ(1850, motorConfig3.maxthrottle);
    EXPECT_EQ(1000, motorConfig3.mincommand);
    EXPECT_EQ(400, motorConfig3.dev.motorPwmRate);
}

TEST(ParameterGroupsfTest, Test_pgLoadArrayRejectsOtherElementSize)
{
    const pgRegistry_t *pgRegistry = pgFind(PG_RESERVED_FOR_TESTING_1);
    ASSERT_NE(nullptr, pgRegistry);

    // a record written by a build whose element was one field shorter
    uint16_t stored[TEST_ARRAY_LENGTH] = {1, 2, 3};
    EXPECT_FALSE(pgLoad(pgRegistry, stored, sizeof(stored), pgVersion(pgRegistry)));
    for (int i = 0; i < TEST_ARRAY_LENGTH; i++) {
        EXPECT_EQ(10 + i, testArray(i)->a);
        EXPECT_EQ(20 + i, testArray(i)->b);
    }

    // and by one whose element was longer
    uint8_t longer[sizeof(testElement_t) * TEST_ARRAY_LENGTH + 2];
    memset(longer, 1, sizeof(longer));
    EXPECT_FALSE(pgLoad(pgRegistry, longer, sizeof(longer), pgVersion(pgRegistry)));
    EXPECT_EQ(10, testArray(0)->a);
}

TEST(ParameterGroupsfTest, Test_pgLoadArrayRoundTrip)
{
    const pgRegistry_t *pgRegistry = pgFind(PG_RESERVED_FOR_TESTING_1);
    ASSERT_NE(nullptr, pgRegistry);

    pgReset(pgRegistry);
    for (int i = 0; i < TEST_ARRAY_LENGTH; i++) {
        testArrayMutable(i)->a = 100 + i;
    }
    testElement_t stored[TEST_ARRAY_LENGTH];
    ASSERT_EQ(sizeof(stored), pgStore(pgRegistry, stored, sizeof(stored)));

    EXPECT_TRUE(pgLoad(pgRegistry, stored, sizeof(stored), pgVersion(pgRegistry)));
    EXPECT_EQ(0, memcmp(stored, testArray_array(), sizeof(stored)));
    for (int i = 0; i < TEST_ARRAY_LENGTH; i++) {
        EXPECT_EQ(100 + i, testArray(i)->a);
        EXPECT_EQ(20 + i, testArray(i)->b);
    }
}

TEST(ParameterGroupsfTest, Test_pgLoadStructStillLoadsShorterRecord)
{
    // a single struct grows by appending fields: the old prefix loads, the new tail keeps its default
    const pgRegistry_t *pgRegistry = pgFind(PG_MOTOR_CONFIG);
    ASSERT_NE(nullptr, pgRegistry);

    motorConfig_t stored;
    pgResetCopy(&stored, PG_MOTOR_CONFIG);
    stored.dev.motorPwmRate = 500;
    const int shorter = offsetof(motorConfig_t, motorPoleCount);

    EXPECT_TRUE(pgLoad(pgRegistry, &stored, shorter, pgVersion(pgRegistry)));
    EXPECT_EQ(500, motorConfig()->dev.motorPwmRate);
    EXPECT_EQ(14, motorConfig()->motorPoleCount);
}

// STUBS

extern "C" {
}
