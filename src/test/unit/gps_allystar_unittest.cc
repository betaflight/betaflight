#include <stdint.h>
#include <string.h>

extern "C" {
#include "io/gps_allystar.h"
}

#include "gtest/gtest.h"

static void writeU32(uint8_t *dst, uint32_t value)
{
    for (unsigned i = 0; i < 4; i++) {
        dst[i] = value >> (8 * i);
    }
}

static void addChecksum(uint8_t *frame, unsigned payloadLength)
{
    uint8_t checksumA = 0;
    uint8_t checksumB = 0;
    for (unsigned i = 2; i < payloadLength + 6; i++) {
        checksumA += frame[i];
        checksumB += checksumA;
    }
    frame[payloadLength + 6] = checksumA;
    frame[payloadLength + 7] = checksumB;
}

static void makeVelned(uint8_t *frame)
{
    memset(frame, 0, 44);
    frame[0] = 0xf1;
    frame[1] = 0xd9;
    frame[2] = 0x01;
    frame[3] = 0x12;
    frame[4] = 36;
    writeU32(frame + 10, static_cast<uint32_t>(-123));
    writeU32(frame + 14, 456);
    writeU32(frame + 18, static_cast<uint32_t>(-78));
    writeU32(frame + 22, 500);
    writeU32(frame + 26, 470);
    writeU32(frame + 30, 9000000);
    writeU32(frame + 34, 17);
    addChecksum(frame, 36);
}

static void consumeFrame(allystarParser_t *parser, const uint8_t *frame, unsigned length, uint32_t nowMs)
{
    for (unsigned i = 0; i < length; i++) {
        allystarParserConsume(parser, frame[i], nowMs);
    }
}

TEST(GpsAllystar, ParsesVelnedAndReturnsFreshSampleOnce)
{
    allystarParser_t parser = {};
    allystarParserInit(&parser);
    uint8_t frame[44];
    makeVelned(frame);
    consumeFrame(&parser, frame, sizeof(frame), 100);

    allystarVelned_t result;
    ASSERT_TRUE(allystarTakeFresh(&parser, 101, &result));
    EXPECT_EQ(-123, result.velN);
    EXPECT_EQ(456, result.velE);
    EXPECT_EQ(-78, result.velD);
    EXPECT_EQ(500u, result.speed3d);
    EXPECT_EQ(470u, result.groundSpeed);
    EXPECT_EQ(9000000, result.heading);
    EXPECT_EQ(17u, result.sAcc);
    EXPECT_FALSE(allystarTakeFresh(&parser, 102, &result));
}

TEST(GpsAllystar, RejectsBadChecksumAndExpiredSample)
{
    allystarParser_t parser = {};
    allystarParserInit(&parser);
    uint8_t frame[44];
    makeVelned(frame);
    frame[42] ^= 1;
    consumeFrame(&parser, frame, sizeof(frame), 100);

    allystarVelned_t result;
    EXPECT_FALSE(allystarTakeFresh(&parser, 101, &result));

    makeVelned(frame);
    consumeFrame(&parser, frame, sizeof(frame), 200);
    EXPECT_FALSE(allystarTakeFresh(&parser, 451, &result));
}

TEST(GpsAllystar, TimesOutPartialFrameAndPassesNmeaBytesThrough)
{
    allystarParser_t parser = {};
    allystarParserInit(&parser);
    uint8_t frame[44];
    makeVelned(frame);
    for (unsigned i = 0; i < 10; i++) {
        allystarParserConsume(&parser, frame[i], 100);
    }
    EXPECT_FALSE(allystarParserConsume(&parser, '$', 151));

    consumeFrame(&parser, frame, sizeof(frame), 200);
    allystarVelned_t result;
    EXPECT_TRUE(allystarTakeFresh(&parser, 201, &result));
}

TEST(GpsAllystar, BuildsReceiverConfigurationForFiveAndTenHertz)
{
    uint8_t frame[ALLYSTAR_PWRCTL2_COMMAND_LENGTH];
    EXPECT_EQ(ALLYSTAR_PWRCTL2_COMMAND_LENGTH, allystarBuildPwrctl2(frame, 5));
    EXPECT_EQ(0xf1, frame[0]);
    EXPECT_EQ(0xd9, frame[1]);
    EXPECT_EQ(0x06, frame[2]);
    EXPECT_EQ(0x44, frame[3]);
    EXPECT_EQ(5, frame[10]);
    EXPECT_EQ(200, frame[14]);
    EXPECT_EQ(0, frame[15]);
    EXPECT_EQ(0x28, frame[22]);
    EXPECT_EQ(0x2e, frame[23]);
    uint8_t checksumA = 0;
    uint8_t checksumB = 0;
    for (unsigned i = 2; i < 22; i++) {
        checksumA += frame[i];
        checksumB += checksumA;
    }
    EXPECT_EQ(checksumA, frame[22]);
    EXPECT_EQ(checksumB, frame[23]);

    EXPECT_EQ(ALLYSTAR_PWRCTL2_COMMAND_LENGTH, allystarBuildPwrctl2(frame, 10));
    EXPECT_EQ(10, frame[10]);
    EXPECT_EQ(100, frame[14]);
    EXPECT_EQ(0, frame[15]);
    EXPECT_EQ(0xc9, frame[22]);
    EXPECT_EQ(0x4a, frame[23]);
    EXPECT_EQ(1, allystarClampRate(0));
    EXPECT_EQ(10, allystarClampRate(20));

    const uint8_t expected[] = {0xf1, 0xd9, 0x06, 0x01, 0x03, 0x00, 0x01, 0x12, 0x01, 0x1e, 0x67};
    EXPECT_EQ(0, memcmp(expected, allystarVelnedCommand(), sizeof(expected)));
}

TEST(GpsAllystar, RejectsOversizedPayloadAndRecoversAtNextFrame)
{
    allystarParser_t parser = {};
    allystarParserInit(&parser);
    uint8_t oversized[45] = {};
    oversized[0] = 0xf1;
    oversized[1] = 0xd9;
    oversized[2] = 0x01;
    oversized[3] = 0x20;
    oversized[4] = 37;
    oversized[6 + 5] = '$'; // Binary payload bytes must not leak to the NMEA parser.
    addChecksum(oversized, 37);
    for (unsigned i = 0; i < sizeof(oversized); i++) {
        EXPECT_TRUE(allystarParserConsume(&parser, oversized[i], 100));
    }

    uint8_t frame[44];
    makeVelned(frame);
    consumeFrame(&parser, frame, sizeof(frame), 200);
    allystarVelned_t result;
    EXPECT_TRUE(allystarTakeFresh(&parser, 201, &result));
}

TEST(GpsAllystar, AcceptsAFrameReceivedAt4800BaudWithoutLongInterbyteGaps)
{
    allystarParser_t parser = {};
    allystarParserInit(&parser);
    uint8_t frame[44];
    makeVelned(frame);
    for (unsigned i = 0; i < sizeof(frame); ++i) {
        EXPECT_TRUE(allystarParserConsume(&parser, frame[i], 100 + 3 * i));
    }
    allystarVelned_t result;
    ASSERT_TRUE(allystarTakeFresh(&parser, 230, &result));
    EXPECT_EQ(-123, result.velN);
}

TEST(GpsAllystar, RejectsImpossibleLengthWithoutSwallowingFollowingNmea)
{
    allystarParser_t parser = {};
    allystarParserInit(&parser);
    const uint8_t badHeader[] = { 0xf1, 0xd9, 0x01, 0x12, 0xff, 0xff };
    consumeFrame(&parser, badHeader, sizeof(badHeader), 100);
    EXPECT_FALSE(allystarParserConsume(&parser, '$', 101));
    uint8_t frame[44];
    makeVelned(frame);
    consumeFrame(&parser, frame, sizeof(frame), 102);
    allystarVelned_t result;
    EXPECT_TRUE(allystarTakeFresh(&parser, 103, &result));
}

static void makeTime(uint8_t *frame, int16_t leap = 18, uint8_t flags = 6, uint8_t navSys = 0, uint32_t refTow = 18000)
{
    memset(frame, 0, 24);
    frame[0] = 0xf1;
    frame[1] = 0xd9;
    frame[2] = 1;
    frame[3] = 5;
    frame[4] = 16;
    writeU32(frame + 10, refTow);
    frame[16] = static_cast<uint8_t>(leap);
    frame[17] = static_cast<uint16_t>(leap) >> 8;
    frame[6] = navSys;
    frame[7] = flags;
    addChecksum(frame, 16);
}

static void receiveVelocity(allystarParser_t *parser, uint32_t iTow, uint32_t nowMs)
{
    uint8_t frame[44];
    makeVelned(frame);
    writeU32(frame + 6, iTow);
    addChecksum(frame, 36);
    consumeFrame(parser, frame, sizeof(frame), nowMs);
}

static void receiveTime(allystarParser_t *parser, uint32_t nowMs, int16_t leap = 18,
    uint8_t flags = 6, uint8_t navSys = 0, uint32_t refTow = 18000)
{
    uint8_t frame[24];
    makeTime(frame, leap, flags, navSys, refTow);
    consumeFrame(parser, frame, sizeof(frame), nowMs);
}

TEST(GpsAllystar, MatchesExactUtcAndRetainsUnmatchedVelocity)
{
    allystarParser_t parser = {};
    allystarVelned_t result;
    receiveTime(&parser, 100);
    receiveVelocity(&parser, 123456789, 101);
    EXPECT_FALSE(allystarTakeForUtc(&parser, 102, 37038788, 1, &result));
    ASSERT_TRUE(allystarTakeForUtc(&parser, 103, 37038789, 1, &result));
    EXPECT_EQ(123456789u, result.iTow);
    EXPECT_EQ(-123, result.velN);
    EXPECT_FALSE(allystarTakeForUtc(&parser, 104, 37038789, 1, &result));
}

TEST(GpsAllystar, RequiresTimeReferenceAndRejectsInvalidReferenceImmediately)
{
    allystarParser_t parser = {};
    allystarVelned_t result;
    receiveVelocity(&parser, 18000, 100);
    EXPECT_FALSE(allystarTakeForUtc(&parser, 101, 0, 1, &result));
    receiveTime(&parser, 102);
    receiveVelocity(&parser, 18000, 103);
    ASSERT_TRUE(allystarTakeForUtc(&parser, 104, 0, 1, &result));

    for (unsigned invalid = 0; invalid < 8; invalid++) {
        receiveTime(&parser, 110);
        receiveVelocity(&parser, 18000, 111);
        switch (invalid) {
        case 0: receiveTime(&parser, 112, -1); break;
        case 1: receiveTime(&parser, 112, 101); break;
        case 2: receiveTime(&parser, 112, 18, 2); break;
        case 3: receiveTime(&parser, 112, 18, 4); break;
        case 4: receiveTime(&parser, 112, 18, 6, 1); break;
        case 5: receiveTime(&parser, 112, 18, 6, 0, 604800000); break;
        case 6: receiveTime(&parser, 112, 18, 0); break;
        case 7: receiveTime(&parser, 112, -256); break;
        }
        EXPECT_FALSE(allystarTakeForUtc(&parser, 113, 0, 1, &result));
    }
}

TEST(GpsAllystar, MatchesUtcMidnightAndGpsWeekWrap)
{
    allystarParser_t parser = {};
    allystarVelned_t result;
    receiveTime(&parser, 100);
    const uint32_t tow[] = { 17999, 18000, 0, 604799999, 604800000, UINT32_MAX };
    const uint32_t utc[] = { 86399999, 0, 86382000, 86381999, 0, 0 };
    for (unsigned i = 0; i < sizeof(tow) / sizeof(tow[0]); i++) {
        receiveVelocity(&parser, tow[i], 101);
        EXPECT_EQ(i < 4, allystarTakeForUtc(&parser, 102, utc[i], 1, &result));
    }
    receiveVelocity(&parser, 18000, 103);
    EXPECT_FALSE(allystarTakeForUtc(&parser, 104, 86400000, 1, &result));
    EXPECT_TRUE(allystarTakeForUtc(&parser, 105, 0, 1, &result));
}

TEST(GpsAllystar, BoundsReferenceAndVelocityAgeAcrossMonotonicWrap)
{
    allystarParser_t parser = {};
    allystarVelned_t result;
    receiveTime(&parser, 100);
    receiveVelocity(&parser, 18000, 59849);
    EXPECT_TRUE(allystarTakeForUtc(&parser, 60099, 0, 1, &result));
    receiveVelocity(&parser, 18000, 60100);
    EXPECT_FALSE(allystarTakeForUtc(&parser, 60100, 0, 1, &result));
    receiveTime(&parser, 60200);
    receiveVelocity(&parser, 18000, 60201);
    EXPECT_FALSE(allystarTakeForUtc(&parser, 60452, 0, 1, &result));
    receiveTime(&parser, UINT32_MAX - 100);
    receiveVelocity(&parser, 18000, UINT32_MAX - 50);
    EXPECT_TRUE(allystarTakeForUtc(&parser, 50, 0, 1, &result));
}

TEST(GpsAllystar, ChangedLeapOffsetDiscardsOldPendingVelocity)
{
    allystarParser_t parser = {};
    allystarVelned_t result;
    receiveTime(&parser, 100);
    receiveVelocity(&parser, 18000, 101);
    receiveTime(&parser, 102, 19);
    EXPECT_FALSE(allystarTakeForUtc(&parser, 103, 86399000, 1, &result));
    receiveVelocity(&parser, 19000, 104);
    EXPECT_TRUE(allystarTakeForUtc(&parser, 105, 0, 1, &result));
}

TEST(GpsAllystar, IgnoresTimeFramesWithBadChecksumOrWrongLength)
{
    allystarParser_t parser = {};
    allystarVelned_t result;
    uint8_t frame[24];
    makeTime(frame);
    frame[23] ^= 1;
    consumeFrame(&parser, frame, sizeof(frame), 100);
    receiveVelocity(&parser, 18000, 101);
    EXPECT_FALSE(allystarTakeForUtc(&parser, 102, 0, 1, &result));
    receiveTime(&parser, 103);
    makeTime(frame, 19);
    frame[23] ^= 1;
    consumeFrame(&parser, frame, sizeof(frame), 104);
    receiveVelocity(&parser, 18000, 105);
    EXPECT_TRUE(allystarTakeForUtc(&parser, 106, 0, 1, &result));
    makeTime(frame);
    frame[4] = 15;
    addChecksum(frame, 15);
    consumeFrame(&parser, frame, 23, 107);
    receiveVelocity(&parser, 18000, 108);
    EXPECT_FALSE(allystarTakeForUtc(&parser, 109, 0, 1, &result));
}

TEST(GpsAllystar, BuildsGpsTimePoll)
{
    const uint8_t expected[] = { 0xf1, 0xd9, 1, 5, 1, 0, 0, 7, 0x1c };
    EXPECT_EQ(sizeof(expected), ALLYSTAR_TIME_COMMAND_LENGTH);
    const uint8_t *command = allystarTimeCommand();
    EXPECT_EQ(0, memcmp(expected, command, sizeof(expected)));
    uint8_t checksumA = 0;
    uint8_t checksumB = 0;
    for (unsigned i = 2; i < ALLYSTAR_TIME_COMMAND_LENGTH - 2; i++) {
        checksumA += command[i];
        checksumB += checksumA;
    }
    EXPECT_EQ(checksumA, command[ALLYSTAR_TIME_COMMAND_LENGTH - 2]);
    EXPECT_EQ(checksumB, command[ALLYSTAR_TIME_COMMAND_LENGTH - 1]);
}

TEST(GpsAllystar, UnchangedTimeReferencePreservesPendingVelocity)
{
    allystarParser_t parser = {};
    allystarVelned_t result;
    receiveTime(&parser, 100);
    receiveVelocity(&parser, 18000, 101);
    receiveTime(&parser, 102, 18, 7, 0, 19000);
    EXPECT_TRUE(allystarTakeForUtc(&parser, 103, 0, 1, &result));
}

TEST(GpsAllystar, AcceptsZeroLeapOffsetAndReferenceAtLastWeekMillisecond)
{
    allystarParser_t parser = {};
    allystarVelned_t result;
    receiveTime(&parser, 100, 0, 6, 0, 604799999);
    receiveVelocity(&parser, 604799999, 101);
    EXPECT_TRUE(allystarTakeForUtc(&parser, 102, 86399999, 1, &result));
}

TEST(GpsAllystar, MatchesOnlyWithinNmeaDecimalQuantization)
{
    allystarParser_t parser = {};
    allystarVelned_t result;
    receiveTime(&parser, 100);
    receiveVelocity(&parser, 18333, 101);
    EXPECT_FALSE(allystarTakeForUtc(&parser, 102, 330, 1, &result));
    EXPECT_FALSE(allystarTakeForUtc(&parser, 102, 334, 1, &result));
    EXPECT_FALSE(allystarTakeForUtc(&parser, 102, 330, 1000, &result));
    EXPECT_FALSE(allystarTakeForUtc(&parser, 102, 330, 0, &result));
    EXPECT_FALSE(allystarTakeForUtc(&parser, 102, 430, 10, &result));
    EXPECT_TRUE(allystarTakeForUtc(&parser, 103, 330, 10, &result));
    receiveVelocity(&parser, 18333, 104);
    EXPECT_TRUE(allystarTakeForUtc(&parser, 105, 340, 10, &result));
    receiveVelocity(&parser, 18330, 106);
    EXPECT_TRUE(allystarTakeForUtc(&parser, 107, 333, 10, &result));
    receiveVelocity(&parser, 18330, 108);
    EXPECT_FALSE(allystarTakeForUtc(&parser, 109, 340, 10, &result));
    EXPECT_TRUE(allystarTakeForUtc(&parser, 110, 330, 1000, &result));
}

TEST(GpsAllystar, MatchesDecimalQuantizationAcrossUtcMidnight)
{
    allystarParser_t parser = {};
    allystarVelned_t result;
    receiveTime(&parser, 100);
    receiveVelocity(&parser, 17999, 101);
    EXPECT_TRUE(allystarTakeForUtc(&parser, 102, 0, 10, &result));
    receiveVelocity(&parser, 18000, 103);
    EXPECT_TRUE(allystarTakeForUtc(&parser, 104, 86399999, 10, &result));
}
