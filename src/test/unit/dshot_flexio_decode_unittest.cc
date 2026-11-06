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

// FlexIO DShot arithmetic: shifter bit pattern, timer compares, telemetry decode, rate search

#include <stdint.h>

extern "C" {
    #include "dshot_flexio.h"
}

#include "gtest/gtest.h"

namespace {

constexpr uint32_t FLEXIO_CLOCK_HZ = 108000000u; // PLL3 PFD2, startup/clocks_imxrt10xx.c

// GCR nibble to symbol
constexpr uint8_t GCR_ENCODE[16] = {
    0x19, 0x1b, 0x12, 0x13, 0x1d, 0x15, 0x16, 0x17,
    0x1a, 0x09, 0x0a, 0x0b, 0x1e, 0x0d, 0x0e, 0x0f
};

// receive shifter word for a 12 bit value: checksum, GCR, run length limit, invert
uint32_t shifterWordFor(uint16_t telemetry12)
{
    const uint8_t n3 = (telemetry12 >> 8) & 0xf;
    const uint8_t n2 = (telemetry12 >> 4) & 0xf;
    const uint8_t n1 = telemetry12 & 0xf;
    const uint8_t n0 = 0xf ^ n1 ^ n2 ^ n3;

    const uint32_t gcr = ((uint32_t)GCR_ENCODE[n3] << 15)
                       | ((uint32_t)GCR_ENCODE[n2] << 10)
                       | ((uint32_t)GCR_ENCODE[n1] << 5)
                       | (uint32_t)GCR_ENCODE[n0];

    // each line bit is the xor of every GCR bit at or above it
    uint32_t line = 0;
    uint32_t running = 0;
    for (int bit = 19; bit >= 0; bit--) {
        running ^= (gcr >> bit) & 1u;
        line |= running << bit;
    }

    return ~line;
}

// a well framed reply always ends in a one
uint32_t framingErrorWordFor(uint16_t telemetry12)
{
    return shifterWordFor(telemetry12) | 1u;
}

int8_t runTraining(const bool worksAt[32], bool *doneOut, unsigned roundLimit = 200)
{
    dshotFlexioTraining_t training;
    dshotFlexioTrainReset(&training);

    const uint32_t good = shifterWordFor(0xfff);
    const uint32_t bad = framingErrorWordFor(0xfff);

    for (unsigned round = 0; round < roundLimit && !training.done; round++) {
        const int index = training.offset - DSHOT_FLEXIO_TCMP_OFFSET_MIN;
        const uint32_t word = worksAt[index] ? good : bad;

        while (!dshotFlexioTrainStep(&training, word)) {
        }
    }

    *doneOut = training.done;
    return training.offset;
}

TEST(ImxrtDshotFlexioTest, ExpandFrameThreeTicksPerBit)
{
    // three ticks per bit: zero 0b001, one 0b011, first bit in the lowest three
    EXPECT_EQ(UINT64_C(0x249249249249), dshotFlexioExpandFrame(0x0000));
    EXPECT_EQ(UINT64_C(0x6db6db6db6db), dshotFlexioExpandFrame(0xffff));

    EXPECT_EQ(UINT64_C(0x24924924924b), dshotFlexioExpandFrame(0x8000));

    EXPECT_EQ(UINT64_C(0x649249249249), dshotFlexioExpandFrame(0x0001));
}

static uint64_t expandFrameOneBitAtATime(uint16_t packet)
{
    uint64_t expanded = 0;
    unsigned index = 0;

    for (unsigned mask = 0x8000; mask != 0; mask >>= 1) {
        expanded |= (uint64_t)((packet & mask) ? 0x3u : 0x1u) << index;
        index += 3;
    }

    return expanded;
}

TEST(ImxrtDshotFlexioTest, ExpandFrameMatchesTheBitLoopEverywhere)
{
    for (uint32_t packet = 0; packet <= 0xffffu; packet++) {
        ASSERT_EQ(expandFrameOneBitAtATime((uint16_t)packet),
                  dshotFlexioExpandFrame((uint16_t)packet)) << "packet " << packet;
    }
}

TEST(ImxrtDshotFlexioTest, ExpandFrameSplitsIntoTwoShifterLoads)
{
    // low 24 bits first, high 24 from the interrupt
    const uint16_t packet = 0x1234;
    const uint64_t expanded = dshotFlexioExpandFrame(packet);
    const uint32_t firstHalf = (uint32_t)(expanded & 0xffffffu);
    const uint32_t secondHalf = (uint32_t)(expanded >> 24);

    EXPECT_EQ(0u, secondHalf >> 24);
    EXPECT_EQ(expanded, ((uint64_t)secondHalf << 24) | firstHalf);
}

TEST(ImxrtDshotFlexioTest, TransmitCompareHoldsBitCountAndDivider)
{
    // high byte: (value + 1) / 2 shift clocks per word
    // low byte: shift clock = FlexIO clock / ((value + 1) * 2) = 3 x bit rate
    const uint32_t dshot600 = dshotFlexioTxTimerCompare(FLEXIO_CLOCK_HZ, 600000);
    EXPECT_EQ(0x2f1du, dshot600);
    EXPECT_EQ(24u, ((dshot600 >> 8) + 1) / 2);
    EXPECT_EQ(600000u * 3u, FLEXIO_CLOCK_HZ / (((dshot600 & 0xff) + 1) * 2));

    const uint32_t dshot300 = dshotFlexioTxTimerCompare(FLEXIO_CLOCK_HZ, 300000);
    EXPECT_EQ(0x2f3bu, dshot300);
    EXPECT_EQ(300000u * 3u, FLEXIO_CLOCK_HZ / (((dshot300 & 0xff) + 1) * 2));

    const uint32_t dshot150 = dshotFlexioTxTimerCompare(FLEXIO_CLOCK_HZ, 150000);
    EXPECT_EQ(0x2f77u, dshot150);
    EXPECT_EQ(150000u * 3u, FLEXIO_CLOCK_HZ / (((dshot150 & 0xff) + 1) * 2));
}

TEST(ImxrtDshotFlexioTest, ReceiveCompareHoldsBitCountAndOffset)
{
    // reply is 21 bits at 5/4 of the bit rate
    const uint32_t base = dshotFlexioRxTimerCompare(FLEXIO_CLOCK_HZ, 600000, 0);
    EXPECT_EQ(0x2948u, base);
    EXPECT_EQ(21u, ((base >> 8) + 1) / 2);

    // offset -1 gives exactly 5/4 of the bit rate and must be in the swept range
    const uint32_t exact = dshotFlexioRxTimerCompare(FLEXIO_CLOCK_HZ, 600000, -1);
    EXPECT_EQ(600000u * 5u / 4u, FLEXIO_CLOCK_HZ / (((exact & 0xff) + 1) * 2));
    EXPECT_GT(-1, DSHOT_FLEXIO_TCMP_OFFSET_MIN);
    EXPECT_LT(-1, DSHOT_FLEXIO_TCMP_OFFSET_MAX);

    // the offset moves only the divider
    const uint32_t low = dshotFlexioRxTimerCompare(FLEXIO_CLOCK_HZ, 600000,
                                                   DSHOT_FLEXIO_TCMP_OFFSET_MIN);
    const uint32_t high = dshotFlexioRxTimerCompare(FLEXIO_CLOCK_HZ, 600000, 14);
    EXPECT_EQ(0x2938u, low);
    EXPECT_EQ(0x2956u, high);
}

TEST(ImxrtDshotFlexioTest, ReceiveCompareAtDshot300)
{
    // 108 MHz / (300 kHz * 5 / 4) / 2 = 144 = 0x90, and 21 bits give 0x29
    EXPECT_EQ(0x2990u, dshotFlexioRxTimerCompare(FLEXIO_CLOCK_HZ, 300000, 0));
}

TEST(ImxrtDshotFlexioTest, ReceiveDividerHasToFitTheCompareField)
{
    // DShot600 needs 72, DShot300 144
    EXPECT_TRUE(dshotFlexioRxDividerFits(FLEXIO_CLOCK_HZ, 600000));
    EXPECT_TRUE(dshotFlexioRxDividerFits(FLEXIO_CLOCK_HZ, 300000));

    // DShot150 needs 288, truncated to 32
    EXPECT_FALSE(dshotFlexioRxDividerFits(FLEXIO_CLOCK_HZ, 150000));
    EXPECT_EQ(32u, dshotFlexioRxTimerCompare(FLEXIO_CLOCK_HZ, 150000, 0) & 0xffu);

    // 90 MHz: DShot150 divider 240, largest offset lands on 255
    EXPECT_TRUE(dshotFlexioRxDividerFits(90000000u, 150000));
    EXPECT_FALSE(dshotFlexioRxDividerFits(90375000u, 150000));

    // most negative offset would run past zero
    EXPECT_TRUE(dshotFlexioRxDividerFits(24000000u, 600000));
    EXPECT_FALSE(dshotFlexioRxDividerFits(22500000u, 600000));
}

// a reply counts when it decodes, whatever its value; 0x0fff is a stopped motor
TEST(ImxrtDshotFlexioTest, TrainingJudgesAReplyByItsDecodeNotByOneFixedValue)
{
    const uint16_t payloads[] = { 0x0fff, 0x0100, 0x02a5, 0x0000 };

    for (unsigned i = 0; i < sizeof(payloads) / sizeof(payloads[0]); i++) {
        SCOPED_TRACE(testing::Message() << "payload " << payloads[i]);

        dshotFlexioTraining_t training;
        dshotFlexioTrainReset(&training);

        for (unsigned frame = 0; frame < DSHOT_FLEXIO_TRAIN_FRAMES; frame++) {
            const bool roundEnded = dshotFlexioTrainStep(&training, shifterWordFor(payloads[i]));
            EXPECT_EQ(frame == DSHOT_FLEXIO_TRAIN_FRAMES - 1, roundEnded);
        }

        // first offset of the sweep is the lowest mask bit
        EXPECT_EQ(1u, training.passMask);
    }

    EXPECT_EQ(0xfffu, dshotFlexioDecodeTelemetry(shifterWordFor(0xfff)));
}

TEST(ImxrtDshotFlexioTest, DecodesEveryTelemetryValue)
{
    for (uint32_t value = 0; value <= 0xfff; value++) {
        EXPECT_EQ(value, dshotFlexioDecodeTelemetry(shifterWordFor(value))) << "value " << value;
    }
}

// Shifter words worked out by hand; the line starts low and toggles on every GCR one bit.
TEST(ImxrtDshotFlexioTest, DecodesKnownFrames)
{
    // eRPM 0x5f4 (exponent 2, period 500): checksum ~(5 ^ f ^ 4) = 1, so nibbles 5 f 4 1;
    // GCR 10101 01111 11101 11011; line 1100 1101 0101 0010 1101 = 0xcd52d; inverted 0xfff32ad2
    EXPECT_EQ(0x5f4u, dshotFlexioDecodeTelemetry(0xfff32ad2u));
    EXPECT_EQ(0xfff32ad2u, shifterWordFor(0x5f4));

    // EDT temperature 0x22a (42 C): checksum ~(2 ^ 2 ^ a) = 5, so nibbles 2 2 a 5;
    // GCR 10010 10010 01010 10101; line 1110 0111 0001 1001 1001 = 0xe7199; inverted 0xfff18e66
    EXPECT_EQ(0x22au, dshotFlexioDecodeTelemetry(0xfff18e66u));
    EXPECT_EQ(0xfff18e66u, shifterWordFor(0x22a));
}

TEST(ImxrtDshotFlexioTest, RejectsFramingError)
{
    const uint32_t good = shifterWordFor(0x0fff);
    EXPECT_EQ(DSHOT_FLEXIO_DECODE_INVALID, dshotFlexioDecodeTelemetry(good | 1u));
}

TEST(ImxrtDshotFlexioTest, RejectsCorruptFrame)
{
    const uint16_t value = 0x2a5;
    const uint32_t good = shifterWordFor(value);

    for (unsigned bit = 0; bit < 20; bit++) {
        EXPECT_EQ(DSHOT_FLEXIO_DECODE_INVALID, dshotFlexioDecodeTelemetry(good ^ (1u << bit)))
            << "single bit error at " << bit << " was not caught";
    }
}

TEST(ImxrtDshotFlexioTest, RejectsBadChecksum)
{
    // valid GCR symbols, wrong checksum nibble
    const uint16_t value = 0x123;
    const uint8_t n3 = (value >> 8) & 0xf;
    const uint8_t n2 = (value >> 4) & 0xf;
    const uint8_t n1 = value & 0xf;
    const uint8_t wrong = (0xf ^ n1 ^ n2 ^ n3) ^ 0x1;

    const uint32_t gcr = ((uint32_t)GCR_ENCODE[n3] << 15)
                       | ((uint32_t)GCR_ENCODE[n2] << 10)
                       | ((uint32_t)GCR_ENCODE[n1] << 5)
                       | (uint32_t)GCR_ENCODE[wrong];

    uint32_t line = 0;
    uint32_t running = 0;
    for (int bit = 19; bit >= 0; bit--) {
        running ^= (gcr >> bit) & 1u;
        line |= running << bit;
    }

    EXPECT_EQ(DSHOT_FLEXIO_DECODE_INVALID, dshotFlexioDecodeTelemetry(~line));
}

TEST(ImxrtDshotFlexioTest, TrainingPicksTheMiddleOfWhatWorked)
{
    bool worksAt[32] = { false };
    for (int offset = -6; offset <= 2; offset++) {
        worksAt[offset - DSHOT_FLEXIO_TCMP_OFFSET_MIN] = true;
    }

    bool done = false;
    const int8_t chosen = runTraining(worksAt, &done);

    EXPECT_TRUE(done);
    EXPECT_EQ(-2, chosen);
}

TEST(ImxrtDshotFlexioTest, TrainingPicksTheMiddleOfAWindowAtTheEnd)
{
    bool worksAt[32] = { false };
    for (int offset = 8; offset <= 14; offset++) {
        worksAt[offset - DSHOT_FLEXIO_TCMP_OFFSET_MIN] = true;
    }

    bool done = false;
    const int8_t chosen = runTraining(worksAt, &done);

    EXPECT_TRUE(done);
    EXPECT_EQ(11, chosen);
}

TEST(ImxrtDshotFlexioTest, TrainingTriesTheHighestOffset)
{
    bool worksAt[32] = { false };
    worksAt[DSHOT_FLEXIO_TCMP_OFFSET_MAX - DSHOT_FLEXIO_TCMP_OFFSET_MIN] = true;

    bool done = false;
    const int8_t chosen = runTraining(worksAt, &done);

    EXPECT_TRUE(done);
    EXPECT_EQ(DSHOT_FLEXIO_TCMP_OFFSET_MAX, chosen);
}

TEST(ImxrtDshotFlexioTest, TrainingKeepsSweepingWhenNothingWorks)
{
    bool worksAt[32] = { false };

    bool done = false;
    const int8_t chosen = runTraining(worksAt, &done, 64);

    EXPECT_FALSE(done);
    EXPECT_GE(chosen, DSHOT_FLEXIO_TCMP_OFFSET_MIN);
    EXPECT_LE(chosen, DSHOT_FLEXIO_TCMP_OFFSET_MAX);
}

TEST(ImxrtDshotFlexioTest, TrainingWrapsAroundAfterAFailedSweep)
{
    dshotFlexioTraining_t training;
    dshotFlexioTrainReset(&training);

    // a framing error ends the round at once, so one word per offset
    const uint32_t bad = framingErrorWordFor(0xfff);
    for (int offset = DSHOT_FLEXIO_TCMP_OFFSET_MIN; offset <= DSHOT_FLEXIO_TCMP_OFFSET_MAX; offset++) {
        EXPECT_EQ(offset, training.offset);
        EXPECT_TRUE(dshotFlexioTrainStep(&training, bad));
    }

    EXPECT_EQ(DSHOT_FLEXIO_TCMP_OFFSET_MIN, training.offset);
    EXPECT_FALSE(training.done);
    EXPECT_EQ(0u, training.passMask);
}

TEST(ImxrtDshotFlexioTest, TrainingNeedsNearlyEveryFrameToBeGood)
{
    dshotFlexioTraining_t training;
    dshotFlexioTrainReset(&training);

    const uint32_t good = shifterWordFor(0xfff);
    // framed but not a GCR symbol: fails a pass without ending the round
    const uint32_t wrong = ~0x1u;

    for (int offset = DSHOT_FLEXIO_TCMP_OFFSET_MIN; offset <= DSHOT_FLEXIO_TCMP_OFFSET_MAX; offset++) {
        const unsigned passes = (offset == 0) ? DSHOT_FLEXIO_TRAIN_PASSES
                              : (offset == 1) ? DSHOT_FLEXIO_TRAIN_PASSES - 1
                              : 0;

        for (unsigned frame = 0; frame < DSHOT_FLEXIO_TRAIN_FRAMES; frame++) {
            const bool roundEnded = dshotFlexioTrainStep(&training, frame < passes ? good : wrong);
            EXPECT_EQ(frame == DSHOT_FLEXIO_TRAIN_FRAMES - 1, roundEnded);
        }
    }

    EXPECT_TRUE(training.done);
    EXPECT_EQ(0, training.offset);
}

TEST(ImxrtDshotFlexioTest, TrainingConvergesOnASpinningMotor)
{
    dshotFlexioTraining_t training;
    dshotFlexioTrainReset(&training);

    const uint32_t framingError = framingErrorWordFor(0xfff);
    uint16_t spinning = 0;

    for (unsigned round = 0; round < 64 && !training.done; round++) {
        // offsets -4 to 0 work and never see the stopped motor value
        const bool works = (training.offset >= -4) && (training.offset <= 0);

        for (;;) {
            const uint32_t word = works ? shifterWordFor(0x100 + (spinning++ & 0x7f)) : framingError;
            if (dshotFlexioTrainStep(&training, word)) {
                break;
            }
        }
    }

    EXPECT_TRUE(training.done);
    EXPECT_EQ(-2, training.offset);
}

TEST(ImxrtDshotFlexioTest, LrintfRoundsHalfwayValuesToEven)
{
    // the target uses vcvtr (FPSCR round to nearest, ties to even), the host a fallback
    static const struct { float value; int32_t expected; } cases[] = {
        {  0.0f,     0 },
        {  0.4999f,  0 },
        {  0.5f,     0 },
        {  0.5001f,  1 },
        {  1.5f,     2 },
        {  2.5f,     2 },
        {  3.5f,     4 },
        { -0.5f,     0 },
        { -1.5f,    -2 },
        { -2.5f,    -2 },
        // throttle values
        {  47.0f,   47 },
        {  1046.4f, 1046 },
        {  2047.5f, 2048 },
    };

    for (const auto &c : cases) {
        EXPECT_EQ(c.expected, dshotFlexioLrintf(c.value)) << "value " << c.value;
        EXPECT_EQ((int32_t)lrintf(c.value), dshotFlexioLrintf(c.value)) << "value " << c.value;
    }
}

} // namespace
