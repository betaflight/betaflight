
#include <stdint.h>
#include <stdbool.h>

#include <limits.h>
#include <algorithm>

extern "C" {
    #include <platform.h>

    #include "build/debug.h"
    #include "common/ringbuf.h"

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

uint8_t rbuffer[256];

TEST(RingBufferTest, U8ByteWriteRead)
{
    rbuf_t data;
    uint8_t *start = (uint8_t *)&rbuffer;
    uint8_t *end = (uint8_t *)&rbuffer + sizeof(rbuffer);
    rbuf_t *dataBuf = rbufInit(&data, start, end);
    uint8_t testData = 0xFF;
    rbufWriteU8(dataBuf, testData);
    EXPECT_EQ(start, dataBuf->readIdx);
    EXPECT_EQ(start+sizeof(testData), dataBuf->writeIdx);
    EXPECT_TRUE(rbufReadBytesRemaining(dataBuf));
    EXPECT_EQ(testData, rbufReadU8(dataBuf));
    EXPECT_FALSE(rbufReadBytesRemaining(dataBuf));
    EXPECT_EQ(dataBuf->writeIdx, dataBuf->readIdx);
}

TEST(RingBufferTest, U16ByteWriteRead)
{
    rbuf_t data;
    uint8_t *start = (uint8_t *)&rbuffer;
    uint8_t *end = (uint8_t *)&rbuffer + sizeof(rbuffer);
    rbuf_t *dataBuf = rbufInit(&data, start, end);
    uint16_t testData = 0xDEAD;
    rbufWriteU16(dataBuf, testData);
    EXPECT_EQ(start, dataBuf->readIdx);
    EXPECT_EQ(start+sizeof(testData), dataBuf->writeIdx);
    EXPECT_TRUE(rbufReadBytesRemaining(dataBuf));
    EXPECT_EQ(testData, rbufReadU16(dataBuf));
    EXPECT_FALSE(rbufReadBytesRemaining(dataBuf));
    EXPECT_EQ(dataBuf->writeIdx, dataBuf->readIdx);
}

TEST(RingBufferTest, U32ByteWriteRead)
{
    rbuf_t data;
    uint8_t *start = (uint8_t *)&rbuffer;
    uint8_t *end = (uint8_t *)&rbuffer + sizeof(rbuffer);
    rbuf_t *dataBuf = rbufInit(&data, start, end);
    uint32_t testData = 0xDEADBEEF;
    rbufWriteU32(dataBuf, testData);
    EXPECT_EQ(start, dataBuf->readIdx);
    EXPECT_EQ(start+sizeof(testData), dataBuf->writeIdx);
    EXPECT_TRUE(rbufReadBytesRemaining(dataBuf));
    EXPECT_EQ(testData, rbufReadU32(dataBuf));
    EXPECT_FALSE(rbufReadBytesRemaining(dataBuf));
    EXPECT_EQ(dataBuf->writeIdx, dataBuf->readIdx);
}

TEST(RingBufferTest, U16BigEndianByteWriteRead)
{
    rbuf_t data;
    uint8_t *start = (uint8_t *)&rbuffer;
    uint8_t *end = (uint8_t *)&rbuffer + sizeof(rbuffer);
    rbuf_t *dataBuf = rbufInit(&data, start, end);
    uint16_t testData = 0xDEAD;
    uint16_t compareData = 0xADDE;
    rbufWriteU16BigEndian(dataBuf, testData);
    EXPECT_EQ(start, dataBuf->readIdx);
    EXPECT_EQ(start+sizeof(testData), dataBuf->writeIdx);
    EXPECT_TRUE(rbufReadBytesRemaining(dataBuf));
    EXPECT_EQ(compareData, rbufReadU16(dataBuf));
    EXPECT_FALSE(rbufReadBytesRemaining(dataBuf));
    EXPECT_EQ(dataBuf->writeIdx, dataBuf->readIdx);
}

TEST(RingBufferTest, U32BigEndianByteWriteRead)
{
    rbuf_t data;
    uint8_t *start = (uint8_t *)&rbuffer;
    uint8_t *end = (uint8_t *)&rbuffer + sizeof(rbuffer);
    rbuf_t *dataBuf = rbufInit(&data, start, end);
    uint32_t testData = 0xDEADBEEF;
    uint32_t compareData= 0xEFBEADDE;
    rbufWriteU32BigEndian(dataBuf, testData);
    EXPECT_EQ(start, dataBuf->readIdx);
    EXPECT_EQ(start+sizeof(testData), dataBuf->writeIdx);
    EXPECT_TRUE(rbufReadBytesRemaining(dataBuf));
    EXPECT_EQ(compareData, rbufReadU32(dataBuf));
    EXPECT_FALSE(rbufReadBytesRemaining(dataBuf));
    EXPECT_EQ(dataBuf->writeIdx, dataBuf->readIdx);
}

TEST(RingBufferTest, DataWriteRead)
{
    rbuf_t data;
    uint8_t *start = (uint8_t *)&rbuffer;
    uint8_t *end = (uint8_t *)&rbuffer + sizeof(rbuffer);
    rbuf_t *dataBuf = rbufInit(&data, start, end);
    uint8_t testData[] = "abcdefghijklmnopqrstuvwxyz0123456789";
    uint8_t dataOut[sizeof(testData)];
    rbufWriteData(dataBuf, testData, sizeof(testData));
    EXPECT_EQ(start, dataBuf->readIdx);
    EXPECT_EQ(start+sizeof(testData), dataBuf->writeIdx);
    EXPECT_TRUE(rbufReadBytesRemaining(dataBuf));
    rbufReadData(dataBuf, &dataOut, sizeof(testData));
    EXPECT_EQ(*testData, *dataOut);
    EXPECT_FALSE(rbufReadBytesRemaining(dataBuf));
    EXPECT_EQ(dataBuf->writeIdx, dataBuf->readIdx);
}

TEST(RingBufferTest, StringWriteRead)
{
    rbuf_t data;
    uint8_t *start = (uint8_t *)&rbuffer;
    uint8_t *end = (uint8_t *)&rbuffer + sizeof(rbuffer);
    rbuf_t *dataBuf = rbufInit(&data, start, end);
    const char testData[] = "abcdefghijklmnopqrstuvwxyz0123456789";
    char dataOut[strlen(testData)];
    rbufWriteString(dataBuf, testData);
    EXPECT_EQ(start, dataBuf->readIdx);
    EXPECT_EQ(start+strlen(testData), dataBuf->writeIdx);
    EXPECT_TRUE(rbufReadBytesRemaining(dataBuf));
    rbufReadData(dataBuf, &dataOut, strlen(testData));
    EXPECT_EQ(*testData, *dataOut);
    EXPECT_FALSE(rbufReadBytesRemaining(dataBuf));
    EXPECT_EQ(dataBuf->writeIdx, dataBuf->readIdx);
}

TEST(RingBufferTest, ZeroTerminatedStringWriteRead)
{
    rbuf_t data;
    uint8_t *start = (uint8_t *)&rbuffer;
    uint8_t *end = (uint8_t *)&rbuffer + sizeof(rbuffer);
    rbuf_t *dataBuf = rbufInit(&data, start, end);
    const char testData[] = "abcdefghijklmnopqrstuvwxyz0123456789";
    char dataOut[sizeof(testData)];
    rbufWriteStringWithZeroTerminator(dataBuf, testData);
    EXPECT_EQ(start, dataBuf->readIdx);
    EXPECT_EQ(start+sizeof(testData), dataBuf->writeIdx);
    EXPECT_TRUE(rbufReadBytesRemaining(dataBuf));
    rbufReadData(dataBuf, &dataOut, sizeof(testData));
    EXPECT_EQ(*testData, *dataOut);
    EXPECT_FALSE(rbufReadBytesRemaining(dataBuf));
    EXPECT_EQ(dataBuf->writeIdx, dataBuf->readIdx);
}

uint8_t rbufferSmall[16]; // even buffer size

TEST(RingBufferTest, CycleBufferRing)
{
    rbuf_t data;
    uint8_t *start = (uint8_t *)&rbufferSmall;
    uint8_t *end = (uint8_t *)&rbufferSmall + sizeof(rbufferSmall);
    rbuf_t *dataBuf = rbufInit(&data, start, end);
    const char testData[] = "1234567"; // odd number of chars
    char dataOut[strlen(testData)];
    for (int ii=1; ii<=10; ii++) { // loop enough times to pass the boundaries of the ring buffer
        rbufWriteString(dataBuf, testData);
        EXPECT_EQ(strlen(testData), rbufReadBytesRemaining(dataBuf));
        rbufReadData(dataBuf, &dataOut, strlen(testData));
        EXPECT_EQ(0, rbufReadBytesRemaining(dataBuf));
        EXPECT_EQ(*testData, *dataOut);
    }
}

// extern "C" {


// }
