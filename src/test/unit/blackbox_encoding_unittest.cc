/*
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
#include <string.h>

extern "C" {
    #include "platform.h"

    #include "blackbox/blackbox.h"
    #include "blackbox/blackbox_encoding.h"
    #include "common/utils.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"

    #include "drivers/serial.h"
    #include "io/serial.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"


static serialPort_t *blackboxPort;
static int serialWritePos = 0;
static int serialReadPos = 0;
static int serialReadEnd = 0;
#define SERIAL_BUFFER_SIZE 256
static uint8_t serialReadBuffer[SERIAL_BUFFER_SIZE];
static uint8_t serialWriteBuffer[SERIAL_BUFFER_SIZE];

serialPort_t serialTestInstance;

void serialWrite(serialPort_t *instance, uint8_t ch)
{
    EXPECT_EQ(instance, &serialTestInstance);
    EXPECT_LT(serialWritePos, sizeof(serialWriteBuffer));
    serialWriteBuffer[serialWritePos++] = ch;
}

void serialWriteBuf(serialPort_t *instance, const uint8_t *data, int count)
{
    while(count--)
        serialWrite(instance, *data++);
}

void serialBeginWrite(serialPort_t *instance)
{
    EXPECT_EQ(instance, &serialTestInstance);
}

void serialEndWrite(serialPort_t *instance)
{
    EXPECT_EQ(instance, &serialTestInstance);
}

uint32_t serialRxBytesWaiting(const serialPort_t *instance)
{
    EXPECT_EQ(instance, &serialTestInstance);
    EXPECT_GE(serialReadEnd, serialReadPos);
    int ret = serialReadEnd - serialReadPos;
    if (ret >= 0) return ret;
    return 0;
}

uint8_t serialRead(serialPort_t *instance)
{
    EXPECT_EQ(instance, &serialTestInstance);
    EXPECT_LT(serialReadPos, serialReadEnd);
    const uint8_t ch = serialReadBuffer[serialReadPos++];
    return ch;
}

uint32_t serialTxBytesFree(const serialPort_t *instance)
{
    UNUSED(instance);
    return SERIAL_BUFFER_SIZE - serialWritePos;
}

bool isSerialTransmitBufferEmpty(const serialPort_t *instance)
{
    EXPECT_EQ(instance, &serialTestInstance);
    return true;
}

void serialTestResetBuffers()
{
    blackboxPort = &serialTestInstance;
    memset(&serialReadBuffer, 0, sizeof(serialReadBuffer));
    serialReadPos = 0;
    serialReadEnd = 0;
    memset(&serialWriteBuffer, 0, sizeof(serialWriteBuffer));
    serialWritePos = 0;
}

TEST(BlackboxEncodingTest, TestWriteUnsignedVB)
{
    serialTestResetBuffers();

    blackboxWriteUnsignedVB(0);
    EXPECT_EQ(0, serialWriteBuffer[0]);
    blackboxWriteUnsignedVB(128);
    EXPECT_EQ(0x80, serialWriteBuffer[1]);
    EXPECT_EQ(1, serialWriteBuffer[2]);
}

TEST(BlackboxTest, TestWriteTag2_3SVariable_BITS2)
{
    serialTestResetBuffers();
    uint8_t *buf = &serialWriteBuffer[0];
    int selector;
    int32_t v[3];

    // 2 bits per field  ss11 2233,
    v[0] = 0;
    v[1] = 0;
    v[2] = 0;
    selector = blackboxWriteTag2_3SVariable(v);
    EXPECT_EQ(0, selector);
    EXPECT_EQ(0, buf[0]);
    EXPECT_EQ(0, buf[1]); // ensure next byte has not been written
    ++buf;

    v[0] = 1;
    selector = blackboxWriteTag2_3SVariable(v);
    EXPECT_EQ(0x10, buf[0]); // 00010000
    EXPECT_EQ(0, buf[1]); // ensure next byte has not been written
    ++buf;

    v[0] = 1;
    v[1] = 1;
    v[2] = 1;
    selector = blackboxWriteTag2_3SVariable(v);
    EXPECT_EQ(0, selector);
    EXPECT_EQ(0x15, buf[0]); // 00010101
    EXPECT_EQ(0, buf[1]); // ensure next byte has not been written
    ++buf;

    v[0] = -1;
    v[1] = -1;
    v[2] = -1;
    selector = blackboxWriteTag2_3SVariable(v);
    EXPECT_EQ(0, selector);
    EXPECT_EQ(0x3F, buf[0]); // 00111111
    EXPECT_EQ(0, buf[1]); // ensure next byte has not been written
    ++buf;

    v[0] = -2;
    v[1] = -2;
    v[2] = -2;
    selector = blackboxWriteTag2_3SVariable(v);
    EXPECT_EQ(0, selector);
    EXPECT_EQ(0x2A, buf[0]); // 00101010
    EXPECT_EQ(0, buf[1]); // ensure next byte has not been written
    ++buf;
}

TEST(BlackboxTest, TestWriteTag2_3SVariable_BITS554)
{
    serialTestResetBuffers();
    uint8_t *buf = &serialWriteBuffer[0];
    int selector;
    int32_t v[3];

    // 554 bits per field  ss11 1112 2222 3333
    // 5 bits per field [-16, 15], 4 bits per field [-8, 7]
    v[0] = 15;
    v[1] = 15;
    v[2] = 7;
    selector = blackboxWriteTag2_3SVariable(v);
    EXPECT_EQ(1, selector);
    EXPECT_EQ(0x5E, buf[0]); // 0101 1110
    EXPECT_EQ(0xF7, buf[1]); // 1111 0111
    EXPECT_EQ(0, buf[2]); // ensure next byte has not been written
    buf += 2;

    v[0] = -16;
    v[1] = -16;
    v[2] = -8;
    selector = blackboxWriteTag2_3SVariable(v);
    EXPECT_EQ(1, selector);
    EXPECT_EQ(0x61, buf[0]); // 0110 0001
    EXPECT_EQ(0x08, buf[1]); // 0000 1000
    EXPECT_EQ(0, buf[2]); // ensure next byte has not been written
    buf += 2;

    v[0] = 7;
    v[1] = 8;
    v[2] = 5;
    selector = blackboxWriteTag2_3SVariable(v);
    EXPECT_EQ(1, selector);
    EXPECT_EQ(0x4E, buf[0]); // 0100 1110
    EXPECT_EQ(0x85, buf[1]); // 1000 0101
    EXPECT_EQ(0, buf[2]); // ensure next byte has not been written
    buf += 2;
}

TEST(BlackboxTest, TestWriteTag2_3SVariable_BITS887)
{
    serialTestResetBuffers();
    uint8_t *buf = &serialWriteBuffer[0];
    int selector;
    int32_t v[3];

    // 877 bits per field  ss11 1111 1122 2222 2333 3333
    // 8 bits per field [-128, 127], 7 bits per field [-64, 63]
    v[0] = 127;
    v[1] = 63;
    v[2] = 63;
    selector = blackboxWriteTag2_3SVariable(v);
    EXPECT_EQ(2, selector);
    EXPECT_EQ(0x9F, buf[0]); // 1001 1111
    EXPECT_EQ(0xDF, buf[1]); // 1101 1111
    EXPECT_EQ(0xBF, buf[2]); // 1011 1111
    EXPECT_EQ(0, buf[3]); // ensure next byte has not been written
    buf += 3;

    v[0] = -128;
    v[1] = -64;
    v[2] = -64;
    selector = blackboxWriteTag2_3SVariable(v);
    EXPECT_EQ(2, selector);
    EXPECT_EQ(0xA0, buf[0]); // 1010 0000
    EXPECT_EQ(0x20, buf[1]); // 0010 0000
    EXPECT_EQ(0x40, buf[2]); // 0100 0000
    EXPECT_EQ(0, buf[3]); // ensure next byte has not been written
    buf += 3;
}
// STUBS
extern "C" {
PG_REGISTER(blackboxConfig_t, blackboxConfig, PG_BLACKBOX_CONFIG, 0);
int32_t blackboxHeaderBudget;
void mspSerialAllocatePorts(void) {}
void blackboxWrite(uint8_t value) {serialWrite(blackboxPort, value);}
int blackboxWriteString(const char *s)
{
    const uint8_t *pos = (uint8_t*)s;
    while (*pos) {
        serialWrite(blackboxPort, *pos);
        pos++;
    }
    const int length = pos - (uint8_t*)s;
    return length;
}
}
