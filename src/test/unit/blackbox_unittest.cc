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
    #include "blackbox/blackbox_io.h"
    #include "common/utils.h"

    #include "config/parameter_group.h"
    #include "config/parameter_group_ids.h"

    #include "drivers/serial.h"
    extern serialPort_t *blackboxPort;

    PG_REGISTER_WITH_RESET_TEMPLATE(blackboxConfig_t, blackboxConfig, PG_BLACKBOX_CONFIG, 0);

    PG_RESET_TEMPLATE(blackboxConfig_t, blackboxConfig,
        .device = DEFAULT_BLACKBOX_DEVICE,
        .rate_num = 1,
        .rate_denom = 1,
        .on_motor_test = 0 // default off
    );
}

#include "unittest_macros.h"
#include "gtest/gtest.h"


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
    memset(&serialReadBuffer, 0, sizeof(serialReadBuffer));
    serialReadPos = 0;
    serialReadEnd = 0;
    memset(&serialWriteBuffer, 0, sizeof(serialWriteBuffer));
    serialWritePos = 0;
}

TEST(BlackboxTest, Test1)
{
    blackboxPort = &serialTestInstance;
    blackboxWriteUnsignedVB(0);
    EXPECT_EQ(0, serialWriteBuffer[0]);
    blackboxWriteUnsignedVB(128);
    EXPECT_EQ(0x80, serialWriteBuffer[1]);
    EXPECT_EQ(1, serialWriteBuffer[2]);
}

// STUBS
extern "C" {
void mspSerialAllocatePorts(void) {}
}
