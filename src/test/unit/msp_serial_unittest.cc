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
#include <string.h>
#include <math.h>

//#define DEBUG_MSP

extern "C" {
    #include <platform.h>
    #include "build/build_config.h"
    #include "build/version.h"
    #include "build/debug.h"

    #include "common/streambuf.h"
    #include "common/utils.h"

    #include "config/parameter_group.h"
    #include "config/parameter_group_ids.h"
    #include "config/config_eeprom.h"
    #include "config/profile.h"

    #include "drivers/system.h"
    #include "drivers/sensor.h"
    #include "drivers/accgyro.h"
    #include "drivers/compass.h"
    #include "drivers/serial.h"
    #include "drivers/serial_softserial.h"
    #include "drivers/buf_writer.h"

    #include "rx/rx.h"

    #include "io/serial.h"
    #include "msp/msp.h"
    #include "msp/msp_protocol.h"
    #include "msp/msp_serial.h"

    #include "fc/runtime_config.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"


extern "C" {
    void mspSerialProcessReceivedCommand(mspPort_t *msp);
    extern mspPort_t mspPorts[];

    PG_REGISTER(serialConfig_t, serialConfig, PG_SERIAL_CONFIG, 0);
}

typedef struct mspHeader_s {
    uint8_t dollar;
    uint8_t m;
    uint8_t direction;
    uint8_t size;
    uint8_t type;
} mspHeader_t;

#define SERIAL_BUFFER_SIZE 256
typedef union mspBuffer_u {
    struct {
        mspHeader_t header;
        uint8_t payload[];
    };
    uint8_t buf[SERIAL_BUFFER_SIZE];
} mspBuffer_t;

static mspBuffer_t serialWriteBuffer;
static int serialWritePos = 0;

static mspBuffer_t serialReadBuffer;
static int serialReadPos = 0;
static int serialReadEnd = 0;

serialPort_t serialTestInstance;

void serialWrite(serialPort_t *instance, uint8_t ch)
{
    EXPECT_EQ(instance, &serialTestInstance);
    EXPECT_LT(serialWritePos, sizeof(serialWriteBuffer.buf));
    serialWriteBuffer.buf[serialWritePos++] = ch;
}

void serialWriteBuf(serialPort_t *instance, uint8_t *data, int count)
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

uint8_t serialRxBytesWaiting(serialPort_t *instance)
{
    EXPECT_EQ(instance, &serialTestInstance);
    EXPECT_GE(serialReadEnd, serialReadPos);
    int ret = serialReadEnd - serialReadPos;
    if(ret >= 0) return ret;
    return 0;
}

uint8_t serialRead(serialPort_t *instance)
{
    EXPECT_EQ(instance, &serialTestInstance);
    EXPECT_LT(serialReadPos, serialReadEnd);
    const uint8_t ch = serialReadBuffer.buf[serialReadPos++];
    return ch;
}

bool isSerialTransmitBufferEmpty(serialPort_t *instance)
{
    EXPECT_EQ(instance, &serialTestInstance);
    return true;
}

void serialTestResetBuffers()
{
    memset(&serialReadBuffer.buf, 0, sizeof(serialReadBuffer.buf));
    serialReadPos = 0;
    serialReadEnd = 0;
    memset(&serialWriteBuffer.buf, 0, sizeof(serialWriteBuffer.buf));
    serialWritePos = 0;
}

// dummy MSP command processor
#define MSP_TEST_ECHO        1
#define MSP_TEST_COMMAND     2
#define MSP_TEST_REPLY       3
#define MSP_TEST_ERROR       4

uint8_t msp_echo_data[]="PING\0PONG";
uint8_t msp_request_data[]={0xbe, 0xef};
uint8_t msp_reply_data[]={0x55,0xaa};

int mspServerCommandHandler(mspPacket_t *command, mspPacket_t *reply)
{
    sbuf_t *src = &command->buf;
    sbuf_t *dst = &reply->buf;
    int cmdLength = sbufBytesRemaining(src);
    reply->cmd = command->cmd;
    switch(command->cmd) {
        case MSP_TEST_ECHO:
            while(sbufBytesRemaining(src) > 0)
                sbufWriteU8(dst, sbufReadU8(src));
            break;
        case MSP_TEST_COMMAND:
            EXPECT_EQ(sizeof(msp_request_data), cmdLength);
            EXPECT_EQ(0, memcmp(sbufPtr(src), msp_request_data, sizeof(msp_request_data)));
            break;
        case MSP_TEST_REPLY:
            EXPECT_EQ(0, cmdLength);
            sbufWriteData(dst, msp_reply_data, sizeof(msp_reply_data));
            break;
        case MSP_TEST_ERROR:
            return -1;
    }
    return 1;
}

int mspClientReplyHandler(mspPacket_t *command, mspPacket_t *reply)
{
    UNUSED(command);
    UNUSED(reply);

    // currently untested
    return -1;
}

class SerialMspUnitTest : public ::testing::Test {
protected:
    mspPort_t *mspPort;
    virtual void SetUp() {
        mspPort = &mspPorts[0];
        mspPort->port = &serialTestInstance;
        serialTestResetBuffers();
    }
};

static uint8_t csumData(uint8_t csum, uint8_t* data, int len)
{
    while(len--)
        csum ^= *data++;
    return csum;
}

TEST_F(SerialMspUnitTest, Test_MspSerialOutFraming)
{
    mspPort->cmdMSP = MSP_TEST_REPLY;
    mspPort->dataSize = 0;
    mspSerialProcessReceivedCommand(mspPort);

    EXPECT_EQ('$', serialWriteBuffer.header.dollar);
    EXPECT_EQ('M', serialWriteBuffer.header.m);
    EXPECT_EQ('>', serialWriteBuffer.header.direction);
    EXPECT_EQ(sizeof(msp_reply_data), serialWriteBuffer.header.size);
    EXPECT_EQ(MSP_TEST_REPLY, serialWriteBuffer.header.type);
    for(unsigned i = 0; i < sizeof(msp_reply_data); i++)
        EXPECT_EQ(msp_reply_data[i], serialWriteBuffer.payload[i]);
    uint8_t checksum = sizeof(msp_reply_data) ^ MSP_TEST_REPLY;
    checksum = csumData(checksum, msp_reply_data, sizeof(msp_reply_data));
    EXPECT_EQ(checksum, serialWriteBuffer.payload[sizeof(msp_reply_data)]);  // checksum
}

TEST_F(SerialMspUnitTest, Test_TestMspSerialInFraming)
{
    uint8_t pkt[] = {'$', 'M', '<', sizeof(msp_request_data), MSP_TEST_COMMAND};

    sbuf_t pbuf = {
        .ptr = serialReadBuffer.buf,
        .end = ARRAYEND(serialReadBuffer.buf),
    };

    sbufWriteData(&pbuf, pkt, sizeof(pkt));
    sbufWriteData(&pbuf, msp_request_data, sizeof(msp_request_data));

    uint8_t csum = 0;
    csum = pkt[3] ^ pkt[4];
    csum = csumData(csum, msp_request_data, sizeof(msp_request_data));
    sbufWriteU8(&pbuf, csum);

    serialReadEnd = sbufPtr(&pbuf) - serialReadBuffer.buf;

    mspSerialProcess();

    EXPECT_EQ('$', serialWriteBuffer.header.dollar);
    EXPECT_EQ('M', serialWriteBuffer.header.m);
    EXPECT_EQ('>', serialWriteBuffer.header.direction);
    EXPECT_EQ(0, serialWriteBuffer.header.size);
    EXPECT_EQ(MSP_TEST_COMMAND, serialWriteBuffer.header.type);
    uint8_t checksum = 0 ^ MSP_TEST_COMMAND;
    EXPECT_EQ(checksum, serialWriteBuffer.payload[0]);
}

// STUBS
extern "C" {
void evaluateOtherData(serialPort_t *, uint8_t) {}
void handleOneshotFeatureChangeOnRestart(void) {}
void stopMotors(void) {}
uint8_t armingFlags = 0;
void delay(uint32_t ms) {UNUSED(ms);}
// from system_stm32fN0x.c
void systemReset(void) {}
void systemResetToBootloader(void) {}
// from serial port drivers
serialPort_t *usbVcpOpen(void) { return NULL; }
serialPort_t *uartOpen(USART_TypeDef *, serialReceiveCallbackPtr, uint32_t, portMode_t, portOptions_t) { return NULL; }
serialPort_t *openSoftSerial(softSerialPortIndex_e, serialReceiveCallbackPtr, uint32_t, portOptions_t) { return NULL; }
void serialSetMode(serialPort_t *, portMode_t) {}
bool isRebootScheduled = false;
}

