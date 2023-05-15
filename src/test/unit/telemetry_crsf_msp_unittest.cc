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
#include <string.h>

#include <limits.h>
#include <algorithm>

extern "C" {
    #include <platform.h>

    #include "build/debug.h"
    #include "build/atomic.h"

    #include "common/crc.h"
    #include "common/utils.h"
    #include "common/printf.h"
    #include "common/gps_conversion.h"
    #include "common/streambuf.h"
    #include "common/typeconversion.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/rx.h"

    #include "drivers/nvic.h"
    #include "drivers/serial.h"
    #include "drivers/system.h"

    #include "fc/runtime_config.h"
    #include "config/config.h"
    #include "flight/imu.h"

    #include "io/serial.h"
    #include "io/gps.h"

    #include "msp/msp.h"
    #include "msp/msp_serial.h"

    #include "rx/rx.h"
    #include "rx/crsf.h"

    #include "sensors/battery.h"
    #include "sensors/sensors.h"

    #include "telemetry/telemetry.h"
    #include "telemetry/msp_shared.h"
    #include "telemetry/smartport.h"
    #include "sensors/acceleration.h"

    rssiSource_e rssiSource;
    bool handleMspFrame(uint8_t *frameStart, uint8_t frameLength, uint8_t *skipsBeforeResponse);
    bool sendMspReply(uint8_t payloadSize, mspResponseFnPtr responseFn);
    uint8_t sbufReadU8(sbuf_t *src);
    int sbufBytesRemaining(sbuf_t *buf);
    void initSharedMsp();
    uint16_t testBatteryVoltage = 0;

    int32_t testAmperage = 0;
    uint8_t mspTxData[64]; // max frame size
    sbuf_t mspTxDataBuf;
    uint8_t crsfFrameOut[CRSF_FRAME_SIZE_MAX];
    uint8_t payloadOutput[64];
    sbuf_t payloadOutputBuf;
    int32_t testmAhDrawn = 0;

    PG_REGISTER(batteryConfig_t, batteryConfig, PG_BATTERY_CONFIG, 0);
    PG_REGISTER(telemetryConfig_t, telemetryConfig, PG_TELEMETRY_CONFIG, 0);
    PG_REGISTER(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 0);
    PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);
    PG_REGISTER(accelerometerConfig_t, accelerometerConfig, PG_ACCELEROMETER_CONFIG,0);

    extern bool crsfFrameDone;
    extern crsfFrame_t crsfFrame;
    extern uint8_t requestBuffer[MSP_TLM_INBUF_SIZE];
    extern struct mspPacket_s requestPacket;
    extern struct mspPacket_s responsePacket;

    uint32_t dummyTimeUs;

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

typedef struct crsfMspFrame_s {
    uint8_t deviceAddress;
    uint8_t frameLength;
    uint8_t type;
    uint8_t destination;
    uint8_t origin;
    uint8_t payload[CRSF_FRAME_RX_MSP_FRAME_SIZE + CRSF_FRAME_LENGTH_CRC];
} crsfMspFrame_t;

const uint8_t crsfPidRequest[] = {
    0x00,0x0D,0x7A,0xC8,0xEA,0x30,0x00,0x70,0x70,0x00,0x00,0x00,0x00,0x69
};

TEST(CrossFireMSPTest, RequestBufferTest)
{
    atomic_BASEPRI = 0;

    initSharedMsp();
    const crsfMspFrame_t *framePtr = (const crsfMspFrame_t*)crsfPidRequest;
    crsfFrame = *(const crsfFrame_t*)framePtr;
    crsfFrameDone = true;
    EXPECT_EQ(CRSF_ADDRESS_BROADCAST, crsfFrame.frame.deviceAddress);
    EXPECT_EQ(CRSF_FRAME_LENGTH_ADDRESS + CRSF_FRAME_LENGTH_EXT_TYPE_CRC + CRSF_FRAME_RX_MSP_FRAME_SIZE, crsfFrame.frame.frameLength);
    EXPECT_EQ(CRSF_FRAMETYPE_MSP_REQ, crsfFrame.frame.type);
    uint8_t *destination = (uint8_t *)&crsfFrame.frame.payload;
    uint8_t *origin = (uint8_t *)&crsfFrame.frame.payload + 1;
    uint8_t *frameStart = (uint8_t *)&crsfFrame.frame.payload + 2;
    uint8_t *frameEnd = (uint8_t *)&crsfFrame.frame.payload + CRSF_FRAME_RX_MSP_FRAME_SIZE + 2;
    EXPECT_EQ(0xC8, *destination);
    EXPECT_EQ(0xEA, *origin);
    EXPECT_EQ(0x30, *frameStart);
    EXPECT_EQ(0x69, *frameEnd);
}

TEST(CrossFireMSPTest, ResponsePacketTest)
{
    initSharedMsp();
    const crsfMspFrame_t *framePtr = (const crsfMspFrame_t*)crsfPidRequest;
    crsfFrame = *(const crsfFrame_t*)framePtr;
    crsfFrameDone = true;
    uint8_t *frameStart = (uint8_t *)&crsfFrame.frame.payload + 2;
    handleMspFrame(frameStart, CRSF_FRAME_RX_MSP_FRAME_SIZE, NULL);
    for (unsigned int ii=1; ii<30; ii++) {
        EXPECT_EQ(ii, sbufReadU8(&responsePacket.buf));
    }
}

//                               |   crsf                     | msp    
//                               sync size type to   from stts size fn   0    1    2    3    4
const uint8_t crsfPidWrite1[] = {0x00,0x0C,0x7A,0xC8,0xEA,0x31,0x1E,0xCA,0x29,0x28,0x1E,0x3A,0x32};
const uint8_t crsfPidWrite2[] = {0x00,0x0C,0x7A,0xC8,0xEA,0x22,0x23,0x46,0x2D,0x14,0x32,0x00,0x00};
const uint8_t crsfPidWrite3[] = {0x00,0x0C,0x7A,0xC8,0xEA,0x23,0x0F,0x00,0x00,0x22,0x0E,0x35,0x19};
const uint8_t crsfPidWrite4[] = {0x00,0x0C,0x7A,0xC8,0xEA,0x24,0x21,0x53,0x32,0x32,0x4B,0x28,0x00};
const uint8_t crsfPidWrite5[] = {0x00,0x0C,0x7A,0xC8,0xEA,0x25,0x00,0x37,0x37,0x4B,0xF8,0x00,0x00};

TEST(CrossFireMSPTest, WriteResponseTest)
{
    initSharedMsp();
    const crsfMspFrame_t *framePtr1 = (const crsfMspFrame_t*)crsfPidWrite1;
    crsfFrame = *(const crsfFrame_t*)framePtr1;
    crsfFrameDone = true;
    uint8_t *frameStart = (uint8_t *)&crsfFrame.frame.payload + 2;
    bool pending1 = handleMspFrame(frameStart, CRSF_FRAME_RX_MSP_FRAME_SIZE, NULL);
    EXPECT_FALSE(pending1); // not done yet
    EXPECT_EQ(0x29, requestBuffer[0]);
    EXPECT_EQ(0x28, requestBuffer[1]);
    EXPECT_EQ(0x1E, requestBuffer[2]);
    EXPECT_EQ(0x3A, requestBuffer[3]);
    EXPECT_EQ(0x32, requestBuffer[4]);

    const crsfMspFrame_t *framePtr2 = (const crsfMspFrame_t*)crsfPidWrite2;
    crsfFrame = *(const crsfFrame_t*)framePtr2;
    crsfFrameDone = true;
    uint8_t *frameStart2 = (uint8_t *)&crsfFrame.frame.payload + 2;
    bool pending2 = handleMspFrame(frameStart2, CRSF_FRAME_RX_MSP_FRAME_SIZE, NULL);
    EXPECT_FALSE(pending2); // not done yet
    EXPECT_EQ(0x23, requestBuffer[5]);
    EXPECT_EQ(0x46, requestBuffer[6]);
    EXPECT_EQ(0x2D, requestBuffer[7]);
    EXPECT_EQ(0x14, requestBuffer[8]);
    EXPECT_EQ(0x32, requestBuffer[9]);
    EXPECT_EQ(0x00, requestBuffer[10]);
    EXPECT_EQ(0x00, requestBuffer[11]);

    const crsfMspFrame_t *framePtr3 = (const crsfMspFrame_t*)crsfPidWrite3;
    crsfFrame = *(const crsfFrame_t*)framePtr3;
    crsfFrameDone = true;
    uint8_t *frameStart3 = (uint8_t *)&crsfFrame.frame.payload + 2;
    bool pending3 = handleMspFrame(frameStart3, CRSF_FRAME_RX_MSP_FRAME_SIZE, NULL);
    EXPECT_FALSE(pending3); // not done yet
    EXPECT_EQ(0x0F, requestBuffer[12]);
    EXPECT_EQ(0x00, requestBuffer[13]);
    EXPECT_EQ(0x00, requestBuffer[14]);
    EXPECT_EQ(0x22, requestBuffer[15]);
    EXPECT_EQ(0x0E, requestBuffer[16]);
    EXPECT_EQ(0x35, requestBuffer[17]);
    EXPECT_EQ(0x19, requestBuffer[18]);

    const crsfMspFrame_t *framePtr4 = (const crsfMspFrame_t*)crsfPidWrite4;
    crsfFrame = *(const crsfFrame_t*)framePtr4;
    crsfFrameDone = true;
    uint8_t *frameStart4 = (uint8_t *)&crsfFrame.frame.payload + 2;
    bool pending4 = handleMspFrame(frameStart4, CRSF_FRAME_RX_MSP_FRAME_SIZE, NULL);
    EXPECT_FALSE(pending4); // not done yet
    EXPECT_EQ(0x21, requestBuffer[19]);
    EXPECT_EQ(0x53, requestBuffer[20]);
    EXPECT_EQ(0x32, requestBuffer[21]);
    EXPECT_EQ(0x32, requestBuffer[22]);
    EXPECT_EQ(0x4B, requestBuffer[23]);
    EXPECT_EQ(0x28, requestBuffer[24]);
    EXPECT_EQ(0x00, requestBuffer[25]);

    const crsfMspFrame_t *framePtr5 = (const crsfMspFrame_t*)crsfPidWrite5;
    crsfFrame = *(const crsfFrame_t*)framePtr5;
    crsfFrameDone = true;
    uint8_t *frameStart5 = (uint8_t *)&crsfFrame.frame.payload + 2;
    bool pending5 = handleMspFrame(frameStart5, CRSF_FRAME_RX_MSP_FRAME_SIZE, NULL);
    EXPECT_TRUE(pending5); // not done yet
    EXPECT_EQ(0x00, requestBuffer[26]);
    EXPECT_EQ(0x37, requestBuffer[27]);
    EXPECT_EQ(0x37, requestBuffer[28]);
    EXPECT_EQ(0x4B, requestBuffer[29]);

}

void testSendMspResponse(uint8_t *payload, const uint8_t )
{
    sbuf_t *plOut = sbufInit(&payloadOutputBuf, payloadOutput, payloadOutput + 64);
    sbufWriteData(plOut, payload, *payload + 64);
    sbufSwitchToReader(&payloadOutputBuf, payloadOutput);
}

TEST(CrossFireMSPTest, SendMspReply)
{
    initSharedMsp();
    const crsfMspFrame_t *framePtr = (const crsfMspFrame_t*)crsfPidRequest;
    crsfFrame = *(const crsfFrame_t*)framePtr;
    crsfFrameDone = true;
    uint8_t *frameStart = (uint8_t *)&crsfFrame.frame.payload + 2;
    bool handled = handleMspFrame(frameStart, CRSF_FRAME_RX_MSP_FRAME_SIZE, NULL);
    EXPECT_TRUE(handled);
    bool replyPending = sendMspReply(64, &testSendMspResponse);
    EXPECT_FALSE(replyPending);
    EXPECT_EQ(0x30, sbufReadU8(&payloadOutputBuf)); // status (MSPv1 + #0)
    EXPECT_EQ(0x1E, sbufReadU8(&payloadOutputBuf)); // payload size
    EXPECT_EQ(0x70, sbufReadU8(&payloadOutputBuf)); // function ID
    for (unsigned int ii=1; ii<=30; ii++) {
        EXPECT_EQ(ii, sbufReadU8(&payloadOutputBuf));
    }
}

// STUBS

extern "C" {

    gpsSolutionData_t gpsSol;
    attitudeEulerAngles_t attitude = { { 0, 0, 0 } };
    uint8_t responseBuffer[MSP_TLM_OUTBUF_SIZE];

    uint32_t micros(void) {return dummyTimeUs;}
    uint32_t microsISR(void) {return micros();}
    serialPort_t *openSerialPort(serialPortIdentifier_e, serialPortFunction_e, serialReceiveCallbackPtr, void *, uint32_t, portMode_e, portOptions_e) {return NULL;}
    const serialPortConfig_t *findSerialPortConfig(serialPortFunction_e ) {return NULL;}
    bool isBatteryVoltageConfigured(void) { return true; }
    uint16_t getBatteryVoltage(void) {
        return testBatteryVoltage;
    }
    uint16_t getLegacyBatteryVoltage(void) {
        return (testBatteryVoltage + 5) / 10;
    }
    uint16_t getBatteryAverageCellVoltage(void) {
        return 0;
    }
    bool isAmperageConfigured(void) { return true; }
    int32_t getAmperage(void) {
        return testAmperage;
    }

    uint8_t calculateBatteryPercentageRemaining(void) {
        return 67;
    }

    int32_t getEstimatedAltitudeCm(void) {
    	return 0;
    }

    bool featureIsEnabled(uint32_t) {return false;}

    bool airmodeIsEnabled(void) {return true;}

    mspDescriptor_t mspDescriptorAlloc(void) {return 0;}

    mspResult_e mspFcProcessCommand(mspDescriptor_t srcDesc, mspPacket_t *cmd, mspPacket_t *reply, mspPostProcessFnPtr *mspPostProcessFn) {

        UNUSED(srcDesc);
        UNUSED(mspPostProcessFn);

        sbuf_t *dst = &reply->buf;
        const uint8_t cmdMSP = cmd->cmd;
        reply->cmd = cmd->cmd;

        if (cmdMSP == 0x70) {
            for (unsigned int ii=1; ii<=30; ii++) {
                sbufWriteU8(dst, ii);
            }
        } else if (cmdMSP == 0xCA) {
            return MSP_RESULT_ACK;
        }

        return MSP_RESULT_ACK;
    }

    void beeperConfirmationBeeps(uint8_t ) {}

    int32_t getMAhDrawn(void) {
      return testmAhDrawn;
    }

    bool telemetryIsSensorEnabled(sensor_e) {
        return true;
    }

    timeUs_t rxFrameTimeUs(void) { return 0; }
}
