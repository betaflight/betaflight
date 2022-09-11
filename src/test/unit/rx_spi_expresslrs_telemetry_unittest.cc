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

/*
 * Based on https://github.com/ExpressLRS/ExpressLRS
 * Thanks to AlessandroAU, original creator of the ExpressLRS project.
 */

#include <stdint.h>
#include <stdbool.h>

#include <limits.h>

extern "C" {
    #include "platform.h"

    #include "build/version.h"
    #include "common/printf.h"

    #include "drivers/io.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/rx.h"

    #include "msp/msp.h"
    #include "msp/msp_serial.h"

    #include "telemetry/telemetry.h"
    #include "telemetry/msp_shared.h"
    #include "rx/crsf_protocol.h"
    #include "rx/expresslrs_telemetry.h"
    #include "flight/imu.h"

    #include "sensors/battery.h"
    #include "sensors/sensors.h"
    #include "sensors/acceleration.h"

    #include "config/config.h"

    #include "io/gps.h"

    #include "msp/msp_protocol.h"

    extern uint8_t tlmSensors;
    extern uint8_t currentPayloadIndex;

    extern volatile bool mspReplyPending;
    extern volatile bool deviceInfoReplyPending;

    bool airMode;

    uint16_t testBatteryVoltage = 0;
    int32_t testAmperage = 0;
    int32_t testmAhDrawn = 0;

    PG_REGISTER(telemetryConfig_t, telemetryConfig, PG_TELEMETRY_CONFIG, 0);
    PG_REGISTER(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 0);
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

//make clean test_rx_spi_expresslrs_telemetry_unittest
TEST(RxSpiExpressLrsTelemetryUnitTest, TestInit)
{
    initTelemetry();
    EXPECT_EQ(tlmSensors, 15);
}

static void testSetDataToTransmit(uint8_t payloadSize, uint8_t *payload)
{
    uint8_t data[ELRS_TELEMETRY_BYTES_PER_CALL] = {0};
    uint8_t maxPackageIndex = (payloadSize - 1) / ELRS_TELEMETRY_BYTES_PER_CALL;
    uint8_t nextPackageIndex;
    bool confirmValue = true;

    setTelemetryDataToTransmit(payloadSize, payload);

    for (int j = 0; j <= maxPackageIndex; j++) {
        nextPackageIndex = getCurrentTelemetryPayload(data);
        if (j != maxPackageIndex) {
            EXPECT_EQ(1 + j, nextPackageIndex);
        } else {
            EXPECT_EQ(0, nextPackageIndex); //back to start
        }
        uint8_t maxLength = (j == maxPackageIndex) ? payloadSize % ELRS_TELEMETRY_BYTES_PER_CALL : ELRS_TELEMETRY_BYTES_PER_CALL;
        for (int i = 0; i < maxLength; i++) {
            EXPECT_EQ(payload[i + j * ELRS_TELEMETRY_BYTES_PER_CALL], data[i]);
        }
        EXPECT_EQ(true, isTelemetrySenderActive());
        confirmCurrentTelemetryPayload(confirmValue);
        confirmValue = !confirmValue;
    }

    EXPECT_EQ(false, isTelemetrySenderActive());
}

TEST(RxSpiExpressLrsTelemetryUnitTest, TestGps)
{
    initTelemetry();
    currentPayloadIndex = 0;

    gpsSol.llh.lat = 56 * GPS_DEGREES_DIVIDER;
    gpsSol.llh.lon = 163 * GPS_DEGREES_DIVIDER;
    gpsSol.llh.altCm = 2345 * 100;            // altitude in cm / 100 + 1000m offset, so CRSF value should be 3345
    gpsSol.groundSpeed = 1630;                // speed in cm/s, 16.3 m/s = 58.68 km/h, so CRSF (km/h *10) value is 587
    gpsSol.numSat = 9;
    gpsSol.groundCourse = 1479;     // degrees * 10

    uint8_t *payload = 0;
    uint8_t payloadSize = 0;

    getNextTelemetryPayload(&payloadSize, &payload);
    EXPECT_EQ(currentPayloadIndex, 1);

    int32_t lattitude = payload[3] << 24 | payload[4] << 16 | payload[5] << 8 | payload[6];
    EXPECT_EQ(560000000, lattitude);
    int32_t longitude = payload[7] << 24 | payload[8] << 16 | payload[9] << 8 | payload[10];
    EXPECT_EQ(1630000000, longitude);
    uint16_t groundSpeed = payload[11] << 8 | payload[12];
    EXPECT_EQ(587, groundSpeed);
    uint16_t GPSheading = payload[13] << 8 | payload[14];
    EXPECT_EQ(14790, GPSheading);
    uint16_t altitude = payload[15] << 8 | payload[16];
    EXPECT_EQ(3345, altitude);
    uint8_t satelliteCount = payload[17];
    EXPECT_EQ(9, satelliteCount);

    testSetDataToTransmit(payloadSize, payload);
}

TEST(RxSpiExpressLrsTelemetryUnitTest, TestBattery)
{
    initTelemetry();
    currentPayloadIndex = 1;

    testBatteryVoltage = 330; // 3.3V = 3300 mv
    testAmperage = 2960; // = 29.60A = 29600mA - amperage is in 0.01A steps
    testmAhDrawn = 1234;

    uint8_t *payload = 0;
    uint8_t payloadSize = 0;

    getNextTelemetryPayload(&payloadSize, &payload);
    EXPECT_EQ(currentPayloadIndex, 2);

    uint16_t voltage = payload[3] << 8 | payload[4]; // mV * 100
    EXPECT_EQ(33, voltage);
    uint16_t current = payload[5] << 8 | payload[6]; // mA * 100
    EXPECT_EQ(296, current);
    uint32_t capacity = payload[7] << 16 | payload[8] << 8 | payload[9]; // mAh
    EXPECT_EQ(1234, capacity);
    uint16_t remaining = payload[10]; // percent
    EXPECT_EQ(67, remaining);

    testSetDataToTransmit(payloadSize, payload);
}

TEST(RxSpiExpressLrsTelemetryUnitTest, TestAttitude)
{
    initTelemetry();
    currentPayloadIndex = 2;

    attitude.values.pitch = 678; // decidegrees == 1.183333232852155 rad
    attitude.values.roll = 1495; // 2.609267231731523 rad
    attitude.values.yaw = -1799; //3.139847324337799 rad

    uint8_t *payload = 0;
    uint8_t payloadSize = 0;

    getNextTelemetryPayload(&payloadSize, &payload);
    EXPECT_EQ(currentPayloadIndex, 3);

    int16_t pitch = payload[3] << 8 | payload[4]; // rad / 10000
    EXPECT_EQ(11833, pitch);
    int16_t roll = payload[5] << 8 | payload[6];
    EXPECT_EQ(26092, roll);
    int16_t yaw = payload[7] << 8 | payload[8];
    EXPECT_EQ(-31398, yaw);

    testSetDataToTransmit(payloadSize, payload);
}

TEST(RxSpiExpressLrsTelemetryUnitTest, TestFlightMode)
{
    initTelemetry();
    currentPayloadIndex = 3;

    airMode = false;

    uint8_t *payload = 0;
    uint8_t payloadSize = 0;

    getNextTelemetryPayload(&payloadSize, &payload);
    EXPECT_EQ(currentPayloadIndex, 0);

    EXPECT_EQ('W', payload[3]);
    EXPECT_EQ('A', payload[4]);
    EXPECT_EQ('I', payload[5]);
    EXPECT_EQ('T', payload[6]);
    EXPECT_EQ('*', payload[7]);
    EXPECT_EQ(0, payload[8]);

    testSetDataToTransmit(payloadSize, payload);
}

TEST(RxSpiExpressLrsTelemetryUnitTest, TestMspVersionRequest)
{ 
    uint8_t request[15] = {238, 12, 122, 200, 234, 48, 0, 1, 1, 0, 0, 0, 0, 128, 0};
    uint8_t response[12] = {200, 10, 123, 234, 200, 48, 3, 1, 0, API_VERSION_MAJOR, API_VERSION_MINOR, 255};
    uint8_t data1[6] = {1, request[0], request[1], request[2], request[3], request[4]};
    uint8_t data2[6] = {2, request[5], request[6], request[7], request[8], request[9]};
    uint8_t data3[6] = {3, request[10], request[11], request[12], request[13], request[14]};
    uint8_t mspBuffer[15] = {0};

    uint8_t *payload = 0;
    uint8_t payloadSize = 0;

    EXPECT_EQ(CRSF_ADDRESS_CRSF_TRANSMITTER, request[0]);
    EXPECT_EQ(CRSF_FRAMETYPE_MSP_REQ, request[2]);
    EXPECT_EQ(CRSF_ADDRESS_FLIGHT_CONTROLLER, request[3]);
    EXPECT_EQ(CRSF_ADDRESS_RADIO_TRANSMITTER, request[4]);

    initTelemetry();
    initSharedMsp();

    setMspDataToReceive(15, mspBuffer);
    receiveMspData(data1[0], data1 + 1);
    receiveMspData(data2[0], data2 + 1);
    receiveMspData(data3[0], data3 + 1);
    EXPECT_FALSE(hasFinishedMspData());
    receiveMspData(0, 0);
    EXPECT_TRUE(hasFinishedMspData());

    processMspPacket(mspBuffer);
    EXPECT_TRUE(mspReplyPending);

    getNextTelemetryPayload(&payloadSize, &payload);

    EXPECT_EQ(payload[1] + 2, payloadSize);
    EXPECT_EQ(CRSF_FRAMETYPE_MSP_RESP, payload[2]);
    EXPECT_EQ(CRSF_ADDRESS_RADIO_TRANSMITTER, payload[3]);
    EXPECT_EQ(CRSF_ADDRESS_FLIGHT_CONTROLLER, payload[4]);

    for (int i = 0; i < 12; i++) {
        EXPECT_EQ(response[i], payload[i]);
    }

    testSetDataToTransmit(payloadSize, payload);
}

TEST(RxSpiExpressLrsTelemetryUnitTest, TestMspPidRequest)
{
    uint8_t pidRequest[15] = {0x00,0x0D,0x7A,0xC8,0xEA,0x30,0x00,0x70,0x70,0x00,0x00,0x00,0x00,0x69, 0x00};
    uint8_t data1[6] = {1, pidRequest[0], pidRequest[1], pidRequest[2], pidRequest[3], pidRequest[4]};
    uint8_t data2[6] = {2, pidRequest[5], pidRequest[6], pidRequest[7], pidRequest[8], pidRequest[9]};
    uint8_t data3[6] = {3, pidRequest[10], pidRequest[11], pidRequest[12], pidRequest[13], pidRequest[14]};
    uint8_t mspBuffer[15] = {0};

    uint8_t *payload = 0;
    uint8_t payloadSize = 0;

    initTelemetry();
    initSharedMsp();

    setMspDataToReceive(sizeof(mspBuffer), mspBuffer);
    receiveMspData(data1[0], data1 + 1);
    EXPECT_FALSE(hasFinishedMspData());
    receiveMspData(data2[0], data2 + 1);
    EXPECT_FALSE(hasFinishedMspData());
    receiveMspData(data3[0], data3 + 1);
    EXPECT_FALSE(hasFinishedMspData());
    receiveMspData(0, 0);
    EXPECT_TRUE(hasFinishedMspData());
    for (int i = 0; i < 15; i ++) {
        EXPECT_EQ(mspBuffer[i], pidRequest[i]);
    }
    EXPECT_FALSE(mspReplyPending);

    processMspPacket(mspBuffer);
    EXPECT_TRUE(mspReplyPending);

    getNextTelemetryPayload(&payloadSize, &payload);
    EXPECT_FALSE(mspReplyPending);

    EXPECT_EQ(payloadSize, payload[1] + 2);
    EXPECT_EQ(CRSF_FRAMETYPE_MSP_RESP, payload[2]);
    EXPECT_EQ(CRSF_ADDRESS_RADIO_TRANSMITTER, payload[3]);
    EXPECT_EQ(CRSF_ADDRESS_FLIGHT_CONTROLLER, payload[4]);
    EXPECT_EQ(0x31, payload[5]); //0x30 + 1 since it is second request, see msp_shared.c:L204
    EXPECT_EQ(0x1E, payload[6]);
    EXPECT_EQ(0x70, payload[7]);
    for (int i = 1; i <= 30; i++) {
        EXPECT_EQ(i, payload[i + 7]);
    }
    EXPECT_EQ(0x1E, payload[37]);

    testSetDataToTransmit(payloadSize, payload);
}

TEST(RxSpiExpressLrsTelemetryUnitTest, TestMspVtxRequest)
{
    uint8_t vtxRequest[15] = {0x00,0x0C,0x7C,0xC8,0xEA,0x30,0x04,0x59,0x18,0x00,0x01,0x00,0x44,0x5E, 0x00};
    uint8_t data1[6] = {1, vtxRequest[0], vtxRequest[1], vtxRequest[2], vtxRequest[3], vtxRequest[4]};
    uint8_t data2[6] = {2, vtxRequest[5], vtxRequest[6], vtxRequest[7], vtxRequest[8], vtxRequest[9]};
    uint8_t data3[6] = {3, vtxRequest[10], vtxRequest[11], vtxRequest[12], vtxRequest[13], vtxRequest[14]};
    uint8_t mspBuffer[15] = {0};

    uint8_t *payload = 0;
    uint8_t payloadSize = 0;

    initTelemetry();
    initSharedMsp();

    setMspDataToReceive(sizeof(mspBuffer), mspBuffer);
    receiveMspData(data1[0], data1 + 1);
    receiveMspData(data2[0], data2 + 1);
    receiveMspData(data3[0], data3 + 1);
    EXPECT_FALSE(hasFinishedMspData());
    receiveMspData(0, 0);
    EXPECT_TRUE(hasFinishedMspData());

    processMspPacket(mspBuffer);
    EXPECT_TRUE(mspReplyPending);

    getNextTelemetryPayload(&payloadSize, &payload);

    EXPECT_EQ(payloadSize, payload[1] + 2);
    EXPECT_EQ(CRSF_FRAMETYPE_MSP_RESP, payload[2]);
    EXPECT_EQ(CRSF_ADDRESS_RADIO_TRANSMITTER, payload[3]);
    EXPECT_EQ(CRSF_ADDRESS_FLIGHT_CONTROLLER, payload[4]);
    EXPECT_EQ(0x32, payload[5]); //0x30 + 2 since it is third request, see msp_shared.c:L204
    EXPECT_EQ(0x00, payload[6]);
    EXPECT_EQ(0x59, payload[7]);

    testSetDataToTransmit(payloadSize, payload);
}

TEST(RxSpiExpressLrsTelemetryUnitTest, TestDeviceInfoResp)
{
    uint8_t mspBuffer[15] = {0};

    uint8_t *payload = 0;
    uint8_t payloadSize = 0;

    uint8_t pingData[4] = {1, CRSF_ADDRESS_CRSF_TRANSMITTER, 1, CRSF_FRAMETYPE_DEVICE_PING};

    initTelemetry();
    initSharedMsp();

    setMspDataToReceive(sizeof(mspBuffer), mspBuffer);
    receiveMspData(pingData[0], pingData + 1);
    EXPECT_FALSE(hasFinishedMspData());
    receiveMspData(0, pingData + 1);
    EXPECT_TRUE(hasFinishedMspData());

    EXPECT_FALSE(deviceInfoReplyPending);

    processMspPacket(mspBuffer);
    EXPECT_TRUE(deviceInfoReplyPending);

    getNextTelemetryPayload(&payloadSize, &payload);
    EXPECT_FALSE(deviceInfoReplyPending);

    EXPECT_EQ(CRSF_FRAMETYPE_DEVICE_INFO, payload[2]);
    EXPECT_EQ(CRSF_ADDRESS_RADIO_TRANSMITTER, payload[3]);
    EXPECT_EQ(CRSF_ADDRESS_FLIGHT_CONTROLLER, payload[4]);
    EXPECT_EQ(0x01, payload[payloadSize - 2]);
    EXPECT_EQ(0, payload[payloadSize - 3]);

    testSetDataToTransmit(payloadSize, payload);
}

// STUBS

extern "C" {

    attitudeEulerAngles_t attitude = { { 0, 0, 0 } };     // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
    gpsSolutionData_t gpsSol;
    rssiSource_e rssiSource;
    uint8_t armingFlags;
    uint8_t stateFlags;
    uint16_t flightModeFlags;

    uint32_t microsISR(void) {return 0; }

    void beeperConfirmationBeeps(uint8_t ) {}

    uint8_t calculateBatteryPercentageRemaining(void) {return 67; }

    int32_t getAmperage(void) {return testAmperage; }

    uint16_t getBatteryVoltage(void) {return testBatteryVoltage; }

    uint16_t getLegacyBatteryVoltage(void) {return (testBatteryVoltage + 5) / 10; }

    uint16_t getBatteryAverageCellVoltage(void) {return 0; }

    batteryState_e getBatteryState(void) {return BATTERY_OK; }

    bool featureIsEnabled(uint32_t) {return true; }
    bool telemetryIsSensorEnabled(sensor_e) {return true; }
    bool sensors(uint32_t ) { return true; }

    bool airmodeIsEnabled(void) {return airMode; }

    bool isBatteryVoltageConfigured(void) { return true; }
    bool isAmperageConfigured(void) { return true; }

    const serialPortConfig_t *findSerialPortConfig(serialPortFunction_e) { return NULL;}
    serialPort_t *openSerialPort(serialPortIdentifier_e, serialPortFunction_e, serialReceiveCallbackPtr, void *, uint32_t, portMode_e, portOptions_e) { return NULL; }
    void serialWriteBuf(serialPort_t *, const uint8_t *, int) {}

    int32_t getEstimatedAltitudeCm(void) { return gpsSol.llh.altCm; }

    int32_t getMAhDrawn(void) { return testmAhDrawn; }

    bool isArmingDisabled(void) { return false; }

    mspDescriptor_t mspDescriptorAlloc(void) {return 0; }

    uint8_t mspSerialOutBuf[MSP_PORT_OUTBUF_SIZE];

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
        } else if (cmdMSP == 0x01) {
            sbufWriteU8(dst, MSP_PROTOCOL_VERSION);
            sbufWriteU8(dst, API_VERSION_MAJOR);
            sbufWriteU8(dst, API_VERSION_MINOR);
        }

        return MSP_RESULT_ACK;
    }

    timeUs_t rxFrameTimeUs(void) { return 0; }

}
