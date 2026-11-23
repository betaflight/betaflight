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
#include <string.h>

#include <vector>

extern "C" {
    #include "platform.h"

    #include "telemetry/mavlink_lib.h"

    #include "build/debug.h"
    #include "io/serial.h"
    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/rx.h"
    #include "rx/rx.h"
    #include "rx/mavlink.h"
    #include "telemetry/telemetry.h"

    PG_REGISTER(telemetryConfig_t, telemetryConfig, PG_TELEMETRY_CONFIG, 0);

    int16_t debug[DEBUG16_VALUE_COUNT];
    uint8_t debugMode;

    const uint32_t baudRates[BAUD_COUNT] = {
        0, 9600, 19200, 38400, 57600, 115200, 230400, 250000,
        400000, 460800, 500000, 921600, 1000000, 1500000, 2000000, 2470000
    };

    static uint32_t fakeMicros;
    uint32_t micros(void) { return fakeMicros; }

    serialPort_t *telemetrySharedPort;
    static uint8_t telemetryBaud;
    uint8_t telemetryProviderBaud(uint8_t, serialPortIdentifier_e) { return telemetryBaud; }

    rssiSource_e rssiSource;
    linkQualitySource_e linkQualitySource;
    static uint16_t lastRssi;
    static rssiSource_e lastRssiSource;
    static uint16_t lastLinkQuality;
    void setRssi(uint16_t value, rssiSource_e source) { lastRssi = value; lastRssiSource = source; }
    void setRssiDbm(int16_t, rssiSource_e) {}
    void setRsnr(int16_t) {}
    void setLinkQualityDirect(uint16_t value) { lastLinkQuality = value; }

    static serialPort_t testPort;
    static serialReceiveCallbackPtr rxCallback;
    static void *rxCallbackData;
    static uint32_t openedBaud;
    static portMode_e openedMode;
    serialPort_t *openSerialPort(serialPortIdentifier_e, serialPortFunction_e, serialReceiveCallbackPtr callback,
        void *callbackData, uint32_t baudrate, portMode_e mode, portOptions_e)
    {
        rxCallback = callback;
        rxCallbackData = callbackData;
        openedBaud = baudrate;
        openedMode = mode;
        return &testPort;
    }
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

#define RADIO_SYS 255
#define RADIO_COMP MAV_COMP_ID_TELEMETRY_RADIO
#define FC_SYS 1
#define MIDRC 1500

class MavlinkRxTest : public ::testing::Test {
protected:
    rxConfig_t rxConfig;
    rxRuntimeState_t rxRuntimeState;

    void SetUp() override
    {
        memset(&rxConfig, 0, sizeof(rxConfig));
        memset(&rxRuntimeState, 0, sizeof(rxRuntimeState));
        rxConfig.midrc = MIDRC;
        rxConfig.rx_uart = SERIAL_PORT_USART1;

        fakeMicros = 1000;
        telemetryBaud = BAUD_AUTO;
        telemetrySharedPort = NULL;
        rssiSource = RSSI_SOURCE_NONE;
        linkQualitySource = LQ_SOURCE_NONE;
        lastRssi = 0;
        lastLinkQuality = 0;
        rxCallback = NULL;
        telemetryConfigMutable()->mavlink_min_txbuff = 35;

        mavlink_message_t drained;
        while (mavlinkGetNextQueueMessage(&drained)) {
        }

        ASSERT_TRUE(mavlinkRxInit(&rxConfig, &rxRuntimeState));
        ASSERT_NE(rxCallback, nullptr);
    }

    void feed(const mavlink_message_t *msg)
    {
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        const uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
        for (uint16_t i = 0; i < len; i++) {
            rxCallback(buf[i], rxCallbackData);
        }
    }

    void sendOverride(const uint16_t ch[18])
    {
        mavlink_message_t msg;
        mavlink_msg_rc_channels_override_pack_chan(RADIO_SYS, RADIO_COMP, MAVLINK_COMM_1, &msg, FC_SYS, 0,
            ch[0], ch[1], ch[2], ch[3], ch[4], ch[5], ch[6], ch[7], ch[8],
            ch[9], ch[10], ch[11], ch[12], ch[13], ch[14], ch[15], ch[16], ch[17]);
        feed(&msg);
    }

    void sendRadioStatus(uint8_t rssi, uint8_t remrssi, uint8_t txbuf)
    {
        mavlink_message_t msg;
        mavlink_msg_radio_status_pack_chan(RADIO_SYS, RADIO_COMP, MAVLINK_COMM_1, &msg,
            rssi, remrssi, txbuf, 0, 0, 0, 0);
        feed(&msg);
    }

    void sendHeartbeat(uint8_t sysid)
    {
        mavlink_message_t msg;
        mavlink_msg_heartbeat_pack_chan(sysid, MAV_COMP_ID_MISSIONPLANNER, MAVLINK_COMM_1, &msg,
            MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
        feed(&msg);
    }

    float readChannel(uint8_t channel)
    {
        return rxRuntimeState.rcReadRawFn(&rxRuntimeState, channel);
    }
};

TEST_F(MavlinkRxTest, InitCentresAllEighteenChannels)
{
    EXPECT_EQ(rxRuntimeState.channelCount, 18);
    for (uint8_t i = 0; i < 18; i++) {
        EXPECT_EQ(readChannel(i), MIDRC) << "channel " << (int)i;
    }
    EXPECT_EQ(rxRuntimeState.rcFrameStatusFn(&rxRuntimeState), RX_FRAME_PENDING);
}

TEST_F(MavlinkRxTest, InitOpensRxTxPortAtElrsDefaultBaud)
{
    EXPECT_EQ(openedBaud, 460800u);
    EXPECT_EQ(openedMode, MODE_RXTX);
    EXPECT_EQ(telemetrySharedPort, &testPort);
    EXPECT_EQ(rssiSource, RSSI_SOURCE_RX_PROTOCOL_MAVLINK);
    EXPECT_EQ(linkQualitySource, LQ_SOURCE_RX_PROTOCOL_MAVLINK);
}

TEST_F(MavlinkRxTest, InitUsesTelemetryProviderBaudOnSharedPort)
{
    telemetryBaud = BAUD_115200;
    ASSERT_TRUE(mavlinkRxInit(&rxConfig, &rxRuntimeState));
    EXPECT_EQ(openedBaud, 115200u);
}

TEST_F(MavlinkRxTest, InitFailsWithoutPort)
{
    rxConfig.rx_uart = SERIAL_PORT_NONE;
    EXPECT_FALSE(mavlinkRxInit(&rxConfig, &rxRuntimeState));
}

TEST_F(MavlinkRxTest, OverrideMapsAllEighteenChannelsInOrder)
{
    uint16_t ch[18];
    for (int i = 0; i < 18; i++) {
        ch[i] = 1001 + i;
    }
    sendOverride(ch);

    for (uint8_t i = 0; i < 18; i++) {
        EXPECT_EQ(readChannel(i), 1001 + i) << "channel " << (int)i;
    }
}

TEST_F(MavlinkRxTest, OverrideIgnoredValuesHoldLastValue)
{
    uint16_t ch[18];
    for (int i = 0; i < 18; i++) {
        ch[i] = 1200;
    }
    sendOverride(ch);

    ch[0] = 0;
    ch[8] = UINT16_MAX;
    ch[17] = 0;
    ch[1] = 1800;
    sendOverride(ch);

    EXPECT_EQ(readChannel(0), 1200);
    EXPECT_EQ(readChannel(8), 1200);
    EXPECT_EQ(readChannel(17), 1200);
    EXPECT_EQ(readChannel(1), 1800);
}

TEST_F(MavlinkRxTest, OverrideCompletesOneFrameAndStampsTime)
{
    uint16_t ch[18] = { 1500 };
    fakeMicros = 123456;
    sendOverride(ch);

    EXPECT_EQ(rxRuntimeState.lastRcFrameTimeUs, 123456u);
    EXPECT_EQ(rxRuntimeState.rcFrameStatusFn(&rxRuntimeState), RX_FRAME_COMPLETE);
    EXPECT_EQ(rxRuntimeState.rcFrameStatusFn(&rxRuntimeState), RX_FRAME_PENDING);
}

TEST_F(MavlinkRxTest, OverrideAndRadioStatusAreNotQueued)
{
    uint16_t ch[18] = { 1500 };
    sendOverride(ch);
    sendRadioStatus(200, 60, 80);

    mavlink_message_t msg;
    EXPECT_FALSE(mavlinkGetNextQueueMessage(&msg));
}

TEST_F(MavlinkRxTest, NoTelemetryBeforeRadioStatus)
{
    EXPECT_FALSE(isValidMavlinkTxBuffer());
    EXPECT_FALSE(shouldSendMavlinkTelemetry());
}

TEST_F(MavlinkRxTest, RadioStatusTxBufGatesTelemetry)
{
    sendRadioStatus(200, 60, 80);
    EXPECT_TRUE(isValidMavlinkTxBuffer());
    EXPECT_TRUE(shouldSendMavlinkTelemetry());

    sendRadioStatus(200, 60, 35);
    EXPECT_FALSE(shouldSendMavlinkTelemetry());

    sendRadioStatus(200, 60, 36);
    EXPECT_TRUE(shouldSendMavlinkTelemetry());
}

TEST_F(MavlinkRxTest, RadioStatusReportsRssiAndLinkQuality)
{
    sendRadioStatus(255, 0, 80);
    EXPECT_EQ(lastRssiSource, RSSI_SOURCE_RX_PROTOCOL_MAVLINK);
    EXPECT_EQ(lastRssi, RSSI_MAX_VALUE);
    EXPECT_EQ(lastLinkQuality, 100);

    sendRadioStatus(0, 130, 80);
    EXPECT_EQ(lastRssi, 0);
    EXPECT_EQ(lastLinkQuality, 0);
}

TEST_F(MavlinkRxTest, OtherMessagesQueueInOrder)
{
    sendHeartbeat(250);
    sendHeartbeat(251);

    mavlink_message_t msg;
    ASSERT_TRUE(mavlinkGetNextQueueMessage(&msg));
    EXPECT_EQ(msg.msgid, MAVLINK_MSG_ID_HEARTBEAT);
    EXPECT_EQ(msg.sysid, 250);
    ASSERT_TRUE(mavlinkGetNextQueueMessage(&msg));
    EXPECT_EQ(msg.sysid, 251);
    EXPECT_FALSE(mavlinkGetNextQueueMessage(&msg));
}

TEST_F(MavlinkRxTest, FullQueueDropsNewestMessages)
{
    for (int i = 0; i < MAVLINK_RX_QUEUE_SIZE + 4; i++) {
        sendHeartbeat(100 + i);
    }

    mavlink_message_t msg;
    int count = 0;
    while (mavlinkGetNextQueueMessage(&msg)) {
        EXPECT_EQ(msg.sysid, 100 + count);
        count++;
    }
    EXPECT_EQ(count, MAVLINK_RX_QUEUE_SIZE - 1);
}
