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

extern "C" {
    #include "platform.h"
    #include "build/build_config.h"
    #include "build/debug.h"
    #include "common/time.h"
    #include "drivers/serial.h"
    #include "fc/runtime_config.h"
    #include "io/beeper.h"
    #include "io/dashboard.h"
    #include "io/gps.h"
    #include "io/serial.h"
    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/gps.h"
    #include "pg/gps_rescue.h"
    #include "scheduler/scheduler.h"

    PG_REGISTER(gpsConfig_t, gpsConfig, PG_GPS_CONFIG, 0);
    PG_REGISTER(gpsRescueConfig_t, gpsRescueConfig, PG_GPS_RESCUE, 0);
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

extern "C" uint32_t simulationMillis;

// Everything the driver writes to the (stubbed) GPS port
static uint8_t g_sentBytes[256];
static unsigned g_sentCount;

// Wrap a UBX payload in sync bytes, class, id, length and the Fletcher checksum,
// and feed it through the receive path a byte at a time.
static void feedUbxFrame(uint8_t msgClass, uint8_t msgId, const uint8_t *payload, uint16_t length)
{
    uint8_t frame[6 + 128 + 2];
    ASSERT_LE(length, 128);
    frame[0] = 0xB5;
    frame[1] = 0x62;
    frame[2] = msgClass;
    frame[3] = msgId;
    frame[4] = length & 0xFF;
    frame[5] = length >> 8;
    memcpy(&frame[6], payload, length);

    uint8_t ckA = 0, ckB = 0;
    for (unsigned i = 2; i < 6u + length; i++) {
        ckA += frame[i];
        ckB += ckA;
    }
    frame[6 + length] = ckA;
    frame[7 + length] = ckB;

    for (unsigned i = 0; i < 8u + length; i++) {
        gpsNewFrame(frame[i]);
    }
}

// UBX-CFG-GNSS (0x06 0x3E): msgVer, numTrkChHw, numTrkChUse, numConfigBlocks,
// then numConfigBlocks blocks of gnssId, resTrkCh, maxTrkCh, reserved, flags (u32)
#define CFG_GNSS_HEADER   4
#define CFG_GNSS_BLOCK    8
#define GNSS_FLAGS_ENABLED 0x00010001u  // enable + sigCfgMask

static uint16_t buildCfgGnss(uint8_t *payload, uint8_t numConfigBlocks, uint8_t declaredCount)
{
    payload[0] = 0;                 // msgVer
    payload[1] = 32;                // numTrkChHw
    payload[2] = 32;                // numTrkChUse
    payload[3] = declaredCount;
    for (uint8_t i = 0; i < numConfigBlocks; i++) {
        uint8_t *block = &payload[CFG_GNSS_HEADER + i * CFG_GNSS_BLOCK];
        block[0] = i;               // gnssId 0..7
        block[1] = 4;               // resTrkCh
        block[2] = 8;               // maxTrkCh
        block[3] = 0;
        const uint32_t flags = GNSS_FLAGS_ENABLED;
        memcpy(&block[4], &flags, sizeof(flags));
    }
    return CFG_GNSS_HEADER + numConfigBlocks * CFG_GNSS_BLOCK;
}

class GpsUbloxCfgGnssTest : public ::testing::Test {
protected:
    void SetUp() override {
        memset(&gpsSol, 0, sizeof(gpsSol));
        memset(&gpsData, 0, sizeof(gpsData));
        gpsConfigMutable()->provider = GPS_UBLOX;
        gpsConfigMutable()->sbasMode = SBAS_NONE;
        gpsConfigMutable()->gps_ublox_use_galileo = true;
        simulationMillis = 100000;
        g_sentCount = 0;
    }

    // The echoed CFG-GNSS, if one was sent: sync, class, id, length, payload, checksum
    bool sentCfgGnss(uint16_t *payloadLength) {
        if (g_sentCount < 8) {
            return false;
        }
        if (g_sentBytes[0] != 0xB5 || g_sentBytes[1] != 0x62 || g_sentBytes[2] != 0x06 || g_sentBytes[3] != 0x3E) {
            return false;
        }
        *payloadLength = g_sentBytes[4] | (g_sentBytes[5] << 8);
        return g_sentCount == 8u + *payloadLength;
    }
};

TEST_F(GpsUbloxCfgGnssTest, EightBlocksAreEchoedWithTheConfiguredFlags)
{
    uint8_t payload[CFG_GNSS_HEADER + 8 * CFG_GNSS_BLOCK];
    const uint16_t length = buildCfgGnss(payload, 8, 8);
    feedUbxFrame(0x06, 0x3E, payload, length);

    uint16_t sentLength = 0;
    ASSERT_TRUE(sentCfgGnss(&sentLength));
    EXPECT_EQ(length, sentLength);
    EXPECT_EQ(UBLOX_ACK_WAITING, gpsData.ackState);
    EXPECT_EQ(0x3E, gpsData.ackWaitingMsgId);

    const uint8_t *sent = &g_sentBytes[6];
    EXPECT_EQ(8, sent[3]) << "the block count is echoed";
    uint32_t flags;
    memcpy(&flags, &sent[CFG_GNSS_HEADER + 1 * CFG_GNSS_BLOCK + 4], sizeof(flags));
    EXPECT_EQ(0u, flags & 0x1) << "SBAS disabled, since sbasMode is SBAS_NONE";
    memcpy(&flags, &sent[CFG_GNSS_HEADER + 2 * CFG_GNSS_BLOCK + 4], sizeof(flags));
    EXPECT_EQ(1u, flags & 0x1) << "Galileo enabled";
    memcpy(&flags, &sent[CFG_GNSS_HEADER + 7 * CFG_GNSS_BLOCK + 4], sizeof(flags));
    EXPECT_EQ(GNSS_FLAGS_ENABLED, flags) << "the eighth block (NavIC) goes back as it came";
}

TEST_F(GpsUbloxCfgGnssTest, MoreBlocksThanTheStructHoldsAreNotEchoed)
{
    // A receiver with nine systems: configblocks[] holds eight, so the loop would
    // index past it, and the echo would have carried a cut-down list.
    uint8_t payload[CFG_GNSS_HEADER + 9 * CFG_GNSS_BLOCK];
    const uint16_t length = buildCfgGnss(payload, 9, 9);
    feedUbxFrame(0x06, 0x3E, payload, length);

    EXPECT_EQ(0u, g_sentCount) << "nothing is sent back";
    EXPECT_EQ(UBLOX_ACK_IDLE, gpsData.ackState) << "and nothing is waited for";
}

TEST_F(GpsUbloxCfgGnssTest, FrameShorterThanItsBlockCountIsNotEchoed)
{
    // Eight blocks declared, seven carried: the eighth block would be whatever
    // an earlier message left in the receive buffer.
    uint8_t payload[CFG_GNSS_HEADER + 7 * CFG_GNSS_BLOCK];
    const uint16_t length = buildCfgGnss(payload, 7, 8);
    feedUbxFrame(0x06, 0x3E, payload, length);

    EXPECT_EQ(0u, g_sentCount);
    EXPECT_EQ(UBLOX_ACK_IDLE, gpsData.ackState);
}

TEST_F(GpsUbloxCfgGnssTest, SevenBlocksStillWork)
{
    // What every receiver before NavIC support reports
    uint8_t payload[CFG_GNSS_HEADER + 7 * CFG_GNSS_BLOCK];
    const uint16_t length = buildCfgGnss(payload, 7, 7);
    feedUbxFrame(0x06, 0x3E, payload, length);

    uint16_t sentLength = 0;
    ASSERT_TRUE(sentCfgGnss(&sentLength));
    EXPECT_EQ(length, sentLength);
    EXPECT_EQ(UBLOX_ACK_WAITING, gpsData.ackState);
}

// STUBS
extern "C" {
uint8_t stateFlags = 0;
uint8_t armingFlags = 0;
uint8_t debugMode = 0;
int16_t debug[DEBUG16_VALUE_COUNT];
const uint32_t baudRates[] = { 0, 9600, 19200, 38400, 57600, 115200, 230400, 250000, 400000, 460800, 500000, 921600, 1000000, 1500000, 2000000, 2470000 };

uint32_t simulationMillis = 0;
uint32_t millis(void) { return simulationMillis; }
uint32_t micros(void) { return 0; }
uint32_t getCycleCounter(void) { return 0; }
uint32_t clockMicrosToCycles(uint32_t micros) { return micros; }
bool featureIsEnabled(uint32_t) { return false; }
bool sensors(uint32_t) { return false; }
void sensorsSet(uint32_t) {}
void sensorsClear(uint32_t) {}
void beeper(beeperMode_e) {}
void dashboardUpdate(timeUs_t) {}
void dashboardShowFixedPage(pageId_e) {}
void schedulerSetNextStateTime(timeDelta_t) {}
void rescheduleTask(taskId_e, timeDelta_t) {}
serialPort_t *openSerialPort(serialPortIdentifier_e, serialPortFunction_e, serialReceiveCallbackPtr, void *, uint32_t, portMode_e, portOptions_e) { return NULL; }
serialType_e serialType(serialPortIdentifier_e) { return SERIALTYPE_UART; }
baudRate_e lookupBaudRateIndex(uint32_t) { return BAUD_9600; }
uint32_t serialGetBaudRate(serialPort_t *) { return 9600; }
void serialSetBaudRate(serialPort_t *, uint32_t) {}
void serialSetMode(serialPort_t *, portMode_e) {}
void serialWrite(serialPort_t *, uint8_t c)
{
    if (g_sentCount < sizeof(g_sentBytes)) {
        g_sentBytes[g_sentCount] = c;
    }
    g_sentCount++;
}
void serialWriteBuf(serialPort_t *, const uint8_t *data, int count)
{
    for (int i = 0; i < count; i++) {
        serialWrite(NULL, data[i]);
    }
}
void serialPrint(serialPort_t *, const char *) {}
uint32_t serialRxBytesWaiting(const serialPort_t *) { return 0; }
uint8_t serialRead(serialPort_t *) { return 0; }
bool isSerialTransmitBufferEmpty(const serialPort_t *) { return true; }
void waitForSerialPortToFinishTransmitting(serialPort_t *) {}
void serialPassthrough(serialPort_t *, serialPort_t *, serialConsumer *, serialConsumer *) {}
}
