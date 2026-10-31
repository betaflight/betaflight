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

// Wrap a UBX payload in sync bytes, class, id, length and the Fletcher checksum,
// and feed it through the receive path a byte at a time.
static void feedUbxFrame(uint8_t msgClass, uint8_t msgId, const uint8_t *payload, uint16_t length)
{
    uint8_t frame[6 + 64 + 2];
    ASSERT_LE(length, 64);
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

// UBX-NAV-STATUS (0x01 0x03), 16 bytes: iTOW, gpsFix, flags, fixStat, flags2, ttff, msss
static void feedNavStatus(uint8_t flags2)
{
    uint8_t payload[16] = { 0 };
    payload[4] = 3;    // gpsFix: 3D
    payload[5] = 0x01; // flags: gpsFixOk
    payload[7] = flags2;
    feedUbxFrame(0x01, 0x03, payload, sizeof(payload));
}

class GpsUbloxTest : public ::testing::Test {
protected:
    void SetUp() override {
        memset(&gpsSol, 0, sizeof(gpsSol));
        gpsConfigMutable()->provider = GPS_UBLOX;
        simulationMillis = 100000;
        // spoofDetState exists from protocol 18, which the firmware gates as M8 and later
        gpsData.ubloxM7orAbove = true;
        gpsData.ubloxM8orAbove = true;
    }
};

TEST_F(GpsUbloxTest, NavStatusSpoofingStateIsTakenFromFlags2)
{
    struct {
        const char *name;
        uint8_t flags2;
        uint8_t expectedState;
        bool expectedDetected;
    } cases[] = {
        { "unknown",            0x00, GPS_SPOOFING_UNKNOWN,   false },
        { "no spoofing",        0x08, GPS_SPOOFING_NONE,      false },
        { "spoofing indicated", 0x10, GPS_SPOOFING_INDICATED, true },
        { "multiple",           0x18, GPS_SPOOFING_MULTIPLE,  true },
        // the neighbouring psmState and carrSoln bits must not leak into the state
        { "indicated, other bits set", 0x10 | 0x03 | 0xC0, GPS_SPOOFING_INDICATED, true },
        { "none, other bits set",      0x08 | 0x03 | 0xC0, GPS_SPOOFING_NONE,      false },
    };

    for (const auto &c : cases) {
        memset(&gpsSol, 0, sizeof(gpsSol)); // each case stands alone: no hold carried over from the last
        feedNavStatus(c.flags2);
        EXPECT_EQ(c.expectedState, gpsSol.spoofingState) << c.name;
        EXPECT_EQ(c.expectedDetected, gpsSpoofingDetected()) << c.name;
    }
}

TEST_F(GpsUbloxTest, SpoofingIsHeldAfterTheLastIndication)
{
    // The receiver's detector fires at the transition into spoofing and then reports
    // "no spoofing" for the epochs that follow, so the warning must outlast the flag.
    feedNavStatus(0x10);
    EXPECT_TRUE(gpsSpoofingDetected());

    simulationMillis += 1000;
    feedNavStatus(0x08);
    EXPECT_EQ(GPS_SPOOFING_NONE, gpsSol.spoofingState);
    EXPECT_TRUE(gpsSpoofingDetected()) << "still held one second after the indication";

    simulationMillis += 8000;
    feedNavStatus(0x08);
    EXPECT_TRUE(gpsSpoofingDetected()) << "still held nine seconds after the indication";

    simulationMillis += 2000;
    feedNavStatus(0x08);
    EXPECT_FALSE(gpsSpoofingDetected()) << "released eleven seconds after the indication";
}

TEST_F(GpsUbloxTest, IndicationExpiresWhenNavStatusStopsArriving)
{
    // NAV-PVT may carry on after the receiver stops sending NAV-STATUS; the last
    // indication must then age out rather than stay latched on the stored state.
    feedNavStatus(0x10);
    EXPECT_TRUE(gpsSpoofingDetected());
    EXPECT_EQ(GPS_SPOOFING_INDICATED, gpsSol.spoofingState);

    simulationMillis += 9000;
    EXPECT_TRUE(gpsSpoofingDetected()) << "still within the hold";

    simulationMillis += 2000;
    EXPECT_FALSE(gpsSpoofingDetected()) << "expired with no further report";
    EXPECT_EQ(GPS_SPOOFING_INDICATED, gpsSol.spoofingState) << "the stored state is what the receiver last said";
}

TEST_F(GpsUbloxTest, ShortNavStatusFrameDoesNotUpdateTheState)
{
    feedNavStatus(0x08);
    ASSERT_EQ(GPS_SPOOFING_NONE, gpsSol.spoofingState);

    // A valid-checksum NAV-STATUS with only 4 payload bytes: flags2 would be read from
    // whatever the previous frame left at offset 7.
    const uint8_t shortPayload[4] = { 0 };
    feedUbxFrame(0x01, 0x03, shortPayload, sizeof(shortPayload));
    EXPECT_EQ(GPS_SPOOFING_NONE, gpsSol.spoofingState);
    EXPECT_FALSE(gpsSpoofingDetected());
}

TEST_F(GpsUbloxTest, ReservedBitsOnAnOlderReceiverAreNotASpoofingState)
{
    // u-blox 7 defines flags2 with psmState only; bits 3..4 are reserved there and must
    // never be read as "spoofing indicated"
    gpsData.ubloxM8orAbove = false;
    feedNavStatus(0x18);
    EXPECT_EQ(GPS_SPOOFING_UNKNOWN, gpsSol.spoofingState);
    EXPECT_FALSE(gpsSpoofingDetected());
    EXPECT_EQ(0u, gpsSol.spoofingIndicatedAtMs);
}

TEST_F(GpsUbloxTest, NoSpoofingReportWithoutAnIndicationIsNotHeld)
{
    feedNavStatus(0x08);
    EXPECT_FALSE(gpsSpoofingDetected());
    EXPECT_EQ(0u, gpsSol.spoofingIndicatedAtMs);
}

TEST_F(GpsUbloxTest, CorruptNavStatusIsIgnored)
{
    feedNavStatus(0x08);
    ASSERT_EQ(GPS_SPOOFING_NONE, gpsSol.spoofingState);

    // Same frame with the spoofing bits set but a bad checksum
    uint8_t frame[6 + 16 + 2] = { 0xB5, 0x62, 0x01, 0x03, 16, 0 };
    frame[6 + 4] = 3;
    frame[6 + 5] = 0x01;
    frame[6 + 7] = 0x10;
    frame[22] = 0x00; // wrong ckA
    frame[23] = 0x00; // wrong ckB
    for (unsigned i = 0; i < sizeof(frame); i++) {
        gpsNewFrame(frame[i]);
    }
    EXPECT_EQ(GPS_SPOOFING_NONE, gpsSol.spoofingState) << "a frame that fails its checksum changes nothing";
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
void serialWrite(serialPort_t *, uint8_t) {}
void serialWriteBuf(serialPort_t *, const uint8_t *, int) {}
void serialPrint(serialPort_t *, const char *) {}
uint32_t serialRxBytesWaiting(const serialPort_t *) { return 0; }
uint8_t serialRead(serialPort_t *) { return 0; }
bool isSerialTransmitBufferEmpty(const serialPort_t *) { return true; }
void waitForSerialPortToFinishTransmitting(serialPort_t *) {}
void serialPassthrough(serialPort_t *, serialPort_t *, serialConsumer *, serialConsumer *) {}
}
