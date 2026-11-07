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

// Wrap a sentence body in '$', the checksum and the line end, and feed it
// through the receive path a byte at a time. Returns whether any byte completed
// a navigation message (the '\r' does, once the checksum passes).
static bool feedNmeaSentence(const char *body)
{
    uint8_t parity = 0;
    for (const char *p = body; *p; p++) {
        parity ^= (uint8_t)*p;
    }
    char sentence[128];
    snprintf(sentence, sizeof(sentence), "$%s*%02X\r\n", body, parity);

    bool result = false;
    for (const char *p = sentence; *p; p++) {
        if (gpsNewFrame((uint8_t)*p)) {
            result = true;
        }
    }
    return result;
}

class GpsNmeaTest : public ::testing::Test {
protected:
    void SetUp() override {
        memset(&gpsSol, 0, sizeof(gpsSol));
        gpsConfigMutable()->provider = GPS_NMEA;
        stateFlags = 0;
        simulationMillis = 100000;
        // rtcSet() stores the offset from millis(), so the current time clears it
        const rtcTime_t none = simulationMillis;
        rtcSet(&none);
        ASSERT_FALSE(rtcHasTime());
    }
};

// The RTC is set by the first RMC with a date and a time after it was cleared.
TEST_F(GpsNmeaTest, RmcSetsTheRtcWithHundredthsAsMilliseconds)
{

    // hhmmss.ss and ddmmyy: 12:35:19.47 on 4 November 2026
    feedNmeaSentence("GPRMC,123519.47,A,4807.038,N,01131.000,E,022.4,084.4,041126,003.1,W");

    ASSERT_TRUE(rtcHasTime());
    dateTime_t dt;
    ASSERT_TRUE(rtcGetDateTime(&dt));
    EXPECT_EQ(2026, dt.year);
    EXPECT_EQ(11, dt.month);
    EXPECT_EQ(4, dt.day);
    EXPECT_EQ(12, dt.hours);
    EXPECT_EQ(35, dt.minutes);
    EXPECT_EQ(19, dt.seconds);
    EXPECT_EQ(470, dt.millis) << "the last two digits of the time field are hundredths";
}

TEST_F(GpsNmeaTest, GgaWithAFixFillsTheSolution)
{
    const bool navMessage = feedNmeaSentence("GPGGA,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");

    EXPECT_TRUE(navMessage);
    EXPECT_TRUE(STATE(GPS_FIX));
    EXPECT_EQ(481173000, gpsSol.llh.lat);         // 48 deg 07.038' in 1e-7 deg
    EXPECT_NEAR(115166667, gpsSol.llh.lon, 1);    // 11 deg 31.000'
    EXPECT_EQ(8, gpsSol.numSat);
    EXPECT_EQ(54540, gpsSol.llh.altCm);           // 545.4 m, cut at one decimal
}

TEST_F(GpsNmeaTest, FieldLongerThanTheBufferIsDroppedAndTheParserCarriesOn)
{
    // The field buffer holds 15 characters plus the terminator. A 16-character
    // altitude field puts the decimal point at index 14, so the one-decimal cut
    // in grab_fields() would land on index 16, one past the buffer; the bound
    // leaves the terminator at index 15 alone. The field itself is too long to
    // be a value, so it reads as 0, and the sentence that follows must parse as
    // if nothing happened.
    feedNmeaSentence("GPGGA,123519.00,4807.038,N,01131.000,E,1,08,0.9,12345678901234.5,M,46.9,M,,");
    EXPECT_EQ(0, gpsSol.llh.altCm);
    EXPECT_EQ(8, gpsSol.numSat) << "the fields before the overlong one are kept";

    feedNmeaSentence("GPGGA,123520.00,4807.038,N,01131.000,E,1,09,0.9,545.4,M,46.9,M,,");
    EXPECT_EQ(9, gpsSol.numSat);
    EXPECT_EQ(54540, gpsSol.llh.altCm);
}

TEST_F(GpsNmeaTest, SentenceWithABadChecksumChangesNothing)
{
    feedNmeaSentence("GPGGA,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
    ASSERT_EQ(54540, gpsSol.llh.altCm);

    const char *corrupt = "$GPGGA,123520.00,4807.038,N,01131.000,E,1,12,0.9,100.0,M,46.9,M,,*00\r\n";
    for (const char *p = corrupt; *p; p++) {
        gpsNewFrame((uint8_t)*p);
    }
    EXPECT_EQ(54540, gpsSol.llh.altCm);
    EXPECT_EQ(8, gpsSol.numSat);
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
void persistentObjectWrite(int, uint32_t) {}
uint32_t persistentObjectRead(int) { return 0; }
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
